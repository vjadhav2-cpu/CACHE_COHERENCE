# L2 to TLC Crossbar Integration Plan (Draft)

## Date

July 27, 2026

## Branch

`feature/l2-tlc-xbar-integration`

## Goal

Connect `cache_system/rv64g_l2_cache.v`'s memory-side port onto the `tlc/`
crossbar (`tl_xbar_nm.v`) as a second master alongside the DMA, so L2
refill/writeback traffic and DMA traffic share the same fabric and the same
downstream memory manager instead of L2 talking to memory directly.

## What Already Exists (Reusable)

- `tlc/tl_xbar_nm.v` already defaults to `N_MASTERS = 3`.
- `tlc/tlc_xbar_fabric_top.v` instantiates one `l1_tilelink_adapter` per
  master slot (`u_l1_m0`, `u_l1_m1`, `u_l1_m2` — see
  `tlc_xbar_fabric_top.v:292,353,414`), each feeding `tl_xbar_nm`.
- In `tlc/tlc_dma_system_with_wrapper_top.v`, the DMA occupies **master
  slot 2**: `dma_tlc_master_wrapper` drives `m2_req_*` straight into
  `u_l1_m2`. Slots **m0 and m1 are exposed at the top level and are
  genuinely unused** — `tlc/tb_tlc_dma_system_basic_v3.v:61-92` ties
  `m0_req_valid`/`m1_req_valid` and both probe-ack buses to `0`.
- `rv64g_l2_cache.v`'s memory-side port (`mem_a_*_o` / `mem_d_*_i`,
  `rv64g_l2_cache.v:58-78`) is already TL-UH shaped and uses the same
  opcode encoding as `tlc/tlc64b2M_params.v`: `Get` (`3'd4`) for refill,
  `PutFullData` (`3'd0`) for writeback, `AccessAck` for the response.

On the surface this looks like a drop-in: free slot + matching opcodes.
It isn't, for one reason below.

## The Critical Mismatch

`rv64g_l2_fsm.v` does **real 8-beat bursts** on the mem port:

- `ST_MEM_READ` streams 8 beats into `mem_d_data_i` (`burst_cnt` 0..7) per
  `Get`, filling a 64-byte line (`rv64g_l2_fsm.v:583-602`).
- `ST_MEM_WRITE` streams 8 beats out on `mem_a_data_o` per `PutFullData`
  writeback (`rv64g_l2_fsm.v:606-621`).

`l1_tilelink_adapter.v` — the thing every existing master slot goes
through — only ever moves **one 64-bit beat per transaction**. Its
`CACHE_LINE_SIZE` is `$clog2(256/8) = 5`, i.e. it was sized for a 32-byte
notion of a line, and the uncached path literally comments "Only first
beat for now" (`l1_tilelink_adapter.v:411`) and zero-pads a single D beat
into the 256-bit `data_to_l1_data` (`l1_tilelink_adapter.v:304,434`). It
was built for the DMA's single-beat `mem_req`/`mem_ack` traffic and was
never extended for genuine bursts.

So dropping L2 onto `m0` the same way `tlc/dma_tlc_bridge.v` drops the DMA
onto `m2` does **not** work out of the box — it would silently truncate
every 64-byte L2 refill/writeback to 1 of its 8 words.

## Two Ways To Resolve It

### Option A — Loop 8 single-beat transactions through the existing path

Write `l2_tlc_bridge.v` that, per L2 line access, issues 8 sequential
single-beat `Get`/`PutFullData` requests through `m0`'s existing
`l1_tilelink_adapter`, one per word, incrementing address by 8 bytes each
time, and feeds each returned/acked beat back into L2's
`mem_d_data_i`/`mem_a_ready_i` timing.

Pros:
- No changes to shared `tlc/` infra (`l1_tilelink_adapter`, `tl_xbar_nm`,
  `tlc_xbar_fabric_top` all untouched).
- Mirrors the "v1 simplification" precedent already used for the DMA
  bridge (single outstanding request, no bursts, documented as v1-only).

Cons:
- 8x the transaction/arbitration overhead per line versus one real burst.
- Relies on the same undocumented "this port is never probed" assumption
  `dma_tlc_bridge.v`'s header flags for the DMA — now for L2 refill
  traffic, which is much more probe-sensitive than DMA traffic (see Open
  Question below).

### Option B — Give L2 a true burst-capable master port

Extend `l1_tilelink_adapter.v` (or add a sibling adapter) to actually
stream `a_size`/`d_size`-driven multi-beat bursts — `tl_xbar_nm`'s raw A/D
channels are already generic, size-aware TileLink and can carry this; the
limitation is purely in the adapter's controller-facing abstraction. Wire
it to a master slot via a thin `l2_tlc_bridge.v` that packs the 8 beats
around the adapter's existing FSM.

Pros:
- Architecturally correct; fixes a real limitation for any future master
  needing cache-line-granularity traffic, not just L2.
- One burst instead of 8 arbitration rounds.

Cons:
- Touches shared infra (`l1_tilelink_adapter.v`, `tlc_xbar_fabric_top.v`)
  that the DMA's existing testbench also depends on — bigger blast
  radius, requires re-running `tb_tlc_dma_system_basic_v3.v` to confirm
  the DMA path is unaffected.

## Recommendation

Start with **Option A** for a first working PR — small blast radius,
proves the wiring/arbitration end-to-end without touching code the DMA
path already depends on. File Option B as explicit follow-up work, same
"v1 simplification, revisit later" pattern `docs/dma_tlc_bridge_plan.md`
used for the DMA bridge.

## Other Mismatches To Handle (mirrors the DMA plan)

- **Source ID width**: `rv64g_l2_cache` uses `SOURCE_W = 6`; `tlc/`'s
  `M_SOURCE_W = SOURCE_W(4) + MST_ID_W(2) = 6`. These happen to line up —
  confirm this is not coincidental before wiring `tl_a_source_i` straight
  across.
- **Address width**: L2's `mem_a_address_o` is 64-bit
  (`rv64g_l2_cache` `ADDR_W = 64`); `tlc_xbar_fabric_top`'s `m0_req_addr`
  is 32-bit. Same truncation question the DMA plan already raised for its
  own path — needs an explicit policy, not a silent truncation.

## Open Question — Probe Safety

`dma_tlc_bridge.v`'s header documents that its uncached port is "never a
legitimate probe target" only because the DMA doesn't hold cache lines.
L2 is different: it **is** a coherence manager for its own L1s, and its
mem-side traffic represents real dirty/clean line movement. Before
building the bridge, confirm `tlc_slave_mem_manager.v` genuinely never
needs to probe this port (i.e. it doesn't track `m0` as an owner) — if it
does, Option A's borrowed assumption from the DMA bridge doesn't hold and
B-channel handling can't be skipped.

## Next Steps

1. Resolve the probe-safety open question above.
2. Confirm Option A vs Option B.
3. Build `l2_tlc_bridge.v`.
4. Build a new system top (e.g. `tlc_dma_l2_shared_mem_top.v`)
   instantiating `rv64g_l2_cache` + `l2_tlc_bridge` alongside the existing
   DMA + fabric + slave-manager pieces from
   `tlc_dma_system_with_wrapper_top.v`.
5. Extend a `tb_tlc_dma_system_basic_v3.v`-style testbench to drive DMA
   and L2 memory traffic concurrently and check both land correctly in
   shared memory. This also fills the separately-noted gap that
   `cache_system/` currently has no testbench at all.

## Results (Option A implemented and verified)

Probe safety was confirmed by reading `tlc_slave_mem_manager.v` directly:
its `S_D_SEND` state clears `dir_v`/`dir_pres` for `Get`/`PutFullData`
requests instead of recording ownership — only `AcquireBlock`/
`AcquirePerm` register a requester in `dir_pres`. Since `l2_tlc_bridge.v`
never issues those opcodes, `m0` can never be probed.

### What was built

- `tlc/l2_tlc_bridge.v` — decomposes each of L2's 8-beat line
  accesses into 8 single-beat Get/PutFullData round trips through
  `l1_tilelink_adapter`, per Option A.
- `tlc/tlc_l2_dma_shared_mem_top.v` — new system top putting
  `rv64g_l2_cache` + `l2_tlc_bridge` on master slot `m0`, alongside the
  existing DMA on `m2`, sharing `tlc_xbar_fabric_top` and the `slv0`
  memory manager/backend.
- `tlc/tb_l2_tlc_bridge_standalone.v` — isolated test of the bridge
  against the real crossbar/slave-manager/memory (8-beat write, then
  8-beat read-back). **Passes.**
- `tlc/tb_tlc_l2_dma_shared_mem.v` — full-stack test: a real
  `AcquireBlock` into `rv64g_l2_cache` refills through `m0` and is
  confirmed to reach shared memory (`slv0_total_rd_count`), then the
  DMA's own write/read sequence is run on `m2` to confirm it still works
  with L2 present. **Passes.**

### Bugs found and fixed along the way (all confirmed by simulation, not just inspection)

1. **`cache_system/rv64g_l2_plru.v` and `rv64g_l2_directory.v`**: reset
   loops used a non-blocking assignment inside a `for` loop
   (`plru_bits_q[si] <= ...`, `ram[i] <= ...`), which Verilator rejects
   (`BLKLOOPINIT`). Never caught before because `cache_system/` had no
   testbench. Fixed by using a blocking assignment in the reset branch
   only (behaviorally identical, standard fix for this exact limitation).
2. **`tlc/l1_tilelink_adapter.v`**: `STATE_UNCACHED_RSP_WAIT` only pulsed
   `data_to_l1_valid` for `AccessAckData` (reads). A plain `AccessAck`
   (write completion) never pulsed it, and the adapter's own
   `l1_request_ready` does not reassert on its own once idle either (it
   needs a fresh `l1_request_valid` to regrant a source ID first) — so an
   uncached write had **no** externally observable completion signal at
   all. This is also why `dma_tlc_bridge.v`'s documented technique
   ("ready reasserting means done") doesn't actually work; that module is
   dead code, never instantiated anywhere, so nothing had exercised this
   path before. Fixed by pulsing `data_to_l1_valid` on plain `AccessAck`
   too (data driven to zero).
3. **`tlc/l2_tlc_bridge.v`** (own bug, caught before this got committed):
   the D-channel output (`mem_d_valid_o` etc.) was a registered signal
   that asserted to 1 and, in the same always-block evaluation, could be
   cancelled back to 0 by `if (mem_d_ready_i)` — since `rv64g_l2_fsm.v`
   (and this bridge's own testbenches) hold `mem_d_ready_o` high
   continuously, this meant the signal never actually appeared as a real
   `1` for a synthesizable clock cycle. Fixed by deriving it
   combinationally from `state`, matching the pattern already used
   correctly for `mem_a_ready_o`.
4. Confirmed **pre-existing, unrelated to this branch**: the existing
   `tlc/tb_tlc_dma_system_basic_v3.v` fails its own "Test 5" sink-data
   check under this Verilator version (5.020), independent of every
   change above (reproduced on a clean stash of this branch's diffs). The
   underlying data movement is correct (`slv0_total_wr_count`/
   `total_rd_count` match the totals documented in
   `docs/tlc_dma_memory_flow_test.md`); the check itself
   (`sink_seen[base_idx+n]`, a variable bit-index write into a packed
   vector) is the same failure pattern found and fixed in this branch's
   own new testbenches (replaced with an unpacked per-bit array). Not
   fixed here since it's outside this branch's stated scope — flagged as
   a separate follow-up.

### Still open (tracked as follow-up, not done here)

- Option B (real burst-capable adapter) — not attempted; Option A's
  8-beat decomposition is what's implemented and tested.
- Dirty-eviction writeback from L2's own coherence FSM (as opposed to the
  bridge's write path, which *is* tested directly) was not exercised —
  doing so needs a real multi-way eviction sequence through
  `rv64g_l2_fsm.v`'s own directory/PLRU logic, which is a separate,
  deeper question about `cache_system`'s own coherence correctness, not
  the crossbar integration this branch is about.
- The pre-existing `tb_tlc_dma_system_basic_v3.v` Test 5 failure above.

## Phase 2 — DMA/L2 Coherency (in progress)

With the crossbar integration in place, L2 and the DMA are two independent
masters sharing the same memory but each with their own private notion of
coherence: L2's is `rv64g_l2_directory.v`, tracking its own L1 core
sharers/owners; the DMA's is inside `tlc_slave_mem_manager.v` at the
crossbar boundary (see below). Neither has visibility into the other.
This section tracks characterizing that gap and choosing a fix, from the
DMA's perspective (per test plan: DMA writes to memory for a peripheral,
does a core with that line cached see it?).

### Step 1 result — hazard characterized (`tlc/tb_dma_l2_coherency_gap.v`)

Test sequence: (1) L2 issues a real `AcquireBlock(NtoB)` on its upstream
port for `addr 0x3000`, caching 8 beats of fresh (zero) data; (2) DMA
writes a new pattern (`0xFEED_FACE...`) to that same address,
peripheral -> memory, entirely through its own path; (3) DMA reads the
address back independently, confirming memory now holds the new data;
(4) L2 re-`Acquire`s the same address.

Result: **TEST PASSED (hazard confirmed as predicted)**. L2 served all 8
beats of step 4 from its own cache, identical to the step-1 (pre-DMA)
values — not the DMA's new data, which step 3 independently proved
memory now holds. `slv0_total_rd_count` did not increase during step 4,
confirming this was a genuine cache hit, not a lucky re-fetch. A real
core reading through L2 after a DMA write to a line L2 already holds
would silently see stale data indefinitely.

(Note: the test's memory-traffic counter baseline has to be captured
*after* step 3, not after step 1 — the DMA's own write path is itself
Acquire+modify+Release, so it performs a real memory read per beat as
part of taking ownership, which would otherwise be misattributed to
step 4 and produce a false "L2 re-fetched" verdict.)

### Step 2 — choosing a fix: the crossbar already has a directory

`tlc_slave_mem_manager.v` is not a plain memory-side ack generator — it
has its own per-line MESI-ish directory (`dir_v`/`dir_tag`/`dir_coh`/
`dir_pres`, `DIR_DEPTH=32`) and a real probe-issuing FSM:

- `S_DIR_LOOKUP` (`tlc_slave_mem_manager.v:330`) computes which masters
  need probing from `dir_pres`, excluding the requester for an
  exclusive-seeking Acquire and including the current owner even for a
  shared Acquire against a Modified line (`probes_excl`/`probes_shared`,
  lines 180-185).
- `S_PROBE_SEND`/`S_PROBE_WAIT` (lines 362-427) issue real B-channel
  `Probe`s to every master `dir_pres` names, priority-encoded
  round-robin, and block on `ProbeAck`/`ProbeAckData` — folding a dirty
  `ProbeAckData` into the pending memory write before continuing.

This is a fully general, already-working coherence engine — it is just
not currently *used* for L2's traffic. `dir_pres` is only ever set on
`AcquireBlock`/`AcquirePerm` (`pending_dir_update`, lines 342/353/464).
Plain `Get`/`PutFullData` — what `l2_tlc_bridge.v` issues under Option A
— instead *clears* any existing presence bit for that line with no probe
first (lines 474-479). So L2's mem-side traffic is invisible to this
directory by construction; only the DMA (the sole Acquire-issuing
master today) is tracked.

Two candidate fixes were compared:

**(a) Make DMA a coherent client of L2's own upstream port** (DMA
"becomes another core"). Confirmed via `rv64g_l2_cache.v`'s port list
that its upstream `tl_a`/`tl_b`/`tl_c`/`tl_d`/`tl_e` bus is a single
shared, source-tagged bus (`SOURCE_W=6`), not per-core physical ports —
so no new arbiter is needed for this repo's current single-agent test
scope. But this still requires: rewriting/adapting `dma_coherent_agent.v`
(currently speaks the crossbar's `l1_request_*`/`probe_req_to_l1_*`
abstraction via `l1_tilelink_adapter`) to instead drive raw
`tl_a`/`tl_c`/`tl_e` and consume `tl_b`/`tl_d` against L2 directly;
claiming/expanding an owner-ID slot in `rv64g_l2_directory.v`'s
`CORES=4`-wide sharer/owner fields; retiring `m2`/`tlc_slave_mem_manager`
from the DMA-to-memory dataflow entirely; and restructuring both
`tb_tlc_l2_dma_shared_mem.v` and `tb_dma_l2_coherency_gap.v`, whose
current designs assume DMA and L2 are independent stimulus paths.

**(b) Make L2's mem-side traffic coherent at the crossbar, reusing
`tlc_slave_mem_manager`'s existing directory** — have `l2_tlc_bridge.v`
issue `AcquireBlock`/`AcquirePerm` + `Release`/`ReleaseData` instead of
`Get`/`PutFullData`. Then:
- `dir_pres` starts tracking L2 (`m0`) as an owner exactly like it
  already tracks DMA (`m2`) — zero changes to the directory/probe FSM
  itself, it already generalizes over `N_MASTERS`.
- When DMA Acquires a line L2 holds, `S_DIR_LOOKUP` already computes
  L2's presence bit and sends it a real Probe via `b_dest`; the
  crossbar's master-ID routing for B-channel already exists generically.
- `l2_tlc_bridge.v` needs to stop tying off `probe_ack_from_l1_*`/
  `probe_req_to_l1_*` (currently unused because the Option A probe-safety
  argument relied on L2 never issuing Acquire) and actually respond:
  translate an incoming probe into a forced tag lookup +
  invalidate/writeback of that specific line inside L2, then answer with
  `ProbeAck`/`ProbeAckData`. L2 has no existing entry point for an
  externally-triggered snoop of an arbitrary line (`rv64g_l2_fsm.v`'s own
  coherence engine only probes L1 cores *above* it) — this is the one
  genuinely new piece of RTL under this option.
- DMA's own engine (`dma_coherent_agent.v`), `m2`, and the crossbar's
  directory/probe FSM are untouched. No ID-space changes to
  `rv64g_l2_directory.v`. No new arbiter. No topology inversion.

### Recommendation

**(b)** is the smaller, better-targeted fix: it reuses working,
already-tested coherence machinery (`tlc_slave_mem_manager`'s directory)
instead of building a parallel one inside L2, and confines new work to
`l2_tlc_bridge.v` plus one new snoop entry point into L2. (a) fully
retires the crossbar directory for this purpose and makes L2's own
directory the sole authority, which is architecturally cleaner in
isolation but costs an ID-space rework, a DMA-engine rewrite, and
touches both existing testbenches.

Not yet done: implementing (b), and re-running (a corrected version of)
`tb_dma_l2_coherency_gap.v` to confirm the hazard is closed — i.e. step
4's re-Acquire now triggers a real Probe round-trip and L2 observes the
DMA's write, or, if L2 no longer holds the line after being probed, is
forced to re-fetch showing the new data.

### Implementing (b): concrete change list

Grounded in reading `l2_tlc_bridge.v` and the relevant parts of
`l1_tilelink_adapter.v`, `rv64g_l2_fsm.v`, and `rv64g_l2_directory.v`
directly (not just the ports) before writing this down:

1. **`tlc/l2_tlc_bridge.v`** — the bulk of the work.
   - Swap the transaction type driven per word: from
     `L1_REQ_UNCACHED_READ`/`L1_REQ_UNCACHED_WRITE` (-> `Get`/
     `PutFullData`, `l2_tlc_bridge.v:159`) to `L1_REQ_READ_MISS`/
     `L1_REQ_WRITE_MISS` (-> `AcquireBlock`) for fills and
     `L1_REQ_WRITE_BACK` (-> `Release`/`ReleaseData`) for writeback.
     Confirmed `l1_tilelink_adapter.v`'s Acquire/Grant/Release path
     (`STATE_ACQUIRE_WAIT`/`STATE_RELEASE_SEND`) is *already* single-beat
     sized to match (`data_to_l1_data <= {{192{1'b0}}, d_data}` at line
     304; `c_data <= pending_data[63:0]` at line 332) — same granularity
     as the uncached path, so the bridge's existing 8-sequential-beats
     loop carries over unchanged, just pointed at a different
     `l1_request_type`. No burst-capable adapter change needed for this
     part.
   - New: a real E-channel (`GrantAck`) round trip per beat — today's
     Get/Put path never touches channel E.
   - New: implement the probe side instead of tying it off. Lines 69-85
     currently hard-wire `probe_ack_from_l1_valid = 0` etc., because the
     whole Option A probe-safety argument was "L2 never issues Acquire,
     so it's never tracked as an owner, so it's never probed" — this
     change removes that premise. The bridge needs to consume
     `probe_req_to_l1_valid/addr/permissions` and produce a real
     `probe_ack_from_l1_*` response.
   - New: track per-outstanding-word ownership (granted T or B?) so it
     knows whether a probe response needs `ProbeAckData` (dirty) or plain
     `ProbeAck`, and whether it owes a `Release` afterward.

2. **A new snoop entry point into `cache_system/` — no existing
   precedent to fully reuse, but partial precedent exists.**
   Nothing today lets something *below* L2 ask "invalidate address X and
   hand me the dirty data if any" — L2's coherence engine is built to
   probe L1 cores *above* it, never to be probed itself. But the pieces
   it needs mostly already exist:
   - `rv64g_l2_directory.v`'s read/write ports already expose exactly
     what's needed per way (`rd_valid`, `rd_sharers`, `rd_owner_valid`/
     `rd_owner_id`, `rd_dirty`, and a write port to update/clear them) —
     confirmed no code change needed here.
   - `rv64g_l2_fsm.v` already has a full "invalidate a line and collect
     dirty data" sequence built for capacity eviction: `ST_EVICT_WAIT`
     (`rv64g_l2_fsm.v:131,572`) computes which L1 cores to probe from
     `dir_rd_sharers_i`/`dir_rd_owner_valid_i` (lines 292-296), sends
     real `B_PROBE`/`B_PROBE_PERM`, waits for `ProbeAck`/`ProbeAckData`,
     writes the result back via `dir_wr_*` (lines 711-766).
   - The actual gap: `dir_rd_set_o`/`dir_wr_*_o` (and the separate tag
     array port, see below) have exactly one driver today — the FSM's
     own PLRU-triggered miss-handling logic. There's no path for an
     externally-named address (from a probe arriving via
     `l2_tlc_bridge`) to trigger this same sequence. New work: a
     snoop-trigger entry point in `rv64g_l2_fsm.v` that does a tag
     lookup by address (not by PLRU victim) and, on a hit, drives the
     *same* `ST_EVICT_WAIT` sequence — reusing it rather than duplicating
     it, with a small flag distinguishing "resume by filling the new
     line" (normal eviction) from "just report done" (snoop-triggered).

3. **`tlc_slave_mem_manager.v` — expected untouched.** The whole point of
   choosing (b) is that its `S_DIR_LOOKUP`/`S_PROBE_SEND`/`S_PROBE_WAIT`/
   `b_dest` routing already generalizes over `N_MASTERS` and doesn't
   special-case which master gets probed.

4. **`tlc_xbar_fabric_top.v` — needs verification, possibly a latent
   bug.** The B-channel probe routing from `tlc_slave_mem_manager` to
   `u_l1_m0`, and the `probe_req_to_l1_*`/`probe_ack_from_l1_*` wiring
   for master slot 0 specifically, is plumbed generically but has never
   been exercised — m0 has never been probed before. Given the earlier
   write-completion bug was exactly this pattern (an unexercised path
   silently broken), this should be verified rather than assumed to
   work.

5. **Testbenches.** `tb_tlc_l2_dma_shared_mem.v` and the coherency-gap
   test both need to drive an overlapping/conflicting access (L2 holding
   a line while DMA touches the same address) to exercise the new probe
   path at all — under the current design this scenario is deliberately
   non-conflicting from the crossbar's point of view. New expected
   outcome for the gap test: step 4's re-Acquire should now either see a
   real Probe fire and get the DMA's fresh data via the probe response,
   or miss and refetch — either way, no longer silently stale.

### Does this change `rv64g_l2_directory.v` itself?

No — confirmed by reading `rv64g_l2_fsm.v`'s directory usage directly.
`rv64g_l2_directory.v`'s existing ports already expose everything the
snoop path needs (see above), and `rv64g_l2_fsm.v`'s `ST_EVICT_WAIT`
already implements the exact "invalidate + collect dirty data + probe
sharers" operation required, just triggered by PLRU victim selection
instead of an external address. The real work is a new *trigger* into
`rv64g_l2_fsm.v` that reuses this existing sequence, not new logic in
the directory module itself.

One relevant fact this surfaced: `rv64g_l2_directory.v` does not store
the tag needed to answer "does L2 have address X cached" — it only holds
sharer/owner/dirty bits per way. The tag lives in a separate RAM
(`rv64g_l2_arrays.v`'s `tag_way_flat_i`), and only `rv64g_l2_fsm.v`
combines the two today (`current_tag == req_tag` against
`dir_rd_valid_i`, `rv64g_l2_fsm.v:188-189`). Anything wanting to check
L2's cache state from outside needs read access to *both* arrays, not
just the directory.

### Considered and deferred: a crossbar-side snoop filter

Question raised: could the crossbar keep a log/shadow of what L2 has
cached and use that to decide whether to probe, instead of building an
active invalidate path? This is the real "snoop filter" pattern used in
real coherence hubs, but it's additive, not a substitute:

- **Staleness/race**: any snapshot the crossbar reads of L2's state can
  be invalidated the very next cycle by `rv64g_l2_fsm.v`'s own
  independent activity (fill/evict/writeback) — a decision based on a
  stale peek can be wrong the instant it's acted on. Avoiding this needs
  either an atomic check-and-invalidate performed by L2 itself (i.e. the
  active Probe/ProbeAck path from option (b)), or a live channel keeping
  the crossbar's shadow copy in sync with every change L2 makes — itself
  new plumbing, not less.
- **A check alone doesn't fix anything**: even a correct "yes, L2 has
  this line" answer still requires some mechanism to make L2 give it up.
  That mechanism is the Probe/ProbeAck path. The filter, at best, lets
  the crossbar skip sending a probe when it already knows L2 doesn't
  have the line — it cuts probe traffic, it doesn't remove the need to
  build the invalidate path.
- A *conservative* filter (crossbar remembers "L2 possibly touched this
  line" on any fill, cleared only on an explicit eviction notification
  from L2; probes whenever it can't prove absence) is the tractable
  version — cheap, safe (false positives just cost an extra probe), but
  still additive on top of, not instead of, the active invalidate path.
  Deferred as a future optimization, not part of the current fix.

### Risk: does the `rv64g_l2_fsm.v` change disturb existing non-DMA
cache behavior?

Real risk, worth being explicit about before touching the FSM:

- The directory/tag read-write ports are single-requestor today, driven
  only by the FSM's own miss-handling logic. Adding a snoop trigger means
  the FSM needs new control flow to arbitrate "servicing a normal core
  request" vs "handling a snoop right now" — surface area that didn't
  exist before.
- The snoop path reuses `ST_EVICT_WAIT` rather than adding a parallel
  state (see above) — good for avoiding duplicated logic, but it means
  that state now needs to serve two callers with two different
  post-probe outcomes (resume-and-fill vs report-done), threaded through
  the *existing* state's exit logic without breaking the capacity-
  eviction case it already handles correctly.
- **No existing regression coverage for "pure cache, no DMA."**
  `cache_system` had no testbench at all before this branch. The
  testbenches built so far (`tb_tlc_l2_dma_shared_mem.v`,
  `tb_dma_l2_coherency_gap.v`) exercise L2 through real Acquire/Grant
  traffic but only single-core, no concurrent-sharer conflicts, no
  multi-way capacity eviction, no dirty-writeback-under-contention. There
  is currently nothing that would catch "the FSM change broke a normal
  core-only eviction" except close code reading.

Mitigation: the change is boundable to a specific seam (a new idle-time
entry condition, plus a small flag gating only `ST_EVICT_WAIT`'s
post-probe exit branch) — `ST_EVICT_WAIT`'s internal probe-collection
logic itself doesn't need to change. But before making this change, a
baseline regression test exercising normal core-driven L2 behavior with
no DMA involved (a capacity eviction with a real dirty sharer getting
probed, confirming today's behavior) should be built first, so the FSM
change lands against a known-good baseline instead of zero non-DMA test
coverage. **Not yet done.**
