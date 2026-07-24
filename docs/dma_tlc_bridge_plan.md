# DMA to TLC Bridge Plan

## Date

July 22, 2026

## Goal

Wrap `xspi_dma_memory_wk2609/dma/dma_flat_full_duplex_top.v` with a bridge so the DMA can issue transactions into the TLC fabric rooted at `tlc/tlc_xbar_top.v`, with the wider system shape:

- DMA master
- L2 / coherent fabric in the middle
- memory on the slave side

## What the Current DMA Actually Looks Like

`dma_flat_full_duplex_top.v` does not speak TileLink today.

It exposes two simple single-beat request/ack buses:

- Memory side:
  - `mem_req`
  - `mem_wr_en`
  - `mem_addr`
  - `mem_wdata`
  - `mem_rdata`
  - `mem_ack`

- Peripheral side:
  - `peri_req`
  - `peri_wr_en`
  - `peri_addr`
  - `peri_wdata`
  - `peri_rdata`
  - `peri_ack`

Important properties:

- It is single-beat request/ack, not multi-channel.
- It is fundamentally load/store style.
- It does not track cache state.
- It does not currently have logic for probes, releases, grants, or grant acknowledgements.

## What the TLC Crossbar Expects

`tlc/tlc_xbar_top.v` is not a raw DMA-facing bus wrapper.

Its visible top-level interface is an L1 cache-controller style interface:

- request valid/address/type/data/permissions
- data return
- probe request outputs
- probe ack inputs

Internally, it drives full TileLink-C with all 5 channels:

- `A`: requests from master to slave
- `B`: probes from slave/manager to master/client
- `C`: voluntary releases and probe acknowledgements from master/client
- `D`: responses/grants from slave/manager to master/client
- `E`: grant acknowledgements from master/client to slave/manager

## Main Design Decision

### Short answer

No, a DMA-to-TLC bridge should usually **not** implement the full behavior of all TLC channels unless you want the DMA to behave like a coherent cache client.

### Why

Your DMA is not an L1 cache:

- it does not hold coherent cache lines
- it does not need to service probes in the normal uncached case
- it does not need to evict dirty cache lines
- it does not naturally generate `C` traffic for releases

So the cleanest bridge is:

- generate only the transaction types needed for DMA memory reads/writes
- consume only the return traffic required to complete those requests
- tie off or structurally disable channels that are only needed for coherent cached clients

## Recommended Integration Boundary

There are two ways to connect the DMA into this TLC world.

### Option 1: Bridge DMA into the L1-style interface of `tlc_xbar_top`

This means the bridge would drive:

- `mX_req_valid`
- `mX_req_addr`
- `mX_req_type`
- `mX_req_data`
- `mX_req_permissions`

and handle:

- `mX_req_ready`
- `mX_data_valid`
- `mX_data`
- `mX_data_error`
- `mX_probe_req_*`
- `mX_probe_ack_*`

Pros:

- least invasive to the existing TLC top
- reuses the existing `l1_tilelink_adapter`

Cons:

- semantically awkward because the DMA is not really an L1 cache
- forces the bridge to fake cache-style request types and permissions
- forces some probe handling path even if the DMA should really be uncached

### Option 2: Build a DMA-to-raw-TLC client bridge and connect below `tlc_xbar_top`

This means creating a new DMA TLC client that plugs directly into the raw TileLink-C master ports used by `tl_xbar_nm`.

Pros:

- architecturally cleaner
- you only implement the channels and opcodes the DMA truly needs
- avoids pretending the DMA is a cache controller

Cons:

- requires a new top-level integration wrapper, or modification beside `tlc_xbar_top`
- slightly more work upfront

## Recommendation

Use **Option 2**.

Build a dedicated `dma_tlc_bridge` as a raw TileLink client, then create a new system wrapper such as:

- `dma_l2_mem_tlc_system_top.v`

This wrapper should instantiate:

- `dma_flat_full_duplex_top`
- `dma_tlc_bridge`
- `tl_xbar_nm` or a lightly adapted TLC fabric top
- L2 / memory-side slave model or real memory block

This is cleaner than forcing the DMA through `l1_tilelink_adapter`.

## Channel-by-Channel Recommendation

### A channel

Use it.

This is the main request path for DMA memory operations.

Map DMA requests like this:

- DMA read: `A = Get`
- DMA write: `A = PutFullData` or `PutPartialData`

You will need:

- `a_valid`
- `a_ready`
- `a_opcode`
- `a_param`
- `a_size`
- `a_source`
- `a_address`
- `a_mask`
- `a_data`

### D channel

Use it.

This is how the DMA receives completions and read data.

Map D responses like this:

- read response data returns through `D`
- write completion returns through `D`
- error/denied maps to DMA error or failed ack behavior

You will need:

- `d_valid`
- `d_ready`
- `d_opcode`
- `d_source`
- `d_sink`
- `d_data`
- `d_denied` or `d_error`

### E channel

Maybe.

If the manager returns `Grant` or `GrantData`, then the client must send `GrantAck` on `E`.

If your DMA bridge uses only uncached TL-C operations and the downstream system returns simple `AccessAck` / `AccessAckData`, then `E` may not be needed in the active data path.

Recommendation:

- design the bridge so `E` is present in the interface
- keep it idle unless the chosen response opcode requires acknowledgement

This gives safety without overcomplicating the first version.

### B channel

Usually no for DMA v1.

`B` is for probes from the coherent manager to the client.

If the DMA does not cache lines and is modeled as an uncached client:

- it should not normally need probe traffic
- you can keep `b_ready` high and treat any incoming probe as unsupported or illegal

Better still:

- place the DMA in a non-probed / uncached client role if your fabric supports that distinction

### C channel

Usually no for DMA v1.

`C` is used for:

- releases
- probe acknowledgements

A non-caching DMA should not need to emit releases, and it should not need probe acknowledgements if probes are architecturally disallowed for that client.

## Practical Conclusion on Channels

For the first DMA-TLC bridge version:

- definitely implement `A`
- definitely implement `D`
- include `E` interface support, but possibly leave inactive unless your response path needs GrantAck
- do not actively use `B`
- do not actively use `C`

So the practical answer is:

**No, not all five channels should be active in the first DMA bridge.**

The bridge should primarily be an `A/D` bridge, with optional minimal `E` handling.

## Suggested DMA Bridge Behavior

## Request mapping

For every DMA-side single-beat request:

- if `mem_req=1` and `mem_wr_en=0`, emit one TL `Get`
- if `mem_req=1` and `mem_wr_en=1`, emit one TL `PutFullData`

Then wait for the matching D response:

- on successful response, raise DMA `mem_ack`
- on read, copy `d_data` to `mem_rdata`

## Important simplifications for v1

- one outstanding DMA request at a time
- one source ID
- no burst combining initially
- 64-bit beat size only
- 8-byte aligned accesses only

This matches your current DMA interface much better and keeps bring-up simple.

## Width / semantic mismatches to handle

### Address width

Current DMA top uses `ADDR_WIDTH=64`.

Current `tlc_xbar_top.v` and `l1_tilelink_adapter.v` visible controller side use 32-bit addresses.

You need one explicit policy:

- either truncate DMA addresses to 32 bits
- or widen the TLC system to 64-bit addresses

Recommendation:

- make this explicit in the bridge parameters
- do not silently truncate without a range check

### Data width

Current DMA beat width is 64 bits.

This matches the TL data beat width used in the TLC code, which is good.

### Cache-line semantics

The L1 adapter talks in 256-bit cache-line semantics for some coherent flows.

Your DMA does not.

That is another reason to avoid using `l1_tilelink_adapter` as the DMA bridge boundary.

## Proposed Modules

### 1. `dma_tlc_bridge.v`

Purpose:

- convert DMA single-beat request/ack into raw TileLink client traffic

Inputs from DMA:

- `dma_req`
- `dma_wr_en`
- `dma_addr`
- `dma_wdata`

Outputs to DMA:

- `dma_rdata`
- `dma_ack`
- optional `dma_error`

TLC side:

- active `A`
- active `D`
- optional `E`
- passive or tied-off `B/C`

### 2. `dma_tlc_client_top.v`

Purpose:

- wrap `dma_flat_full_duplex_top` and connect one side, probably `mem_*`, into `dma_tlc_bridge`

This lets you preserve the current DMA top unchanged.

### 3. `dma_l2_mem_tlc_system_top.v`

Purpose:

- instantiate DMA wrapper
- connect to TLC crossbar / L2 / memory slave
- reserve another master slot for future CPU/L1 if needed

## Recommended Bring-Up Order

1. Build `dma_tlc_bridge.v` as a single-outstanding uncached client.
2. Connect only DMA memory-side `mem_*` signals to the bridge first.
3. Tie the peripheral side of DMA to your existing peripheral path or TB model.
4. Hook the bridge into the TLC fabric below the L1-controller abstraction if possible.
5. Start with only A/D traffic.
6. Add minimal E handling only if the chosen slave/L2 returns Grant-style responses.
7. Add pipelining, multiple outstanding requests, and bursts later.

## What I Recommend We Do Next

1. Decide the bridge boundary:
   - preferred: raw TLC client below `tlc_xbar_top`
   - fallback: fake an L1-style requester into `tlc_xbar_top`
2. Decide whether the DMA is:
   - uncached non-coherent client
   - fully coherent client
3. Freeze v1 assumptions:
   - one outstanding request
   - 64-bit beat
   - 8-byte alignment
   - 32-bit or 64-bit address policy
4. Then implement:
   - `dma_tlc_bridge.v`
   - system wrapper around `dma_flat_full_duplex_top`

## Current Recommendation in One Sentence

Treat the DMA as an **uncached TileLink client**, not as an L1 cache, and build a bridge that mainly uses **A and D**, with **optional E**, while avoiding active B/C handling in v1.



////////////////////////////////////////////////////////////////////////////////////////////////////
A “coherent TL-C manager wrapper” means:

  - on the outside, it looks like a TL-C slave to the crossbar
  - on the inside, it drives your existing memory using a simpler interface, such as TL-UH or raw mem_*

  So it is a protocol/behavior wrapper around memory, not a modification of memory itself.

  What that wrapper must do is more than signal translation. It must also implement coherence behavior:

  - keep directory state for each cache line
  - know who currently holds a line
  - send B-channel probes when another master has conflicting permissions
  - wait for C-channel probe acks or releases
  - send the correct D response
  - consume E for grant acknowledgement

  That is why I used the word “manager,” not just “bridge.”

  About your idea of a direct TLC bridge from mem_* to TL-C:

  - If by “bridge” you mean only “convert read/write/address/data into TL-C wires,” that is not enough for a coherent memory slave.
  - A raw memory interface has no notion of sharers, owners, probes, downgrade, release, or grant tracking.
  - TL-C slave-side logic needs those behaviors, not just the wires.

  So there are two valid options:

  1. mem_* to TL-UH bridge

  - good for non-coherent memory
  - only really needs A and D

  2. mem_* to TL-C manager

  - needed if this memory sits behind a coherent crossbar
  - must implement directory + A/B/C/D/E behavior

  The important correction is this: you would not connect your new memory bridge to tlc_slave_model. tlc/tlc_slave_model.v:34 is already the TL-C slave manager.
  It already contains both:

  - directory logic
  - behavioral memory storage

  So if you build a real memory-backed TL-C slave, it would replace tlc_slave_model, not connect underneath it.

  The clean architecture is:

  tlc_xbar_top
    -> your new TL-C memory manager module
        -> tl_sram_ctrl_top   or raw mem_*

  Not:

  tlc_xbar_top
    -> tlc_slave_model
        -> another TL-C bridge
        -> memory

  Because tlc_slave_model is already occupying the coherent slave role.

  So the best practical path is:

  - keep tl_sram_ctrl_top unchanged
  - create a new module that has TL-C slave ports on top
  - inside that module, use directory/coherence FSM logic
  - after coherence is resolved, issue TL-UH transactions to tl_sram_ctrl_top