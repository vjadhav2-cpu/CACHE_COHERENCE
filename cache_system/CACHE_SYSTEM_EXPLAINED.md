# Cache System Explained

This document explains the cache system in this repository in simple technical words. It is based on the current Verilog files, especially `rv64g_cache_system.v`, `rv64g_l1_dcache.v`, `rv64g_l2_cache.v`, `rv64g_l2_fsm.v`, and the array/helper modules.

## 1. Big Picture

The design is a multi-core cache system for an RV64G-style processor.

At the top level:

```text
CPU cores
   |
   | one private L1 data cache per core
   v
L1 data caches
   |
   | TileLink-like channels through a socket/crossbar
   v
Shared L2 cache and coherence manager
   |
   | memory TileLink-like A/D interface
   v
External memory
```

The main job of the system is:

1. Give each core a fast private L1 data cache.
2. Keep all L1 caches coherent, so two cores do not use incompatible copies of the same cache line.
3. Use the shared L2 cache as the central manager that tracks ownership, sharers, dirty data, probes, refills, and writebacks.
4. Move data in 64-byte cache lines over a 64-bit data bus, so a full cache line is transferred as 8 beats.

## 2. Main Configuration

The shared constants are in `params.vh`.

Important values:

| Item | Value |
| --- | --- |
| Number of cores | 4 |
| Cache line size | 64 bytes |
| Words per cache line | 8 words of 64 bits |
| L1 size | 16 KiB per core |
| L1 associativity | 8-way |
| L1 sets | 32 |
| L2 size | 256 KiB shared |
| L2 associativity | 16-way |
| L2 sets | 256 |

Address breakdown:

For L1:

```text
addr[2:0]   = byte inside a 64-bit word
addr[5:3]   = word inside a 64-byte line
addr[10:6]  = L1 set index, 32 sets
addr[63:11] = L1 tag
```

For L2:

```text
addr[5:0]   = byte inside a 64-byte line
addr[13:6]  = L2 set index, 256 sets
addr[63:14] = L2 tag
```

## 3. Coherence States

The code names the states with an NBTT style, but they map closely to MESI ideas.

| Code name | Bits | Meaning | Similar MESI state |
| --- | --- | --- | --- |
| `MESI_N` | `00` | Not valid in this cache | Invalid |
| `MESI_B` | `01` | Shared clean copy | Shared |
| `MESI_T` | `10` | Exclusive clean copy | Exclusive |
| `MESI_TT` | `11` | Exclusive dirty copy | Modified |

Simple meaning:

- `N`: this cache cannot use the line.
- `B`: this cache can read the line, but other caches may also have it.
- `T`: this cache has exclusive permission, but the data is still clean.
- `TT`: this cache has exclusive permission and has modified the line.

Writes require exclusive permission. If an L1 has only `B`, it must ask the L2 for permission before writing.

## 4. TileLink-Like Channels

The design uses TileLink-style channels between L1s and L2.

| Channel | Direction | Purpose |
| --- | --- | --- |
| A | L1 to L2 | Requests such as `AcquireBlock` and `AcquirePerm` |
| B | L2 to L1 | Probes, used to downgrade or invalidate L1 copies |
| C | L1 to L2 | Releases and probe acknowledgements, optionally with data |
| D | L2 to L1 | Grants, grant data, and release acknowledgements |
| E | L1 to L2 | Grant acknowledgement |

The most important messages are:

- `AcquireBlock`: L1 asks for a cache line.
- `AcquirePerm`: L1 already has a shared line and asks for write permission.
- `Probe`: L2 asks an L1 to downgrade or invalidate a line.
- `ProbeAck`: L1 confirms the probe.
- `ProbeAckData`: L1 confirms the probe and sends dirty data back.
- `Release`: L1 voluntarily gives up a clean line.
- `ReleaseData`: L1 voluntarily gives up a dirty line with data.
- `GrantData`: L2 sends a cache line to L1.
- `Grant`: L2 grants permission without sending data.
- `GrantAck`: L1 confirms it received the grant.

## 5. Top-Level Module

File: `rv64g_cache_system.v`

This module connects everything together.

It creates:

- one `rv64g_l1_dcache` per core;
- one `tl_socket_m1` to connect all L1s to the shared L2;
- one `rv64g_l2_cache`;
- the external memory interface from the L2.

The CPU side has one scalar request interface per core:

- `cpu_req_i`: request valid;
- `cpu_we_i`: write enable;
- `cpu_be_i`: byte enables;
- `cpu_addr_i`: address;
- `cpu_wdata_i`: write data;
- `cpu_gnt_o`: request accepted;
- `cpu_rvalid_o`: response valid;
- `cpu_rdata_o`: read data or operation result.

It also wires atomic signals:

- AMO;
- load-reserved;
- store-conditional.

The vector LSU ports on the L1 caches are currently tied off at the top level, but the L1 implementation contains vector support internally.

## 6. L1 Data Cache

File: `rv64g_l1_dcache.v`

Each core has a private L1 data cache.

Its main jobs are:

1. Detect read and write hits.
2. Return data quickly on read hits.
3. Update data on write hits if it has write permission.
4. Request data or permissions from L2 on misses and upgrades.
5. Evict old lines when needed.
6. Write back dirty victims.
7. Respond to L2 probes.
8. Support AMO, LR, and SC operations.
9. Support internal vector LSU banked access.

### L1 hit path

The L1 reads all ways for the selected set and word. It compares all tags in the set. A hit means:

- the line state is not `MESI_N`;
- the stored tag matches the request tag.

On a read hit:

1. L1 accepts the CPU request.
2. It returns the selected 64-bit word.
3. It updates PLRU for replacement tracking.

On a write hit:

- If state is `MESI_T` or `MESI_TT`, L1 writes the word directly and changes the state to `MESI_TT`.
- If state is `MESI_B`, L1 cannot write yet. It sends `AcquirePerm` to L2 to get exclusive permission.

### L1 read miss

On a normal read miss:

1. L1 chooses a victim way using PLRU.
2. If the victim is dirty, L1 sends `ReleaseData` to L2.
3. If the victim is clean but valid, L1 sends `Release`.
4. L1 sends `AcquireBlock` for the requested line.
5. L2 responds with `GrantData`.
6. L1 receives 8 beats and fills the selected way.
7. L1 sends `GrantAck`.
8. L1 returns the requested word to the CPU.

The state after refill is usually:

- `MESI_B` for a shared read;
- `MESI_T` for a read-for-ownership or exclusive request.

### L1 write miss

On a write miss:

1. L1 chooses a victim.
2. It writes back or releases the victim if needed.
3. It sends `AcquireBlock` with write intent.
4. L2 grants a line with exclusive permission.
5. L1 fills the line.
6. L1 applies the CPU write into the correct word using byte enables.
7. L1 marks the line `MESI_TT`.
8. L1 completes the CPU request.

This is a read-for-ownership flow: the L1 gets the whole line first, then modifies it locally.

### L1 probe handling

The L2 can send a probe to an L1 on channel B.

The L1 handles probes even while it is in several miss states. This is important because if an L1 ignored probes while waiting for L2, the system could deadlock.

When a probe arrives:

1. L1 checks whether it has the probed line.
2. If it does not have the line, it sends `ProbeAck`.
3. If it has a clean line, it invalidates the line and sends `ProbeAck`.
4. If it has a dirty line, it invalidates the line and sends `ProbeAckData` with all 8 data beats.

The current L1 probe response generally invalidates the line to `MESI_N`.

### L1 refill and writeback FSM

The L1 FSM states include:

| State | Purpose |
| --- | --- |
| `S_IDLE` | Normal hit processing and accepting CPU requests |
| `S_REF_REQ` | Send `AcquireBlock` |
| `S_PERM_REQ` | Send `AcquirePerm` |
| `S_REF_WAIT` | Wait for grant data or grant |
| `S_ACK` | Send `GrantAck` |
| `S_RESP` | Complete the CPU response after refill or permission grant |
| `S_WB_REQ` | Start a release or probe response |
| `S_WB_DATA` | Send remaining data beats |
| `S_WB_WAIT` | Wait for `ReleaseAck` |
| `S_PROBE_RESP` | Send a probe response |
| `S_AMO_MODIFY` | Compute atomic result |
| `S_AMO_WRITE` | Write atomic result to cache |
| `S_VLSU_MISS` | Start vector miss refill |
| `S_VLSU_REPLAY` | Wait until vector misses are resolved and can replay |

## 7. L1 Banked Arrays

Files:

- `rv64g_l1_banked_arrays.v`
- `rv64g_l1_sram_bank.v`
- `rv64g_l1_bank_arbiter.v`
- `rv64g_l1_crossbar.v`
- `rv64g_l1_vlsu_hit_detect.v`
- `rv64g_l1_vlsu_miss_handler.v`

The L1 data array is split into 8 banks.

Each bank stores:

- data for all ways;
- tags for all ways;
- coherence state for all ways.

The bank is selected by:

```text
addr[5:3]
```

That is the word number inside the 64-byte line. Since a line has 8 words, there are 8 banks.

### Why bank the L1?

The banked design supports vector memory operations. A vector operation may have up to 8 lanes. If the lanes access different banks, they can be served together. If multiple lanes access the same bank, the crossbar serializes them over multiple cycles.

### Scalar access

The scalar CPU side gets priority over vector access in each bank arbiter.

For scalar tag/state updates, the design can broadcast the tag/state write to all banks. This matters because the state and tag for a cache line must stay consistent across the line's banks.

### Vector access

The vector crossbar:

1. Latches all lane requests.
2. Computes which bank each lane needs.
3. Grants one lane per bank per cycle.
4. Tracks unfinished lanes.
5. Continues until every active lane is served.

The vector miss handler:

1. Detects which lanes missed.
2. Captures unique missed cache lines.
3. Requests refills one line at a time.
4. Signals when all refills are done.
5. Allows the vector operation to replay.

This is simple and blocking, but easy to reason about.

## 8. L1 Replacement Policy

File: `rv64g_l1_plru.v`

The L1 uses an 8-way pseudo-LRU tree.

PLRU means "pseudo least recently used." It is cheaper than exact LRU. It keeps 7 bits per set and walks a small tree to choose a victim.

The victim choice prefers invalid ways first. If all ways are valid, it uses the PLRU tree.

## 9. Atomic, LR, and SC Support

Files:

- `rv64g_l1_dcache.v`
- `rv64g_atomic_alu.v`

The atomic ALU supports RV64A-style AMO operations:

- swap;
- add;
- xor;
- and;
- or;
- signed min/max;
- unsigned min/max;
- word and doubleword forms.

For AMO:

1. L1 reads the old value.
2. If it does not have write permission, it gets permission from L2.
3. The atomic ALU computes the new value.
4. L1 writes the new value into the cache line.
5. L1 returns the old value to the CPU.

For LR/SC:

- LR loads the value and records a reservation for the cache line.
- SC succeeds only if the reservation is still valid.
- Probes or local stores to the same reserved line clear the reservation.
- On SC success, L1 writes the new data and returns `0`.
- On SC failure, L1 returns `1`.

## 10. L2 Cache

File: `rv64g_l2_cache.v`

The L2 is the shared cache and coherence manager.

It contains:

1. `rv64g_l2_fsm`: main control logic.
2. `rv64g_l2_directory`: coherence directory.
3. `rv64g_l2_arrays`: L2 tag and data arrays.
4. `rv64g_l2_mshr`: tracks one active transaction and pending probes.
5. `rv64g_l2_plru`: replacement policy.

The L2 accepts L1 requests on channels A, C, and E. It sends probes on B and grants on D. It also talks to external memory.

## 11. L2 Directory

File: `rv64g_l2_directory.v`

The directory is the most important coherence structure in the L2.

For each L2 set and way, it stores:

- `valid`: whether the line exists in this L2 slot;
- `sharers`: a bit mask of cores that have shared clean copies;
- `owner_valid`: whether one core owns the line exclusively;
- `owner_id`: which core owns it;
- `dirty`: whether the owner has dirty data.

The directory enforces two invariants:

1. If `dirty` is set, `owner_valid` is forced true.
2. If `owner_valid` is true, the sharer mask is forced to zero.

This means the directory represents either:

- multiple shared clean readers; or
- one exclusive owner; but not both at the same time.

## 12. L2 FSM

File: `rv64g_l2_fsm.v`

The L2 FSM is the central decision maker.

Its main jobs are:

1. Accept an L1 acquire request.
2. Read L2 tags and directory state.
3. Decide hit or miss.
4. Send probes if another core has the line.
5. Wait for probe acknowledgements.
6. Fetch data from memory on L2 miss.
7. Write back dirty L2 victims to memory.
8. Grant data or permission to the requesting L1.
9. Update the directory.
10. Wait for `GrantAck`.
11. Complete the transaction and free the MSHR.

The current L2 is mostly blocking. It has an MSHR, but the FSM only accepts a new A-channel acquire when the MSHR is free. So the system handles one main L2 coherence transaction at a time.

Important states:

| State | Purpose |
| --- | --- |
| `ST_IDLE` | Wait for A-channel acquire or C-channel release |
| `ST_RAM_WAIT` | Wait for tag/directory read timing |
| `ST_CHECK` | Check hit/miss and send probes if needed |
| `ST_WAIT_ACK` | Wait for probe acknowledgements |
| `ST_EVICT_WAIT` | Wait for probes to invalidate an L2 victim |
| `ST_MEM_READ` | Read a full line from memory |
| `ST_MEM_WRITE` | Write a dirty victim line to memory |
| `ST_MEM_RESP` | Wait for memory write response |
| `ST_REL_DATA` | Receive data beats from an L1 release |
| `ST_GRANT` | Send grant or release acknowledgement |
| `ST_UPDATE` | Update directory and tags |
| `ST_WAIT_E` | Wait for L1 `GrantAck` |
| `ST_COMPLETE` | Deallocate MSHR and return to idle |

## 13. L2 Hit Flow

Example: core 0 reads a line that is already clean in L2 and has no exclusive owner.

1. L1 sends `AcquireBlock` with `NtoB`.
2. Socket forwards the request to L2 and adds the core ID to the source field.
3. L2 reads tag and directory.
4. L2 finds a hit.
5. No probes are needed.
6. L2 sends `GrantData` for 8 beats.
7. L1 fills the cache line as `MESI_B`.
8. L1 sends `GrantAck`.
9. L2 updates the directory by adding core 0 to the sharer mask.

## 14. L2 Miss Flow

Example: core 0 reads a line that is not in L2.

1. L1 sends `AcquireBlock`.
2. L2 checks tags and misses.
3. L2 chooses a victim way using PLRU.
4. If the victim is present in L1 caches, L2 probes those L1s first.
5. If the victim is dirty, L2 writes it back to memory.
6. L2 sends a memory `Get` for the requested line.
7. Memory returns 8 beats.
8. L2 writes those beats into its data array.
9. L2 sends `GrantData` to the requesting L1.
10. L2 writes the new tag and directory entry.
11. L2 waits for `GrantAck`.
12. The transaction completes.

## 15. Write Permission Flow

Example: core 0 has a shared line and wants to write it.

1. L1 write hits in state `MESI_B`.
2. L1 sends `AcquirePerm` with `BtoT`.
3. L2 looks up the line.
4. L2 sends probes to other sharers and any owner, excluding the requester.
5. Other L1s invalidate their copies and send `ProbeAck` or `ProbeAckData`.
6. L2 waits until all pending probe bits are clear.
7. L2 sends `Grant`.
8. L1 sends `GrantAck`.
9. L1 performs the store and marks the line `MESI_TT`.
10. L2 updates the directory so core 0 is the owner and the line is dirty.

## 16. Dirty Owner Read Flow

Example: core 1 reads a line that core 0 owns dirty.

1. Core 1 L1 sends `AcquireBlock` with `NtoB`.
2. L2 sees that core 0 is the dirty owner.
3. L2 sends a probe to core 0.
4. Core 0 sends `ProbeAckData` with the dirty line.
5. L2 writes that data into the L2 data array.
6. L2 sends `GrantData` to core 1.
7. L2 updates the directory so the line is clean and shared.
8. The requester becomes a sharer.

In the current L1 implementation, a probe generally invalidates the probed L1 copy. The L2 update logic for a read of an owned line adds the old owner and requester as sharers, which is the intended directory-level downgrade behavior.

## 17. Voluntary L1 Eviction

When L1 needs to replace a valid line:

- If the line is clean, it sends `Release`.
- If the line is dirty, it sends `ReleaseData` with 8 beats.

The L2 accepts the release, updates or stores data if needed, updates the directory, and sends `ReleaseAck`.

## 18. L2 MSHR

File: `rv64g_l2_mshr.v`

The MSHR tracks one active L2 transaction.

It stores:

- request address;
- request source;
- request type;
- pending probe acknowledgements.

When L2 sends probes, it loads a bit mask of cores that must reply. Each `ProbeAck` clears one bit. The L2 waits until the pending mask becomes zero before granting the requester.

Because there is only one valid MSHR entry, the current L2 is not a highly parallel non-blocking cache. It is a simpler blocking coherence manager.

## 19. L2 Arrays

File: `rv64g_l2_arrays.v`

The L2 arrays store:

- cache data;
- tags.

The directory stores coherence metadata separately.

Like the L1, a full cache line is 8 words. The L2 reads and writes one 64-bit word per beat. Refills, grants, and writebacks therefore take 8 beats for data.

## 20. L2 Replacement Policy

File: `rv64g_l2_plru.v`

The L2 uses a 16-way pseudo-LRU tree.

It uses 15 bits per set. It prefers invalid ways first. If all ways are valid, it chooses a victim by walking the PLRU tree.

## 21. Socket / Crossbar Between L1 and L2

File: `tl_socket_m1.v`

This module connects multiple L1 clients to one L2 manager.

For L1-to-L2 channels:

- Channel A requests are arbitrated from many L1s into one L2 input.
- Channel C releases/probe acknowledgements are arbitrated from many L1s into one L2 input.
- Channel E grant acknowledgements are arbitrated from many L1s into one L2 input.

For L2-to-L1 channels:

- Channel D grants are demultiplexed back to the correct L1.
- Channel B probes are demultiplexed to the target L1.

The socket extends the source ID by adding the core ID. This lets the L2 know which core sent a request, and lets responses route back to the correct L1.

The socket also tracks multi-beat bursts so all beats of a burst stay with the same selected client.

## 22. Simple End-to-End Examples

### Read hit in L1

```text
CPU -> L1: read address A
L1: tag matches and state is valid
L1 -> CPU: return data
```

No L2 access is needed.

### Write hit with exclusive permission

```text
CPU -> L1: write address A
L1: tag matches and state is T or TT
L1: write data with byte enables
L1: mark line TT
L1 -> CPU: complete
```

No L2 access is needed.

### Write hit on shared line

```text
CPU -> L1: write address A
L1: tag matches but state is B
L1 -> L2: AcquirePerm
L2 -> other L1s: probes
other L1s -> L2: ProbeAck
L2 -> L1: Grant
L1 -> L2: GrantAck
L1: perform write and mark TT
L1 -> CPU: complete
```

### Read miss

```text
CPU -> L1: read address A
L1: miss
L1: choose victim
L1 -> L2: release victim if needed
L1 -> L2: AcquireBlock
L2: get line from L2 array or memory
L2 -> L1: GrantData, 8 beats
L1: fill cache line
L1 -> L2: GrantAck
L1 -> CPU: return requested word
```

### Dirty line requested by another core

```text
Core 0 L1: has line A dirty
Core 1 L1 -> L2: AcquireBlock for line A
L2 -> Core 0 L1: Probe
Core 0 L1 -> L2: ProbeAckData, 8 beats
L2: update data array
L2 -> Core 1 L1: GrantData
L2: update directory
```

## 23. Important Design Notes

1. The system is centered around 64-byte cache lines.
2. A full line transfer is 8 beats because the bus is 64 bits wide.
3. L1s are private and L2 is shared.
4. L2 is the coherence authority.
5. The directory tells L2 who has a line and whether it is dirty.
6. The current L2 flow is mostly blocking because there is one active MSHR.
7. L1 handles probes in several non-idle states to avoid coherence deadlock.
8. Dirty data must come back through `ProbeAckData` or `ReleaseData`.
9. Writes require exclusive permission.
10. PLRU is used for victim selection in both L1 and L2.

## 24. File-by-File Summary

| File | Purpose |
| --- | --- |
| `params.vh` | Shared constants: cache sizes, states, TileLink opcodes and params |
| `rv64g_cache_system.v` | Top-level system wiring for cores, L1s, socket, L2, and memory |
| `rv64g_l1_dcache.v` | Main L1 data cache controller |
| `rv64g_l1_banked_arrays.v` | Banked L1 array wrapper for scalar and vector access |
| `rv64g_l1_sram_bank.v` | One L1 SRAM bank with data, tag, and state arrays |
| `rv64g_l1_bank_arbiter.v` | Per-bank scalar/vector arbitration |
| `rv64g_l1_crossbar.v` | Routes vector lanes to L1 banks and handles bank conflicts |
| `rv64g_l1_vlsu_hit_detect.v` | Detects vector lane hits/misses |
| `rv64g_l1_vlsu_miss_handler.v` | Collects unique vector misses and refills them one at a time |
| `rv64g_l1_plru.v` | L1 8-way replacement policy |
| `rv64g_atomic_alu.v` | AMO computation unit |
| `tl_socket_m1.v` | Multi-L1 to single-L2 TileLink-style socket |
| `tl_arbiter.v` | Generic valid/ready arbiter |
| `tl_demux.v` | Generic valid/ready demultiplexer |
| `rv64g_l2_cache.v` | L2 wrapper connecting FSM, directory, arrays, MSHR, and PLRU |
| `rv64g_l2_fsm.v` | Main L2 coherence and memory-access FSM |
| `rv64g_l2_directory.v` | L2 coherence directory |
| `rv64g_l2_arrays.v` | L2 data and tag arrays |
| `rv64g_l2_mshr.v` | Tracks active transaction and pending probe acknowledgements |
| `rv64g_l2_plru.v` | L2 16-way replacement policy |
| `rv64g_l1_arrays.v` | Older/non-banked L1 array implementation |

## 25. Mental Model

A simple way to understand the system:

- The L1 is the fast local copy.
- The L2 is the manager and backup copy.
- The directory is the L2's notebook.
- A read can share data with other readers.
- A write must first become the only writer.
- Probes are how the L2 asks other L1s to give up or downgrade their copies.
- `ProbeAckData` and `ReleaseData` are how dirty data gets returned.
- `GrantData` gives a line to an L1.
- `Grant` gives permission.
- `GrantAck` tells L2 the permission transfer is complete.

