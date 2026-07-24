# DMA Microarchitecture Specification

## 1. Scope and Compliance

### 1.1 Scope
This document explains the DMA core behavior implemented in `dma_flat_full_duplex_top` and its internal blocks.

In scope:
- Transfer modes: Memory-to-Peripheral (M2P) and Peripheral-to-Memory (P2M)
- How `start`, `stream_type`, base addresses, and `num_bytes` are handled
- Internal data movement: read streamer -> FIFO -> write streamer
- Completion and error behavior (`busy`, `done`, alignment checks)

Out of scope:
- Detailed internals of SPI bridge, crossbar, and memory controller
- Physical implementation topics (timing closure, floorplan, DFT)

### 1.2 Conventions Used Here
- "Must" = required behavior for correctness
- "Should" = recommended behavior
- "Can" = optional behavior

### 1.3 Referenced RTL
- `dma_flat_full_duplex_top.v`
- `dma_streamer` (defined in the same file)
- `dma_fifo` (defined in the same file)

## 2. Architecture Overview

### 2.1 DMA Hierarchy (Local Only)
- `dma_flat_full_duplex_top`
- `u_read : dma_streamer`
- `u_fifo : dma_fifo`
- `u_write : dma_streamer`

### 2.2 One-Line Mental Model
The DMA runs two coordinated movers in parallel: one fetches data from source, one writes data to destination, and FIFO absorbs timing mismatch.

### 2.3 Transfer Modes
- M2P (`stream_type=0`): read from memory, write to peripheral
- P2M (`stream_type=1`): read from peripheral, write to memory

### 2.4 Clock/Reset
- Single clock domain (`clk`)
- Top-level reset is synchronous active-high (`rst`)
- FIFO uses active-low reset internally (`rst_n = ~rst`)

## 3. Interface Contract

### 3.1 Control Interface
Inputs:
- `start`
- `stream_type`
- `mem_base_addr`
- `peri_base_addr`
- `num_bytes`

Behavior:
- If `start` comes when idle, descriptor is latched and transfer starts.
- If `start` comes while busy, request is ignored to protect in-flight transfer.

### 3.2 Memory/Peripheral Bus Contract
Both sides follow single-beat request/ack behavior:
- Request signals: `{req, wr_en, addr, wdata}`
- Response/complete: `ack` and `rdata` for reads

Practical rule:
- A beat is complete only when request and acknowledge handshake for that beat.

### 3.3 Status Signals
- `busy` = at least one internal streamer is active
- `done` = both streamers are complete
- `done` is masked during accepted-start handoff to avoid stale completion glitches

### 3.4 Valid Configuration
- `num_bytes` must be aligned to beat size (`DATA_WIDTH/8`)
- Misaligned length is rejected by config checks (no normal transfer progression)

## 4. Microarchitecture Definition

### 4.1 Datapath
- Read streamer captures read data from selected source bus
- Captured data is pushed into FIFO
- Write streamer pops FIFO and drives destination bus writes
- Top-level muxing maps read/write buses to memory/peripheral based on mode

### 4.2 Control Path
Key control logic handles:
- Descriptor acceptance and latching
- Read-side beat progression
- Write-side prefetch/buffered progression
- Final completion aggregation

### 4.3 Streamer FSM (Shared Structure)
States:
- `STATE_IDLE`
- `STATE_REQ`
- `STATE_DONE`

Read role in `STATE_REQ`:
- Issue request
- On handshake, capture read data and pulse FIFO write-valid
- Increment address and decrement bytes-left

Write role in `STATE_REQ`:
- Prefetch from FIFO into internal queue
- Issue write when buffered data is available
- On handshake, consume buffered beat
- Increment address and decrement bytes-left

### 4.4 FIFO and Flow Control
- FIFO decouples producer (read side) and consumer (write side)
- Read side uses near-full protection
- Write side uses staged buffering to maintain forward progress under ack variability

### 4.5 Corner Cases
- Misaligned byte count: rejected
- `start` while busy: ignored
- Descriptor values are locked at accepted start and not changed mid-transfer

## 5. Performance and Capacity

### 5.1 Main Limits
- Beat size = `DATA_WIDTH/8` bytes
- FIFO capacity set by `FIFO_DEPTH`
- Streamer prefetch/drain behavior bounded by `BURST_MAX` and internal queue depth

### 5.2 What Affects Throughput
- Source-side acknowledge speed
- Destination-side acknowledge speed
- FIFO occupancy behavior
- Write-side prefetch effectiveness

### 5.3 What Affects Latency
- Start acceptance delay
- Per-beat handshake delay
- FIFO staging delay
- Write queue dispatch delay

## 6. Robustness and Signoff Risks

### 6.1 Expected Stable Behavior
- No descriptor mutation during active transfer
- No interleaving of multiple descriptors in one DMA instance
- Forward progress under bounded backpressure

### 6.2 Practical Risk Checklist
- Long destination stalls can reduce throughput sharply
- FIFO edge behavior should be stress-tested near full/empty transitions
- Downstream non-responsiveness can dominate completion time

### 6.3 Bring-Up Debug Checklist
- Check `start` pulse width and idle acceptance
- Check mode mapping (`stream_type`) against expected source/sink
- Check `num_bytes` alignment
- Check whether read side is filling FIFO and write side is draining FIFO
- Check if `busy` sticks due to missing downstream ack

## 7. Verification Traceability

### 7.1 Current Baseline in Repo
The following collateral now exists for DMA verification handoff.

```text
+----------------+----------------------------------------------+---------------------------------------------------------------+---------------+
| Artifact ID    | File                                         | Purpose                                                       | Status        |
+----------------+----------------------------------------------+---------------------------------------------------------------+---------------+
| DMA-VTB-001    | tb_dma_flat_full_duplex.v (only DMA)         | DMA-only directed/self-checking testbench                     | Implemented   |
| DMA-VTB-002    | tb_simple_integration_e2e.v                  | System-level integration regression wrapper                   | Implemented   |
| DMA-VTB-003    | tb_simple_integration.v                      | Existing directed integration scenario                         | Implemented   |
| DMA-MON-001    | dma_spi8_xbar_system_monitor.v               | Transaction-level monitor at system level                      | Implemented   |
+----------------+----------------------------------------------+---------------------------------------------------------------+---------------+
```

### 7.2 DMA-Only Test Scenarios Executed (TC1-TC8)
The following scenarios were executed in `tb_dma_flat_full_duplex.v` and are intended as the seed suite for the Verification team.

```text
+---------+---------------------------------------------------------------+---------------------------+-----------------------------------------------+-----------------------------------------------+--------+
| TC ID   | Scenario                                                      | Stimulus                  | Expected Result                                | Actual Result                                 | Status |
+---------+---------------------------------------------------------------+---------------------------+-----------------------------------------------+-----------------------------------------------+--------+
| TC1     | P2M basic transfer                                            | 64B (8 beats)             | Destination memory data matches source pattern | Destination memory data matched source pattern | PASS   |
| TC2     | M2P basic transfer                                            | 64B (8 beats)             | Destination peripheral data matches source     | Destination peripheral data matched source     | PASS   |
| TC3     | start-while-busy handling                                    | 128B transfer + overlap   | Overlap start ignored; active transfer intact  | Overlap start ignored; active transfer intact  | PASS   |
| TC4     | Misaligned length rejection                                  | num_bytes=14              | No normal write traffic on rejected request    | No normal write traffic observed               | PASS   |
| TC5     | Zero-length request                                          | num_bytes=0               | Completes without bus traffic                  | Completed without bus traffic                  | PASS   |
| TC6     | Variable-latency backpressure stress                         | 128B P2M + 128B M2P       | Data integrity under variable source/sink ack  | Data integrity held under variable latency     | PASS   |
| TC7     | Explicit burst case (exact burst length)                     | 128B (16 beats) both dirs | Full-beat burst transfer integrity             | 16-beat burst transfer integrity confirmed     | PASS   |
| TC8     | Explicit burst case (multi-chunk burst transfer)             | 256B (32 beats) both dirs | Multi-burst-chunk transfer integrity           | Multi-burst-chunk integrity confirmed          | PASS   |
+---------+---------------------------------------------------------------+---------------------------+-----------------------------------------------+-----------------------------------------------+--------+
```

### 7.3 What Was Checked (Not Handshake-Only)
Checks currently implemented in DMA-only TB:
- End-data scoreboard checks at destination side (per beat/address)
- Direction legality check for P2M: memory side write, peripheral side read
- Direction legality check for M2P: memory side read, peripheral side write
- Address alignment checks on active requests (`addr[2:0]==0`)
- Source/sink address progression checks (+8 bytes per acknowledged beat)
- Beat-count consistency checks at completion (`done`)
- Timeout-based forward-progress checks for each testcase

### 7.4 Gaps and Open Coverage for Verification Team
The current suite is a strong directed baseline, but signoff gaps remain:
- No assertion/SVA-based protocol properties yet (current checks are procedural)
- No constrained-random traffic generation (latency is patterned, not randomized by seed)
- No coverage model/UCDB reporting yet for mode/length/backpressure bins
- No explicit near-4KB-boundary burst split testcase in DMA-only TB
- No long-run regression with randomized descriptor sequences

### 7.5 Scope for Verification Team (Build-On Plan)
Recommended next steps for DV expansion:
- Convert key procedural checks into assertions for accepted-start behavior
- Convert key procedural checks into assertions for done/beat-count consistency
- Convert key procedural checks into assertions for direction legality invariants
- Add constrained-random tests for randomized `num_bytes` (aligned, misaligned, zero)
- Add constrained-random tests for randomized source/sink ack stalls
- Add constrained-random tests for randomized start timing including overlap attempts
- Add coverage model bins for mode (`M2P`, `P2M`)
- Add coverage model bins for length class (0, 1 beat, short burst, full burst, multi-burst)
- Add coverage model bins for backpressure class (source-limited, sink-limited, mixed)
- Add focused corner test for 4KB boundary behavior
- Add focused corner test for FIFO near-empty/near-full stress windows
- Add focused corner test for repeated back-to-back descriptors

### 7.6 Requirement-to-Verification Mapping
```text
+-------------+--------------------------------------------------------------+-------------------------------------------------------------+--------------------------------------------------------------+-------------+
| Req ID      | Requirement                                                  | Current Evidence                                            | Next Verification Step                                        | Owner       |
+-------------+--------------------------------------------------------------+-------------------------------------------------------------+--------------------------------------------------------------+-------------+
| DMA-REQ-001 | Descriptor latches only on accepted idle start               | TC1/TC2/TC6/TC7/TC8 exercise accepted start                | Add assertion for start acceptance/ignore semantics           | Verification|
| DMA-REQ-002 | Misaligned num_bytes is rejected                             | TC4                                                        | Add assertion proving no progress on rejected config          | Verification|
| DMA-REQ-003 | done only after complete transfer progression                | TC1-TC8 done checks + beat-count checks                    | Add assertion linking done to expected beat retirement        | Verification|
| DMA-REQ-004 | start while busy is ignored safely                           | TC3                                                        | Add random overlap-start stress with coverage                 | Verification|
| DMA-REQ-005 | Burst and multi-burst transfer data integrity                | TC7 (16 beats), TC8 (32 beats)                             | Add 4KB-boundary burst split testcase                         | Verification|
| DMA-REQ-006 | Robust behavior under backpressure                           | TC6 variable latency + protocol checks                      | Add constrained-random backpressure sweeps                    | Verification|
+-------------+--------------------------------------------------------------+-------------------------------------------------------------+--------------------------------------------------------------+-------------+
```

### 7.7 Signoff-Ready Exit Criteria (DMA)
- Directed suite (TC1-TC8) remains passing in CI
- Assertion suite is added and passing cleanly
- Constrained-random regressions complete with no protocol/data failures
- Functional coverage goals are met for mode/length/backpressure/error bins
- Requirement mapping table is closed out with evidence links for each requirement

### 7.8 Reproducibility Note
Current DMA-only results are from `tb_dma_flat_full_duplex.v` under Verilator, with protocol checks and data scoreboarding enabled.
