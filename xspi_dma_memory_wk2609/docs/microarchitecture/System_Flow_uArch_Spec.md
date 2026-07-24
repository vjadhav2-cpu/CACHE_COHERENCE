# System Flow Microarchitecture Specification

## 1. Scope and Compliance

### 1.1 Scope
This document explains the complete transfer flow outside the DMA core so teammates can quickly reason about end-to-end behavior.

In scope:
- How data moves from peripheral side to memory side and back
- Role of adapters, bridges, wrappers, crossbar, and memory controller
- System behavior for M2P and P2M flows
- Integration-level backpressure, buffering, and debug visibility

Out of scope:
- Internal DMA block details (covered in `DMA_uArch_Spec.md`)
- Physical implementation details
- Software driver API

### 1.2 Conventions Used Here
- "Must" = required behavior for correctness
- "Should" = recommended behavior
- "Can" = optional behavior

### 1.3 Referenced RTL
- `spi_dma_xbar_memory_system.v`
- `dma_spi8_xbar_system_top.v`
- `spi8_to_xbar_adapter.v`
- `streamer_spi8_top.v`
- `dma_memc_to_xbar_m0_adapter.v`
- `tluh_bridge_into_wrapper_top.v`
- `bridge_spi8_top.v`
- `tilelink_uh_3M_2S.v`
- `tl_sram_ctrl_top.v`

## 2. Architecture Overview

### 2.1 Hierarchy (Outside DMA Core)
- `spi_dma_xbar_memory_system`
- `u_dma_xbar : dma_spi8_xbar_system_top`
- `u_spi8_to_xbar_adapter : spi8_to_xbar_adapter`
- `u_streamer_spi8_top : streamer_spi8_top`
- `u_dma_memc_to_xbar_m0_adapter : dma_memc_to_xbar_m0_adapter`
- `u_tilelink_uh_3M_2S : tilelink_uh_3M_2S`
- `u_memory : tl_sram_ctrl_top`

### 2.2 One-Line Mental Model
System flow is a chain: DMA-side requests get translated, routed through interconnect, serviced by banked memory, and responses travel back through the same integration boundaries.

### 2.3 End-to-End Intent by Mode
- M2P: memory path is source side, peripheral path is sink side
- P2M: peripheral path is source side, memory path is sink side

## 3. Interface Contract

### 3.1 System Boundary Contract
Top-level integration exposes:
- DMA control and status
- SPI request/response visibility taps
- TileLink master0 debug taps
- Memory operation summary counters

### 3.2 Internal Boundary Contract
Main interface boundaries:
- Streamer-facing `memc_*` boundary
- Adapter to crossbar master0 A/D channel boundary
- Crossbar slave0 to memory-controller A/D boundary
- Memory-controller to bank arbiter and bank RAM boundary

### 3.3 Handshake and Progress Rules
- A transaction beat is accepted only on qualified handshake (`valid/ready` or `req/ack` per boundary).
- Backpressure must propagate upstream through handshake signals.
- No boundary should silently drop accepted traffic.

### 3.4 Error Handling Expectations
- Contradictory requests at constrained boundaries should be flagged.
- Queue overflow conditions should be detectable and verifiable.
- Response error information should remain observable through adapter/integration signals.

## 4. Microarchitecture Definition

### 4.1 Peripheral-Side Flow
- Peripheral-side traffic is adapted through `bridge_spi8_top`.
- SPI-facing events and read requests are observable at dedicated TB/debug ports.
- Local bridge logic maps this behavior into DMA-facing transfer semantics.

### 4.2 Memory-Side Flow
- DMA memory-side traffic enters `tluh_bridge_into_wrapper_top`.
- Bridge/wrapper logic converts and stages transactions into TileLink-style channels.
- `dma_memc_to_xbar_m0_adapter` groups and dispatches requests toward crossbar master0.

### 4.3 Crossbar Traversal
- Master0 traffic enters `tilelink_uh_3M_2S`.
- Address/routing logic forwards memory-targeted traffic to slave0.
- Slave0 boundary connects to `tl_sram_ctrl_top`.

### 4.4 Memory Controller Subsystem
- `tl_uh_slave_if` converts TileLink traffic to internal memory request format.
- `bank_arbiter` selects destination bank and tracks response source.
- `sram_bank` instances service reads/writes with configured latencies.

### 4.5 Adapter Scheduling Behavior
- `dma_memc_to_xbar_m0_adapter` queues incoming requests.
- It can coalesce contiguous/aligned traffic into larger dispatch groups.
- Dispatch timing depends on queue depth and idle/flush conditions.

## 5. Performance and Capacity

### 5.1 Throughput Bottlenecks
- Peripheral bridge response cadence
- Adapter queue flush and dispatch policy
- Crossbar arbitration delays
- Memory bank readiness/latency

### 5.2 Buffering Points
- DMA FIFO (impacts whole-flow smoothness)
- Adapter request queue
- Single-entry staging points in bridge/wrapper paths

### 5.3 Latency Contributors
- Start-to-first-dispatch delay
- Protocol conversion/staging delay
- Crossbar traversal delay
- Memory-bank service delay

## 6. Robustness and Signoff Risks

### 6.1 Expected Stable Behavior
- No silent data loss under valid backpressure scenarios
- Forward progress when downstream readiness is eventually provided
- Deterministic completion for accepted transaction sequences

### 6.2 Practical Risk Checklist
- Queue head-of-line blocking under mixed request patterns
- Throughput drops under long asymmetric stalls
- Error visibility loss across protocol conversion boundaries

### 6.3 Bring-Up Debug Checklist
- Confirm mode intent matches observed source/sink path
- Confirm adapter queue drains under realistic backpressure
- Confirm crossbar slave0 routing for memory-address traffic
- Confirm memory response source tagging consistency
- Confirm end-to-end done behavior with randomized stalls

## 7. Verification Traceability

### 7.1 Current Baseline in Repo
```text
+----------------+--------------------------------------+-----------------------------------------------------------+-------------+
| Artifact ID    | File                                 | Purpose                                                   | Status      |
+----------------+--------------------------------------+-----------------------------------------------------------+-------------+
| SYS-VTB-001    | tb_simple_integration_e2e.v          | Top integration harness (TB + DUT connected end-to-end)  | Implemented |
| SYS-VTB-002    | tb_simple_integration.v              | Directed integration scenarios + data verification        | Implemented |
| SYS-VTB-003    | dma_spi8_xbar_system_stimulus.v      | Descriptor sequence stimulus for DMA+SPI8+xbar flow       | Implemented |
| SYS-MON-001    | dma_spi8_xbar_system_monitor.v       | Flow-level monitor with pass/fail style logging           | Implemented |
| SYS-VTB-004    | crossbar_tb.v                        | Crossbar-specific directed and burst traffic tests        | Implemented |
+----------------+--------------------------------------+-----------------------------------------------------------+-------------+
```

### 7.2 System-Level Scenarios Already Exercised
```text
+---------+--------------------------------------------------------+---------------------------------------+-----------------------------------------------------+--------+
| SC ID   | Scenario                                               | Stimulus                               | Expected Result                                     | Status |
+---------+--------------------------------------------------------+---------------------------------------+-----------------------------------------------------+--------+
| SC1     | End-to-end P2M write path                             | 64B transfer via integration TB        | Write flow completes and memory-side traffic seen    | PASS   |
| SC2     | End-to-end M2P read path                              | 64B transfer + readback capture        | Read flow completes and SPI write events observed    | PASS   |
| SC3     | End-to-end write+read data integrity loop             | Directed write then readback           | Captured read data matches written data              | PASS   |
| SC4     | Descriptor sequence sweep                             | Multiple vectors (64B/128B/16B)        | Each run reaches completion without protocol errors  | PASS   |
| SC5     | Crossbar single-beat + burst route checks             | Master/slave directed traffic           | Correct slave routing and response behavior          | PASS   |
+---------+--------------------------------------------------------+---------------------------------------+-----------------------------------------------------+--------+
```

### 7.3 Gaps and Open Coverage
- No unified assertion layer across adapter->xbar->memory boundaries yet
- No constrained-random system regression spanning all interfaces together
- Limited explicit coverage for long mixed traffic under sustained backpressure
- No formalized coverage closure report for route/utilization/backpressure bins

### 7.4 Scope for Verification Team (Build-On Plan)
- Add assertion checks for adapter queue dispatch/retire invariants
- Add assertion checks for crossbar source/response consistency
- Add assertion checks for memory-controller response correctness per source ID
- Add constrained-random end-to-end scenarios with independent source/sink stalls
- Add coverage bins for mode (`M2P`, `P2M`)
- Add coverage bins for traffic class (single, short burst, full burst, multi-burst)
- Add coverage bins for contention profile (no contention, crossbar contention, bank contention)
- Add long-run stability regression for repeated descriptor streams

### 7.5 Requirement-to-Verification Mapping
```text
+-------------+--------------------------------------------------------------+----------------------------------------------------------+---------------------------------------------------------------+-------------+
| Req ID      | Requirement                                                  | Current Evidence                                         | Next Verification Step                                         | Owner       |
+-------------+--------------------------------------------------------------+----------------------------------------------------------+---------------------------------------------------------------+-------------+
| SYS-REQ-001 | End-to-end control launches complete transfer path           | SC1/SC2/SC3                                              | Add assertion for launch-to-completion liveness                | Verification|
| SYS-REQ-002 | Adapter dispatch and completion behavior is correct          | SC4 + monitor visibility                                 | Add adapter queue/dispatch assertions + random stress          | Verification|
| SYS-REQ-003 | Crossbar routes master0 memory traffic to slave0 correctly   | SC5 crossbar tests                                       | Add scoreboard for route/response under mixed contention       | Verification|
| SYS-REQ-004 | Memory subsystem returns source-consistent responses         | SC1-SC5 indirect evidence                                | Add response-source assertion and memory-side scoreboard       | Verification|
| SYS-REQ-005 | System remains correct under backpressure                    | Directed/limited checks                                  | Add constrained-random backpressure sweeps                     | Verification|
+-------------+--------------------------------------------------------------+----------------------------------------------------------+---------------------------------------------------------------+-------------+
```

### 7.6 Signoff-Ready Exit Criteria (System Flow)
- Directed baseline scenarios remain passing in CI
- Assertion suite is added and passing for adapter/xbar/memory boundaries
- Constrained-random system regressions complete with no protocol/data failures
- Coverage closure is achieved for mode/traffic/contention/backpressure goals
- Requirement mapping table is closed with evidence links
