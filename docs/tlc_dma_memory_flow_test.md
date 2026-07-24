# TL-C DMA to Memory Flow

This document lists the RTL/testbench files used for the current DMA -> TL-C crossbar -> memory flow, and the test cases currently covered by simulation.

## File Tree

```text
cache_coherence/
├── docs/
│   └── tlc_dma_memory_flow_test.md
├── tlc/
│   ├── tb_tlc_dma_system_basic_v3.v
│   ├── tb_tlc_dma_system_basic_v3_init.mem
│   ├── tlc_dma_system_with_wrapper_top.v
│   ├── tlc_xbar_fabric_top.v
│   ├── tlc_slave_mem_manager.v
│   ├── l1_tilelink_adapter.v
│   ├── tl_xbar_nm.v
│   ├── tl_arbiter.v
│   ├── source_id_manager.v
│   └── tlc64b2M_params.v
└── xspi_dma_memory_wk2609/
    ├── dma/
    │   ├── dma_tlc_master_wrapper.v
    │   ├── dma_coherent_agent.v
    │   └── dma_flat_full_duplex_top.v
    └── memory/
        ├── raw_sram_ctrl_top.v
        ├── bank_arbiter.v
        ├── sram_bank.v
        ├── sram_model.v
        └── tl_sram_ctrl_pkg.vh
```

## What Each File Does

### Testbench and Test Data

- `tlc/tb_tlc_dma_system_basic_v3.v`
  - Main Verilator testbench
  - Drives DMA control
  - Models the peripheral-side source/sink traffic
  - Checks data correctness

- `tlc/tb_tlc_dma_system_basic_v3_init.mem`
  - Memory preload file used through `MEM_INIT_FILE`
  - Seeds the SRAM contents for the "DMA reads existing memory data" test

### Top-Level Integration

- `tlc/tlc_dma_system_with_wrapper_top.v`
  - Full top used by the testbench
  - Instantiates DMA wrapper, TL-C fabric, slave managers, and raw memory blocks

- `tlc/tlc_xbar_fabric_top.v`
  - TL-C fabric wrapper
  - Connects masters to slave-side TL-C channels

### TL-C Fabric / Protocol Support

- `tlc/tlc_slave_mem_manager.v`
  - Coherent slave-side manager
  - Tracks requests
  - Talks TL-C on the crossbar side
  - Drives raw `mem_*` on the memory side

- `tlc/l1_tilelink_adapter.v`
  - Adapter used by the DMA coherent side to speak TL-C request/response/probe signaling

- `tlc/tl_xbar_nm.v`
  - Crossbar routing/arbitration logic

- `tlc/tl_arbiter.v`
  - Arbitration helper used by the TL-C fabric

- `tlc/source_id_manager.v`
  - Source ID allocation/tracking support

- `tlc/tlc64b2M_params.v`
  - TL-C parameter/opcode definitions used by multiple TL-C modules

### DMA Path

- `xspi_dma_memory_wk2609/dma/dma_tlc_master_wrapper.v`
  - Wraps DMA core and coherent agent together
  - Exposes DMA control/peripheral interface and TL-C master interface

- `xspi_dma_memory_wk2609/dma/dma_coherent_agent.v`
  - Bridges DMA `mem_*` traffic into TL-C coherent transactions
  - Handles acquire/grant/writeback sequencing

- `xspi_dma_memory_wk2609/dma/dma_flat_full_duplex_top.v`
  - DMA datapath/core used to generate memory-side read/write requests

### Raw Memory Path

- `xspi_dma_memory_wk2609/memory/raw_sram_ctrl_top.v`
  - Raw memory top without TL-UH interface
  - Accepts raw `mem_req`, `mem_we`, `mem_addr`, `mem_wdata`, `mem_wmask`

- `xspi_dma_memory_wk2609/memory/bank_arbiter.v`
  - Maps raw memory requests into memory-bank requests

- `xspi_dma_memory_wk2609/memory/sram_bank.v`
  - Bank wrapper around the actual SRAM model

- `xspi_dma_memory_wk2609/memory/sram_model.v`
  - Backing SRAM storage model
  - Supports `INIT_FILE`

- `xspi_dma_memory_wk2609/memory/tl_sram_ctrl_pkg.vh`
  - Shared package/header used by the raw SRAM control path

## Full Flow Being Tested

```text
Peripheral stimulus
  -> dma_flat_full_duplex_top.v
  -> dma_coherent_agent.v
  -> l1_tilelink_adapter.v
  -> tlc_xbar_fabric_top.v / tl_xbar_nm.v
  -> tlc_slave_mem_manager.v
  -> raw_sram_ctrl_top.v
  -> bank_arbiter.v
  -> sram_bank.v
  -> sram_model.v
```

And on DMA readback:

```text
sram_model.v
  -> raw_sram_ctrl_top.v
  -> tlc_slave_mem_manager.v
  -> TL-C D/C path
  -> dma_coherent_agent.v
  -> dma_flat_full_duplex_top.v
  -> peripheral sink capture in tb_tlc_dma_system_basic_v3.v
```

## Test Cases Covered

The current testbench covers 5 scenarios in one run.

### Test 1: First 8-Beat Peripheral to Memory DMA Write

- DMA writes 8 beats from peripheral-side stimulus into memory
- Address range:
  - Peripheral side: `0x00` to `0x38`
  - Memory side: `0x00` to `0x38`
- Pattern:
  - `0x1122334455000000` to `0x1122334455000007`

### Test 2: First 8-Beat Memory to Peripheral DMA Read

- DMA reads back the same 8 beats from memory
- Testbench checks returned sink data against expected source pattern

### Test 3: Second 8-Beat Peripheral to Memory DMA Write

- DMA writes a second, different 8-beat block after the first block is complete
- Address range:
  - Peripheral side: `0x40` to `0x78`
  - Memory side: `0x40` to `0x78`
- Pattern:
  - `0xA1B2C3D4E5000100` to `0xA1B2C3D4E5000107`

### Test 4: Second 8-Beat Memory to Peripheral DMA Read

- DMA reads back the second 8-beat block
- Testbench verifies all 8 beats

### Test 5: DMA Reads Pre-Existing Memory Data

- Memory is preloaded using `MEM_INIT_FILE`
- DMA reads data that was not written by DMA in the same run
- Address range:
  - `0x80` to `0xB8`
- Pattern:
  - `0xCCDDEE0077000200` to `0xCCDDEE0077000207`

## How Preload Works

The testbench now uses:

```verilog
tlc_dma_system_with_wrapper_top #(
    .MEM_INIT_FILE("tlc/tb_tlc_dma_system_basic_v3_init.mem")
) dut (...);
```

This `MEM_INIT_FILE` is passed down into:

- `raw_sram_ctrl_top.v`
- `sram_bank.v`
- `sram_model.v`

Inside `sram_model.v`, the preload happens through:

```verilog
$readmemh(INIT_FILE, mem);
```

## Current Simulation Result

The current extended testbench passes in Verilator:

- Test 1 pass
- Test 2 pass
- Test 3 pass
- Test 4 pass
- Test 5 pass

Final observed totals from the latest passing run:

- `src_hs_count = 16`
- `sink_hs_count = 24`
- `slv0_total_wr_count = 40`
- `slv0_total_rd_count = 40`

