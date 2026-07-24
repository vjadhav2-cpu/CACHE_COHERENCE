// C++ Testbench Wrapper for Verilator
// Simulates tb_simple_integration.v behavior

#include <verilated.h>
#include <verilated_vcd_c.h>
#include "Vtb_simple_integration.h"
#include <iostream>
#include <iomanip>
#include <vector>

// Simulation time management
vluint64_t main_time = 0;
double sc_time_stamp() { return main_time; }

class Testbench {
public:
    Vtb_simple_integration *dut;
    VerilatedVcdC *tfp;
    std::vector<uint64_t> written_data;
    std::vector<uint64_t> read_data;
    int write_count;
    int read_count;
    bool spi_rd_req_prev;
    bool spi_wr_evt_prev;
    
    Testbench() {
        dut = new Vtb_simple_integration;
        tfp = nullptr;
        write_count = 0;
        read_count = 0;
        spi_rd_req_prev = false;
        spi_wr_evt_prev = false;
        written_data.resize(8, 0);
        read_data.resize(8, 0);
    }
    
    ~Testbench() {
        if (tfp) tfp->close();
        delete dut;
    }
    
    void enable_trace(const char* filename) {
        Verilated::traceEverOn(true);
        tfp = new VerilatedVcdC;
        dut->trace(tfp, 99);
        tfp->open(filename);
    }
    
    void tick() {
        // Positive edge
        dut->m_clk = 1;
        dut->eval();
        if (tfp) tfp->dump(main_time);
        main_time += 5;  // 5ns
        
        // Negative edge
        dut->m_clk = 0;
        dut->eval();
        if (tfp) tfp->dump(main_time);
        main_time += 5;  // 5ns (total 10ns period = 100MHz)
    }
    
    void reset() {
        dut->reset_m = 1;
        dut->start = 0;
        dut->stream_type = 0;
        dut->dma_mem_base_addr = 0;
        dut->dma_peri_base_addr = 0;
        dut->num_bytes = 0;
        dut->spi_rd_rsp_valid_tb = 0;
        dut->spi_rd_rsp_data_tb = 0;
        
        for (int i = 0; i < 10; i++) tick();
        
        dut->reset_m = 0;
        std::cout << "[" << main_time << "] Reset released" << std::endl;
        
        for (int i = 0; i < 5; i++) tick();
    }
    
    void handle_spi_read_request() {
        // Detect rising edge of spi_rd_req_valid_tb
        if (dut->spi_rd_req_valid_tb && !spi_rd_req_prev) {
            std::cout << "[" << main_time << "] SPI read request for addr 0x"
                      << std::hex << std::setw(12) << std::setfill('0')
                      << (uint64_t)dut->spi_rd_req_addr_tb << std::dec << std::endl;
            
            // Wait a bit then respond
            for (int i = 0; i < 2; i++) tick();
            
            dut->spi_rd_rsp_valid_tb = 1;
            // Generate pattern: replicate lower byte
            uint8_t pattern = dut->spi_rd_req_addr_tb & 0xFF;
            dut->spi_rd_rsp_data_tb = 0;
            for (int i = 0; i < 8; i++) {
                dut->spi_rd_rsp_data_tb |= ((uint64_t)pattern << (i * 8));
            }
            
            if (write_count < 8) {
                written_data[write_count] = dut->spi_rd_rsp_data_tb;
                std::cout << "[" << main_time << "] SPI read response [" << write_count
                          << "]: 0x" << std::hex << std::setw(16) << std::setfill('0')
                          << dut->spi_rd_rsp_data_tb << std::dec << " (STORED)" << std::endl;
                write_count++;
            }
            
            tick();
            dut->spi_rd_rsp_valid_tb = 0;
        }
        spi_rd_req_prev = dut->spi_rd_req_valid_tb;
    }
    
    void capture_spi_write_event() {
        // Detect rising edge of spi_wr_evt_valid_tb during read operation
        if (dut->spi_wr_evt_valid_tb && !spi_wr_evt_prev && dut->stream_type == 0) {
            if (read_count < 8) {
                read_data[read_count] = dut->spi_wr_evt_data_tb;
                std::cout << "[" << main_time << "] SPI Write Event [" << read_count
                          << "]: addr=0x" << std::hex << std::setw(12) << std::setfill('0')
                          << (uint64_t)dut->spi_wr_evt_addr_tb
                          << " data=0x" << std::setw(16)
                          << dut->spi_wr_evt_data_tb << std::dec << " (CAPTURED)" << std::endl;
                read_count++;
            }
        }
        spi_wr_evt_prev = dut->spi_wr_evt_valid_tb;
    }
    
    void run_write_test() {
        std::cout << "\n[" << main_time << "] TEST 1: WRITE - 64 bytes from SPI to Memory at addr 0x0000" << std::endl;
        
        dut->stream_type = 1;  // Write operation
        dut->dma_mem_base_addr = 0;
        dut->dma_peri_base_addr = 0;
        dut->num_bytes = 64;
        write_count = 0;
        
        tick();
        dut->start = 1;
        std::cout << "[" << main_time << "] Start asserted" << std::endl;
        tick();
        dut->start = 0;
        
        // Wait for busy
        while (!dut->busy && main_time < 100000) {
            handle_spi_read_request();
            capture_spi_write_event();
            tick();
        }
        std::cout << "[" << main_time << "] DMA busy asserted" << std::endl;
        
        // Run until done
        while (!dut->done && main_time < 5000000) {
            handle_spi_read_request();
            capture_spi_write_event();
            tick();
        }
        
        std::cout << "[" << main_time << "] DMA done asserted - WRITE complete" << std::endl;
        std::cout << "[" << main_time << "] Memory writes: " << dut->mem_total_wr_count
                  << ", Memory reads: " << dut->mem_total_rd_count << std::endl;
        
        for (int i = 0; i < 10; i++) tick();
    }
    
    void run_read_test() {
        std::cout << "\n[" << main_time << "] TEST 2: READ - 64 bytes from Memory to SPI at addr 0x0000" << std::endl;
        
        dut->stream_type = 0;  // Read operation
        dut->dma_mem_base_addr = 0;
        dut->dma_peri_base_addr = 0;
        dut->num_bytes = 64;
        read_count = 0;
        
        tick();
        dut->start = 1;
        std::cout << "[" << main_time << "] Start asserted" << std::endl;
        tick();
        dut->start = 0;
        
        // Wait for busy
        while (!dut->busy && main_time < 100000) {
            handle_spi_read_request();
            capture_spi_write_event();
            tick();
        }
        std::cout << "[" << main_time << "] DMA busy asserted" << std::endl;
        
        // Run until done
        while (!dut->done && main_time < 5000000) {
            handle_spi_read_request();
            capture_spi_write_event();
            tick();
        }
        
        std::cout << "[" << main_time << "] DMA done asserted - READ complete" << std::endl;
        std::cout << "[" << main_time << "] Memory writes: " << dut->mem_total_wr_count
                  << ", Memory reads: " << dut->mem_total_rd_count << std::endl;
        
        for (int i = 0; i < 10; i++) tick();
    }
    
    void verify_data() {
        std::cout << "\n========================================" << std::endl;
        std::cout << "DATA VERIFICATION" << std::endl;
        std::cout << "========================================" << std::endl;
        
        int errors = 0;
        for (int i = 0; i < 8; i++) {
            if (written_data[i] == read_data[i]) {
                std::cout << "[" << i << "] PASS: Written=0x" << std::hex << std::setw(16)
                          << std::setfill('0') << written_data[i] << ", Read=0x"
                          << read_data[i] << std::dec << " ✓" << std::endl;
            } else {
                std::cout << "[" << i << "] FAIL: Written=0x" << std::hex << std::setw(16)
                          << std::setfill('0') << written_data[i] << ", Read=0x"
                          << read_data[i] << std::dec << " ✗" << std::endl;
                errors++;
            }
        }
        
        std::cout << "\n========================================" << std::endl;
        if (errors == 0) {
            std::cout << "✓ ALL DATA VERIFIED - TEST PASSED" << std::endl;
        } else {
            std::cout << "✗ VERIFICATION FAILED - " << errors << " errors" << std::endl;
        }
        std::cout << "========================================" << std::endl;
    }
    
    void print_final_stats() {
        std::cout << "\n========================================" << std::endl;
        std::cout << "Test Complete" << std::endl;
        std::cout << "Final Memory Statistics:" << std::endl;
        std::cout << "  Total Writes: " << dut->mem_total_wr_count << std::endl;
        std::cout << "  Total Reads:  " << dut->mem_total_rd_count << std::endl;
        std::cout << "========================================\n" << std::endl;
    }
};

int main(int argc, char **argv) {
    Verilated::commandArgs(argc, argv);
    
    Testbench *tb = new Testbench();
    tb->enable_trace("tb_simple_integration_verilator.vcd");
    
    std::cout << "Simple Integration Test Starting (Verilator)" << std::endl;
    std::cout << "========================================\n" << std::endl;
    
    tb->reset();
    tb->run_write_test();
    tb->run_read_test();
    tb->verify_data();
    tb->print_final_stats();
    
    delete tb;
    return 0;
}
