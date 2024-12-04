// Copyright 2024 EPFL and Politecnico di Torino.
// Solderpad Hardware License, Version 2.1, see LICENSE.md for details.
// SPDX-License-Identifier: Apache-2.0 WITH SHL-2.1
//
// File: gr_heep_tb.cpp
// Author: Michele Caon, Luigi Giuffrida
// Date: 16/10/2024
// Description: Verilator C++ testbench for GR-HEEP

// System libraries
#include <cstdlib>
#include <cstdio>
#include <getopt.h>
#include <stdint.h>
#include <errno.h>

// Verilator libraries
#include <verilated.h>
#include <verilated_fst_c.h>
#include <svdpi.h>

// User libraries
#include "tb_macros.hh"
#include "Vtb_system.h"
// For the bridge
#include <iostream>
#include <fstream>
// -------- Bridge2Xheep --------
#include "Bridge2Xheep.h"
// ------------------------------

// Defines
// -------
#define FST_FILENAME "logs/waves.fst"
#define PRE_RESET_CYCLES 200
#define RESET_CYCLES 200
#define POST_RESET_CYCLES 50
#define MAX_SIM_CYCLES 2e6
#define BOOT_SEL 0        // 0: JTAG boot
#define EXEC_FROM_FLASH 0 // 0: do not execute from flash
#define RUN_CYCLES 500
#define TB_HIER_NAME "TOP.tb_system"
// -------- Bridge2Xheep --------
#define NUMBER_0_9 48 // '0' = 48 in ASCII
#define NUMBER_A_F 55 // 'A' = 65 in ASCII
//  ------------------------------

// Data types
// ----------
enum boot_mode_e
{
    BOOT_MODE_JTAG = 0,
    BOOT_MODE_FLASH = 1,
    BOOT_MODE_FORCE = 2
};

// Function prototypes
// -------------------
// Process runtime parameters
std::string getCmdOption(int argc, char *argv[], const std::string &option);

// DUT initialization
void initDut(Vtb_system *dut, uint8_t boot_mode, uint8_t exec_from_flash);

// Generate clock and reset
void clkGen(Vtb_system *dut);
void rstDut(Vtb_system *dut, uint8_t gen_waves, VerilatedFstC *trace);

// Run simulation for the specififed number of cycles
void runCycles(unsigned int ncycles, Vtb_system *dut, uint8_t gen_waves, VerilatedFstC *trace);

// ---------- Bridge2Xheep Functions Prototypes -----------
void genReqBridge(std::ifstream &hex_file, Vtb_system *dut, Drv *drv, ReqBridge *req);
// ---------------------------------------------

// Global variables
// ----------------
// Testbench logger
TbLogger logger;
vluint64_t sim_cycles = 0;

// ------ Global variables for instruction file reading -----
char tmp_hex;
int ascii_offset;
int isValidChar = 1;

// Address related variables
unsigned int address = 0;
int isNewAddress = 0;
int addrEmptyByte = 7;

// Instructions related variables
unsigned int instruction = 0;
int instrFilledByte = 0;

int k = 1;
int tmp_instruction = 0;
// ----------------------------------------------------------

int main(int argc, char *argv[])
{
    // Exit value
    int exit_val = EXIT_SUCCESS;

    // COMMAND-LINE OPTIONS
    // --------------------
    // Define command-line options
    bool gen_waves = true;
    bool no_err = false;
    const option longopts[] = {
        {"help", no_argument, NULL, 'h'},
        {"log_level", required_argument, NULL, 'l'},
        {"trace", required_argument, NULL, 't'},
        {"no_err", required_argument, NULL, 'q'},
        {NULL, 0, NULL, 0}};

    // Parse command-line options
    int opt;
    while ((opt = getopt_long(argc, argv, "hl:t:q:", longopts, NULL)) >= 0)
    {
        switch (opt)
        {
        case 'h':
            printf("Usage: %s [OPTIONS]\n", argv[0]);
            printf("Options:\n");
            printf("  -h, --help\t\t\tPrint this help message\n");
            printf("  -l, --log_level=LOG_LEVEL\tSet the log level\n");
            printf("  -t, --trace=[true/false]\t\tGenerate waveforms\n");
            printf("  -q, --no_err=[true/false]\t\t\tAlways return 0\n");
            exit(0);
            break;
        case 'l':
            logger.setLogLvl(optarg);
            break;
        case 't':
            if (strcmp(optarg, "1") == 0 || strcmp(optarg, "true") == 0)
            {
                gen_waves = true;
            }
            break;
        case 'q':
            if (strcmp(optarg, "1") == 0 || strcmp(optarg, "true") == 0)
            {
                no_err = true;
            }
            break;
        default:
            printf("Usage: %s [OPTIONS]\n", argv[0]);
            printf("Try '%s --help' for more information.\n", argv[0]);
            exit(1);
            break;
        }
    }

    // Parse the remaining command-line arguments
    // ------------------------------------------
    std::string boot_mode_str;
    unsigned int boot_mode = 0;
    std::string firmware_file;
    std::string max_cycles_str;
    unsigned long max_cycles = MAX_SIM_CYCLES;

    // Boot mode
    boot_mode_str = getCmdOption(argc, argv, "+boot_mode=");
    if (boot_mode_str == "jtag" || boot_mode_str == "0")
    {
        boot_mode = BOOT_MODE_JTAG;
    }
    else if (boot_mode_str == "flash" || boot_mode_str == "1")
    {
        boot_mode = BOOT_MODE_FLASH;
    }
    else if (boot_mode_str == "force" || boot_mode_str == "2")
    {
        boot_mode = BOOT_MODE_FORCE;
    }
    else
    {
        TB_WARN("Invalid boot mode '%s'. Defaulting to JTAG", boot_mode_str.c_str());
        boot_mode_str = "jtag";
        boot_mode = BOOT_MODE_JTAG;
    }

    // Firmware HEX file
    firmware_file = getCmdOption(argc, argv, "+firmware=");
    if (firmware_file.empty())
    {
        TB_ERR("No firmware file specified");
        exit(EXIT_FAILURE);
    }
    /*else
    {
        // Check if file exists
        FILE *fp = fopen(firmware_file.c_str(), "r");
        if (fp == NULL)
        {
            TB_ERR("Cannot open firmware file '%s': %s", firmware_file.c_str(), strerror(errno));
            exit(EXIT_FAILURE);
        }
    }*/

    // Added for the bridge. Open the file with instructions
    std::ifstream hex_file;

    hex_file.open(firmware_file.c_str());

    if(!hex_file){
        std::cerr << "[TESTBENCH]: Error opening hex instruction file " << firmware_file.c_str() << std::endl;
        return -1;
    }

    // Max simulation cycles
    max_cycles_str = getCmdOption(argc, argv, "+max_cycles=");
    if (!max_cycles_str.empty())
    {
        max_cycles = std::stoul(max_cycles_str);
    }

    // Testbench initialization
    // ------------------------
    // Create log directory
    if (gen_waves)
        Verilated::mkdir("logs");

    // Create Verilator simulation context
    VerilatedContext *cntx = new VerilatedContext;
    cntx->commandArgs(argc, argv);
    if (gen_waves)
        cntx->traceEverOn(true);

    // Pass the simulation context to the logger
    logger.setSimContext(cntx);

    // Instantiate the DUT
    Vtb_system *dut = new Vtb_system(cntx);
    // ----------- Instantiate Bridge2Xheep components ---------
    Drv *drv = new Drv(dut);
    ReqBridge *req = new ReqBridge;
    // ---------------------------------------------------------

    // Set the file to store the waveforms in
    VerilatedFstC *trace = NULL;
    if (gen_waves)
    {
        trace = new VerilatedFstC;
        dut->trace(trace, 10);
        trace->open(FST_FILENAME);
    }

    // Set scope for DPI functions
    svSetScope(svGetScopeFromName(TB_HIER_NAME));
    svScope scope = svGetScope();
    if (scope == 0)
    {
        TB_ERR("svSetScope(): failed to set scope for DPI functions to %s", TB_HIER_NAME);
        exit(EXIT_FAILURE);
    }

    // Print testbench configuration
    // -----------------------------
    TB_CONFIG("Log level set to %u", logger.getLogLvl());
    TB_CONFIG("Waveform tracing %s", gen_waves ? "enabled" : "disabled");
    TB_CONFIG("Max simulation cycles set to %lu", max_cycles);
    TB_CONFIG("Boot mode: %s", boot_mode_str.c_str());
    TB_CONFIG("Firmware: %s", firmware_file.c_str());
    TB_CONFIG("Executing from %s", EXEC_FROM_FLASH ? "flash" : "RAM");

    // RUN SIMULATION
    // --------------
    TB_LOG(LOG_MEDIUM, "Starting simulation");

    // Initialize the DUT
    initDut(dut, boot_mode, EXEC_FROM_FLASH);

    // Reset the DUT
    rstDut(dut, gen_waves, trace);

    // Load firmware to SRAM
    switch (boot_mode)
    {
    case BOOT_MODE_JTAG:
        TB_LOG(LOG_LOW, "Waiting for JTAG (e.g., OpenOCD) to load firmware...");
        break;

    case BOOT_MODE_FORCE:
        TB_LOG(LOG_LOW, "Loading firmware...");
        TB_LOG(LOG_MEDIUM, "- writing firmware to SRAM...");
        //dut->tb_loadHEX(firmware_file.c_str());

        // Firmware loading procedure replaced by the SW bridge
        while (hex_file) // This loop goes on until the end of file is reached
        {
            // Generate clock
            clkGen(dut);

            // Call the method to generate the request
            genReqBridge(hex_file, dut, drv, req);

            // Send request to the driver
            //drv->drive(req);
            // Commented since in the hw implementation the bridge start a new transation automatically when
            // the instruction valid flag is set.
            if (req->address < 0x180){
                dut->inst_valid_i = 0;
            } 
            else if (req->address >= 0x180 && req->address < 0x1ec){
                dut->inst_valid_i = req->valid;
                dut->instruction_i = req->instruction;
                req->valid = 0;
            } 
            else if (req->address >= 0x1ec && req->address < 0x5100){
                dut->inst_valid_i = req->valid;
                dut->instruction_i = req->instruction;
                req->valid = 0;
            } 
            else if (req->address >= 0x5100 && req->address < 0xe800){
                dut->inst_valid_i = req->valid;
                dut->instruction_i = req->instruction;
                req->valid = 0;
            }
            else if ( req->address >= 0xe800 && req->address < 0xec78){
                dut->inst_valid_i = req->valid;
                dut->instruction_i = req->instruction;
                req->valid = 0;
            }
            else if (req->address >= 0xec78 && req->address < 0xedb0){
                dut->inst_valid_i = req->valid;
                dut->instruction_i = req->instruction;
                req->valid = 0;
            }
            else if (req->address >= 0xedb0){
                dut->inst_valid_i = req->valid;
                dut->instruction_i = req->instruction;
                req->valid = 0;
            }
            else {
                dut->inst_valid_i = 1;
                dut->instruction_i = 0;
            }

            // Evaluate the DUT
            dut->eval();

            // Save waveforms
            if (gen_waves) trace->dump(cntx->time());
            cntx->timeInc(1);
        }

        runCycles(1, dut, gen_waves, trace);
        TB_LOG(LOG_MEDIUM, "- triggering boot loop exit...");
        dut->tb_set_exit_loop();
        runCycles(1, dut, gen_waves, trace);
        TB_LOG(LOG_LOW, "Firmware loaded. Running app...");
        break;

    case BOOT_MODE_FLASH:
        TB_LOG(LOG_LOW, "Waiting for boot code to load firmware from flash...");
        break;

    default:
        TB_ERR("Invalid boot mode: %d", boot_mode);
        exit(EXIT_FAILURE);
    }

    // Run until the end of simulation is reached
    while (!cntx->gotFinish() && cntx->time() < (max_cycles << 1) && dut->exit_valid_o == 0)
    {
        TB_LOG(LOG_FULL, "Running %lu cycles...", RUN_CYCLES);
        runCycles(RUN_CYCLES, dut, gen_waves, trace);
    }
    if (cntx->time() >= (max_cycles << 1))
    {
        TB_WARN("Max simulation cycles reached");
    }

    // Print simulation status
    TB_LOG(LOG_LOW, "Simulation complete");

    // Check exit value
    if (dut->exit_valid_o)
    {
        TB_LOG(LOG_LOW, "Exit value: %d", dut->exit_value_o);
        exit_val = dut->exit_value_o;
        runCycles(10, dut, gen_waves, trace);
    }
    else
    {
        TB_ERR("No exit value detected");
        exit_val = EXIT_FAILURE;
    }

    // CLEAN UP
    // --------
    // Simulation complete
    dut->final();

    // Clean up and exit
    if (gen_waves)
        trace->close();
    delete dut;
    delete cntx;
    // ----------- Clean up Bridge2Xheep components -----------
    delete drv;
    delete req;
    hex_file.close();
    // ------------------------------------------------------
    if (no_err)
        exit(EXIT_SUCCESS);
    exit(exit_val);
}

void initDut(Vtb_system *dut, uint8_t boot_mode, uint8_t exec_from_flash)
{
    // Clock and reset
    dut->clk_i = 0;
    dut->rst_ni = 1;
    /*
    // Bridge Signals
    dut->req_i = 0;
    dut->we_i = 0;
    dut->be_i = 0;
    dut->addr_i = 0;
    dut->wdata_i = 0;
    */

    // Static configuration
    dut->boot_select_i = boot_mode == BOOT_MODE_FLASH;
    dut->execute_from_flash_i = exec_from_flash;
    dut->eval();
}

void clkGen(Vtb_system *dut)
{
    dut->clk_i ^= 1;
}

void rstDut(Vtb_system *dut, uint8_t gen_waves, VerilatedFstC *trace)
{
    dut->rst_ni = 1;
    TB_LOG(LOG_MEDIUM, "Resetting DUT...");
    runCycles(PRE_RESET_CYCLES, dut, gen_waves, trace);
    dut->rst_ni = 0;
    TB_LOG(LOG_MEDIUM, "- reset asserted");
    runCycles(RESET_CYCLES, dut, gen_waves, trace);
    TB_LOG(LOG_MEDIUM, "- reset released");
    dut->rst_ni = 1;
    runCycles(POST_RESET_CYCLES, dut, gen_waves, trace);
}

void runCycles(unsigned int ncycles, Vtb_system *dut, uint8_t gen_waves, VerilatedFstC *trace)
{
    VerilatedContext *cntx = dut->contextp();
    for (unsigned int i = 0; i < (2 * ncycles); i++)
    {
        // Generate clock
        clkGen(dut);

        // Evaluate the DUT
        dut->eval();

        // Save waveforms
        if (gen_waves)
            trace->dump(cntx->time());
        if (dut->clk_i == 1)
            sim_cycles++;
        cntx->timeInc(1);
    }
}

std::string getCmdOption(int argc, char *argv[], const std::string &option)
{

    std::string cmd;
    for (int i = 0; i < argc; ++i)
    {
        std::string arg = argv[i];
        size_t arg_size = arg.length();
        size_t option_size = option.length();

        if (arg.find(option) == 0)
        {
            cmd = arg.substr(option_size, arg_size - option_size);
        }
    }
    return cmd;
}

void genReqBridge(std::ifstream &hex_file, Vtb_system *dut, Drv *drv, ReqBridge *req)
{
    if (hex_file)
    {
        if (!dut->clk_i)
        {
            if (!(dut->busy_o))
            {
                isValidChar = 1;

                hex_file.get(tmp_hex);

                // transform the char hex value to an integer
                if (tmp_hex >= '0' && tmp_hex <= '9')
                {
                    ascii_offset = NUMBER_0_9;
                }
                else if (tmp_hex >= 'A' && tmp_hex <= 'F')
                {
                    ascii_offset = NUMBER_A_F;
                }
                else if (tmp_hex == '@')
                {
                    isNewAddress = 1;
                }
                else
                {
                    isValidChar = 0;
                }

                if (isNewAddress)
                {
                    if (addrEmptyByte >= 0)
                    {
                        // It may happen that the line does not contain always 8 hex values per instruction.
                        // In this case, we need to fill the empty bytes with 0 before setting the new address.
                        // However, those zeros will become MSB, so there is no need to add them manually.
                        if (instrFilledByte > 0 && instrFilledByte < 4)
                        {
                            instrFilledByte = 0;
                            // Call set instr method
                            req->instruction = instruction;
                            req->valid = 1;
                            instruction = 0;
                        }
                        else if (tmp_hex != '@')
                        {
                            // Since 0 is decimal 48 in ASCII, we need to subtract 48 to get the correct value.
                            // Since A is decimal 65 in ASCII, we need to subtract 65 and add 10 to get the correct value for A-F.
                            // We shift the value to the left by 4*i to get the correct address, since the address is 32 bit
                            // and we are reading 8 hex values starting from the MSB.
                            address += ((tmp_hex - ascii_offset) << 4 * addrEmptyByte);
                            addrEmptyByte--;
                        }
                    }
                    else
                    {
                        isNewAddress = 0;
                        addrEmptyByte = 7;
                        // call setAddress Method
                        req->address = address;

                        // Workaround to avoid misalignement of instructions at 0x180.
                        // In fact, instruction with address lower than 0x180 are not valid and should be discarded.
                        // However, the main.hex file contains instruction with address lower than 0x180.
                        // These instructions are read anyway and the valid flag is set to 1 each time a new
                        // instruction is valid. Due to this behavior, as soon the address 0x180 is read, since the
                        // previous instrucion set valid to 1, a wrong instrucion is sent to the bridge and then written
                        // in the RAM, causing malfunctioning of the system. Setting the valid flag to 0 when the address
                        // is exactly 0x180 avoids this issue. This does not affect the correct behavior of the system
                        // since the instructions are read always after the address and as soon the correct instruction for the
                        // address 0x180 is read, a new valid is set to 1.
                        // For all others jumps, there is not this issue, since the drive method in the bridge manages to set valid
                        // to 0 when the instruction is written in the RAM.
                        if (req->address == 0x180)
                        {
                            req->valid = 0;
                        }

                        address = 0;
                    }
                }
                else
                {
                    if (isValidChar)
                    {
                        // Here is more complex. Since the instructions are in the form 97 F1 00 00 but they need to be
                        // written in the form 00 00 F1 97, we need first to read 2 hex values and shift the first one
                        // to the left by 4 bits. Then the sum of the 2 hex values is shifted to the left a number of bytes
                        // proportional to the number of hex values read so far.
                        // When an instrucion is complete, we call the setInstr method.
                        tmp_instruction += (tmp_hex - ascii_offset) << 4 * k;
                        k--;

                        if (k == -1)
                        {
                            if (instrFilledByte < 4)
                            {
                                instruction += (tmp_instruction << (8 * instrFilledByte));
                                instrFilledByte++;
                            }

                            k = 1;
                            tmp_instruction = 0;
                        }
                    }
                    else if (instrFilledByte == 4)
                    {
                        instrFilledByte = 0;
                        // Call set instr method
                        req->instruction = instruction;
                        req->valid = 1;
                        instruction = 0;
                    }
                }
            }
        }
    }
}