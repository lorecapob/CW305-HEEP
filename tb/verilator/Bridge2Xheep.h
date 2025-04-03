/*
 * Copyright 2025 Politecnico di Torino.
 * Solderpad Hardware License, Version 2.1, see LICENSE.md for details.
 * SPDX-License-Identifier: Apache-2.0 WITH SHL-2.1
 *
 * Author: Lorenzo Capobianco
 * Date: 03/04/2025
 * Description: Bridge2Xheep header file.
 */

#ifndef BRIDGE2XHEEP_H
#define BRIDGE2XHEEP_H

#include <verilated.h>
#include "Vtb_system_cw305.h"

class ReqBridge
{
private:

public:
    vluint32_t instruction;
    vluint32_t address;
    vluint8_t instr_valid;
    vluint8_t addr_valid;

    vluint8_t ready;

    ReqBridge();
    ~ReqBridge();

};


class RespBridge
{
private:

public:
    RespBridge();
    ~RespBridge();
};


class Drv
{
private:
    Vtb_system_cw305 *dut;
    ReqBridge *req;

public:
    int state = 0;
    int busy = 0;

    Drv(Vtb_system_cw305 *dut);
    ~Drv();

    void drive(ReqBridge *req);
};

#endif
