/*
 * Copyright 2025 Politecnico di Torino.
 * Solderpad Hardware License, Version 2.1, see LICENSE.md for details.
 * SPDX-License-Identifier: Apache-2.0 WITH SHL-2.1
 *
 * Author: Lorenzo Capobianco
 * Date: 03/04/2025
 * Description: Bridge2Xheep source file. Used for bridge software model simulation.
 */

#include <iostream>

#include "Bridge2Xheep.h"


ReqBridge::ReqBridge()
{
    this->instruction = 0;
    this->address = 0;
    this->instr_valid = 0;
    this->addr_valid = 0;
}

ReqBridge::~ReqBridge()
{
}

RespBridge::RespBridge()
{
}

RespBridge::~RespBridge()
{
}

Drv::Drv(Vtb_system_cw305 *dut)
{
    this->dut = dut;
    this->req = NULL;
}

Drv::~Drv()
{
}
/*
void Drv::drive(ReqBridge *req)
{
    /*
    dut->req_i                = 0;
    dut->we_i                 = 1;
    dut->be_i                 = 0b1111;
    dut->addr_i               = 0x180;
    dut->wdata_i              = 0;
    *//*

    if (req->instr_valid)
    {
        if (req->address >= 0x180)
        {   
            switch (this->state)
            {
            case 0:
                // IDLE state

                dut->req_i = 0;
                this->state = 1;
                this->busy = 1;
                break;

            case 1:
                // REQUEST_SENT state
                
                dut->req_i = 1;
                dut->addr_i = req->address;
                dut->wdata_i = req->instruction;
                dut->we_i = 1;
                dut->be_i = 0b1111;
                
                if (dut->gnt_o)
                {
                    this->state = 2;
                    this->busy = 0;
                } else {
                    this->state = 1;
                }

                break;

            case 2:
                dut->req_i = 0;
                dut->we_i = 0;

                req->address += 4;
                req->instr_valid = 0;
                
                this->state = 0;
                break;
            
            default:
                this->state = 0;
                this->busy = 0;
                break;
            }
        }
        
    }
    
}*/
