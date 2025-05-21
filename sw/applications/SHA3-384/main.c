#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "fips202.h"

//#ifdef PERF_CNT_CYCLES
    #include "core_v_mini_mcu.h"
    #include "csr.h"
//#endif


int main(void) {
    int result = 0;
    uint8_t input[200];
    uint8_t t[48];
    //#if PERF_CNT_CYCLES
        unsigned int cycles, cycles2;
    //#endif

    for (int i = 0; i < 200; i++) {
        input[i] = i;
    }

    printf("Test started!\n");
    //#ifdef PERF_CNT_CYCLES
        CSR_CLEAR_BITS(CSR_REG_MCOUNTINHIBIT, 0x1);
        CSR_WRITE(CSR_REG_MCYCLE, 0);
    //#endif
        sha3_384(t, input, 200);
    //#ifdef PERF_CNT_CYCLES
        CSR_READ(CSR_REG_MCYCLE, &cycles);
        printf("Number of clock cycles for test-1 : %d\n", cycles);
    //#endif

    printf("Test: terminated\n");


    //printf("Received: ");
    //for (int i = 0; i < 48; i++) {
    //    printf("%02X", t[i]);
    //}

    return result;
}
