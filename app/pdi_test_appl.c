//###########################################################################
//
// FILE:   pdi_test_appl.c
//
// TITLE:  EtherCAT(ET1100/ET1200) PDI (Processor Data Interface) example for
//         F2837x devices
//
//###########################################################################
#include "ethercat_slave_c28x_hal.h"

Uint16 read_data[10];
Uint16 write_data=0x0002;
Uint16 read_adder=0x130;
Uint16 write_adder=0x120;
uint16_t numbytes=2;
void main()
{
    ESC_initHW();

   while(1)
    {
       ESC_readSPI(read_adder,2,read_data);
       ESC_writeSPI(write_adder,&write_data,2);

     DELAY_US(10000);
    }
}


void PDI_Isr(void)
{
    return;
}
//
// End of File
//
