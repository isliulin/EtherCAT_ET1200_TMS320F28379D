/*******************************************************************************
 * 文件名称: ethercat_slave_c28x_hal.h
 * 文件说明: 用于EtherCAT从控制器(ESC)的C28x HAL硬件功能驱动
 * 功能说明:
 *         (1)本驱动适用于EtherCAT控制器ET1200/ET1100
 *         (2)本驱动可使用LaunchPad_F28379D评估板测试
 *            如果使用评估板,在全局宏定义中定义_LAUNCHXL_F28379D
 * 完成时间:
 *    版本:
 * 修改记录:
 * *****************************************************************************/
#include "F28x_Project.h"
#ifndef _ETHERCAT_SLAVE_C28x_HAL_H_
#define _ETHERCAT_SLAVE_C28x_HAL_H_

#ifdef __cplusplus
extern "C" {
#endif
/***********************************宏定义****************************************/
//-------------------------------使用的器件型号--------------------------------------
#define HAL_ET1100                   0
#define HAL_ET1200                   1
//---------------------------使用EtherCAT从站协议栈----------------------------------
#define ETHERCAT_STACK               0


extern uint16_t*            pEsc; //pointer to the ESC memory

//---------EtherCAT从站协议栈中需调用的中断函数(在EtherCAT从站堆栈中定义)---------------------
#if(ETHERCAT_STICK)
extern void Sync0_Isr(void);
extern void PDI_Isr(void);
extern void Sync1_Isr(void);
#endif
/***********************************************************************************
 * @fn      ESC_getTimer(void);
 * @brief   This function returns the current timer counter value from
 *          CPU Timer0. The C28x CPU Timer counter decrements from
 *          0xFFFFFFFF so a one抯 complement of the current timer counter is
 *          returned to the caller
 *
 * @param   None
 *
 * @return  complement of the current CPU TIMER0 counter value
 ***********************************************************************************/
extern uint32_t ESC_getTimer(void);


/***********************************************************************************
 * @fn          ESC_clearTimer(void);
 * @brief       This function clears the CPU Timer0 timer counter to 0x0.
 *              Customer could set it to 0xFFFFFFFF if needed
 *
 * @param       None
 *
 * @return      None
 ************************************************************************************/
extern void ESC_clearTimer(void);


/***********************************************************************************
 * @fn          ESC_timerIncPerMilliSec(void);
 * @brief       function returns timer increment per ms
 *
 * @param       None
 *
 * @return      constant value of no.of increments per ms as per CPU clock
 ************************************************************************************/
extern uint32_t ESC_timerIncPerMilliSec(void);


/*****************************************************************************************
 * @fn          ESC_readWordNonISR
 * @brief       This function returns the 16 bit data value at the address pointed
 *              by the offset parameter. This function disables the interrupts while
 *              data is read and enables the interrupts back after data is read.
 *              So the function cannot be called from inside another ISR
 *
 * @param
 *      uint16_t offset � 16 bit location address that needs to be read
 *
 * @return      16 bit data at the address location offset.
*****************************************************************************************/
extern uint16_t ESC_readWordNonISR(uint16_t offset);


/*****************************************************************************************
 * @fn          ESC_readWordISR
 * @brief       This function returns the 16 bit data value at the address pointed
 *              by the offset parameter. This function doesn抰 alter any interrupt masks.
 *              So this function can be called from within another ISR
 *
 * @param
 *      uint16_t offset � 16 bit location address that needs to be read
 *
 * @return      16 bit data at the address location offset.
*****************************************************************************************/
extern uint16_t ESC_readWordISR(uint16_t offset);


/*****************************************************************************************
 * @fn          ESC_readDWordNonISR
 * @brief       This function returns 32 bits of data at the address pointed by the Address
 *              parameter. This function disables the interrupts while the block of data is
 *              read and enables the interrupts back after the data is read. So this function
 *              cannot be called from within another ISR
 *
 * @param
 *      uint16_t Address � Address where data has to be read from the ESC
 *
 * @return      uint32_t - 32 bits data at the address location.
*****************************************************************************************/
extern uint32_t ESC_readDWordNonISR(uint16_t Address);


/*****************************************************************************************
 * @fn          ESC_readDWordISR
 * @brief       This function returns 32 bits of data at the address pointed by the Address
 *              parameter. This function doesn抰 alter any interrupts while data is read.
 *              So this function can be called from within another ISR
 *
 * @param
 *      uint16_t Address � Address where data has to be read from the ESC
 *
 * @return       uint32_t - 32 bits data at the address location.
*****************************************************************************************/
extern uint32_t ESC_readDWordISR(uint16_t Address);


/*****************************************************************************************
 * @fn          ESC_readBlockISR
 * @brief       This function returns block of data of number of bytes equal to length
 *              provided by Len parameter at the address pointed by the Address parameter.
 *              This function doesn抰 alter any interrupt masks.
 *              So this function can be called from within another ISR
 *
 * @param
 *      uint16_t *pData � Pointer to the destination array where the block of data will be copied
 *      uint16_t Address � Address where data has to be read from the ESC
 *      uint16_t Len    � Length of data block in bytes
 *
 * @return       none
*****************************************************************************************/
extern void ESC_readBlockISR(uint16_t *pData,uint16_t Address,uint16_t Len);


/*****************************************************************************************
 * @fn          ESC_readBlockNonISR
 * @brief       This function returns block of data of number of bytes equal to length
 *              provided by Len parameter at the address pointed by the Address parameter.
 *              This function disables the interrupts while the block of data is read and
 *              enables the interrupts back after the data is read. So this function
 *              cannot be called from within another ISR
 * @param
 *      uint16_t *pData � Pointer to the destination array where the block of data will be copied
 *      uint16_t Address � Address where data has to be read from the ESC
 *      uint16_t Len    � Length of data block in bytes
 *
 * @return       none
*****************************************************************************************/
extern void ESC_readBlockNonISR(uint16_t *pData,uint16_t Address,uint16_t Len);


/*****************************************************************************************
 * @fn          ESC_writeWordNonISR
 * @brief       This function writes 32 bit of data to the address pointed by the Address
 *              parameter. This function disables the interrupts while the block of data
 *              is written to and enables the interrupts back after the data is written.
 *              So this function cannot be called from within another ISR.
 *
 * @param
 *      uint16_t DWordValue � 16 bit data value that needs to be written to ESC
 *      uint16_t Address � Address where data has to be written to in the ESC
 *
 * @return       none
*****************************************************************************************/
extern void ESC_writeWordNonISR(uint16_t WordValue, uint16_t Address);


/*****************************************************************************************
 * @fn          ESC_writeWordISR
 * @brief       This function writes 16 bit of data to the address pointed by the Address
 *              parameter. This function doesn抰 alter any interrupt masks. So this
 *              function can be called from within another ISR
 *
 * @param
 *      uint16_t WordValue � 16 bit data value that needs to be written to ESC
 *      uint16_t Address � Address where data has to be written to in the ESC
 *
 * @return       none
*****************************************************************************************/
extern void ESC_writeWordISR(uint16_t WordValue, uint16_t Address);


/*****************************************************************************************
 * @fn          ESC_writeDWordNonISR
 * @brief       This function writes 32 bit of data to the address pointed by the Address
 *              parameter. This function disables the interrupts while the block of data
 *              is written to and enables the interrupts back after the data is written.
 *              So this function cannot be called from within another ISR.
 *
 * @param
 *      uint32_t DWordValue � 32 bit data value that needs to be written to ESC
 *      uint16_t Address � Address where data has to be written to in the ESC
 *
 * @return       none
*****************************************************************************************/
extern void ESC_writeDWordNonISR(uint32_t DWordValue, uint16_t Address);


/*****************************************************************************************
 * @fn          ESC_writeDWordISR
 * @brief       This function writes 32 bit of data to the address pointed by the Address
 *              parameter. This function doesn抰 alter any interrupt masks. So this
 *              function can be called from within another ISR
 *
 * @param
 *      uint32_t DWordValue � 32 bit data value that needs to be written to ESC
 *      uint16_t Address � Address where data has to be written to in the ESC
 *
 * @return       none
*****************************************************************************************/
extern void ESC_writeDWordISR(uint32_t DWordValue, uint16_t Address);


/*****************************************************************************************
 * @fn          ESC_writeBlockISR
 * @brief       This function writes block of data of number of bytes equal to length
 *              provided by Len parameter at the address pointed by the Address parameter.
 *              This function doesn抰 alter any interrupt masks. So this function can
 *              be called from within another ISR
 *
 * @param
 *      uint16_t *pData � Pointer to the source array where the block of data needs be copied
 *                          from, to the destination address in ESC
 *      uint16_t Address � Address where data has to be written to in the ESC
 *      uint16_t Len     � Length of data block in bytes
 *
 * @return       none
*****************************************************************************************/
extern void ESC_writeBlockISR(uint16_t *pData,uint16_t Address,uint16_t Len);


/*****************************************************************************************
 * @fn          ESC_writeBlockISR
 * @brief       This function writes block of data of number of bytes equal to length
 *              provided by Len parameter at the address pointed by the Address parameter.
 *              This function disables the interrupts while the block of data is written
 *              to and enables the interrupts back after the data is written.
 *              So this function cannot be called from within another ISR.
 * @param
 *      uint16_t *pData � Pointer to the source array where the block of data needs be copied
 *                          from, to the destination address in ESC
 *      uint16_t Address � Address where data has to be written to in the ESC
 *      uint16_t Len     � Length of data block in bytes
 *
 * @return       none
*****************************************************************************************/
extern void ESC_writeBlockNonISR(uint16_t *pData,uint16_t Address,uint16_t Len);


/*****************************************************************************************
* @fn           ESC_setupPDITestInterface
* @brief        This function is optional and available for applications or users to perform
*               a test of the PDI interface. The function reads the PDI control registers,
*               initializes an array of registers that needs to be read from ESC and also
*               performs read write tests on all of the RAM in ESC using the HAL API.
*
* @param    none
*
* @return       none

*****************************************************************************************/
extern void ESC_setupPDITestInterface(void);


/*****************************************************************************************
 * @fn          ESC_InitHW
 * @brief       This function is the main function that initializes the F32837xD MCU for
 *              EtherCAT examples. This function initializes system PLL, configures PIE,
 *              configures GPIO, EMIF or SPI depending on the build option chosen.
 *              This function also resets the ET1100 using the RESET GPIO and checks for
 *              EEPROM LOADED signal to go active.
 *
 * @param        none
 *
 * @return       none
*****************************************************************************************/
extern void ESC_initHW(void);

/*****************************************************************************************
 * @fn          setup_emif2_pinmux_async_16bit
 * @brief       This function initializes EMIF2 interface of the MCU for PDI functionality
 *
 * @param
 *  uint16_t cpu_sel : 1 for CPU1 and this function is only supported for CPU1 for now.
 *
 * @return       none
*****************************************************************************************/
extern void setup_emif2_pinmux_async_16bit(uint16_t cpu_sel); // EMIF driver call


/*****************************************************************************************
 * @fn          setup_emif1_pinmux_async_16bit_option2
 * @brief       This function initializes EMIF2 interface of the MCU for PDI functionality
 *
 * @param
 *  uint16_t cpu_sel : 1 for CPU1 and this function is only supported for CPU1 for now.
 *
 * @return       none
*****************************************************************************************/
extern void setup_emif1_pinmux_async_16bit_option2(uint16_t cpu_sel); // EMIF driver call


/*****************************************************************************************
 * @fn          ESC_signalPass
 * @brief       This function is internal to the HAL and can be used for debug.
 *              The function toggles a GPIO when a test or step completes successfully.
 *
 * @param       none
 *
 * @return       none
*****************************************************************************************/
extern void ESC_signalPass(void);


/*****************************************************************************************
 * @fn          ESC_signalFail
 * @brief       This function is internal to the HAL and can be used for debug.
 *              The function toggles a GPIO when a test or step completes in failure.
 *
 * @param       none
 *
 * @return       none
*****************************************************************************************/
extern void ESC_signalFail(void);


/*****************************************************************************************
 * @fn          ESC_passFailSignalSetup
 * @brief       This function is internal to the HAL and sets up the GPIO that can be
 *              toggled for a PASS/FAIL condition
 *
 * @param       none
 *
 * @return       none
*****************************************************************************************/
extern void ESC_passFailSignalSetup(void);


/*****************************************************************************************
 * @fn          ESC_ET1100EEPROMLoadedCheck
 * @brief       This function can be called to check if the EEPROM loaded signal is
 *              active after a reset before accessing PDI interface from MCU.
 *
 * @param       none
 *
 * @return       uint16_t
 *                        returns 0 if EEPROM LOADED signal is not active after 5s
 *                        returns 1 if EEPROM LOADED signal is active
*****************************************************************************************/
extern uint16_t  ESC_ET1100EEPROMLoadedCheck(void);


/*****************************************************************************************
 * @fn          ESC_resetET1100
 * @brief       This function can be called to toggle reset signal of ET1100 by the MCU.
 *              the reset signal is pulled low for 500mS before it is pulled HIGH again
 *
 * @param       none
 *
 * @return      none
*****************************************************************************************/
extern void ESC_resetET1100(void);


/*****************************************************************************************
 * @fn          ESC_HoldET1100InReset
 * @brief       This function can be called keep the reset signal to ET1100 LOW,
 *              holding the ESC in reset from MCU.
 *
 * @param       none
 *
 * @return       none
*****************************************************************************************/
extern void ESC_holdET1100InReset(void);
/*****************************************************************************************
 * @fn          ESC_ReleaseET1100reset
 * @brief       This function can be called deactivate the reset signal to ET1100
 *              being ET1100 out of reset
 *
 * @param       none
 *
 * @return      none
*****************************************************************************************/
extern void ESC_releaseET1100Reset(void);


/*****************************************************************************************
 * @fn          ESC_ConfigureSync0GPIO
 * @brief       This function can be to initialize the GPIO used for SYNC0 signal
 *
 * @param       none
 *
 * @return       none
*****************************************************************************************/
extern void ESC_configureSync0GPIO(void);


/*****************************************************************************************
 * @fn          ESC_ConfigureSync1GPIO
 * @brief       This function can be to initialize the GPIO used for SYNC1 signal
 *
 * @param       none
 *
 * @return       none
*****************************************************************************************/
extern void ESC_configureSync1GPIO(void);

/****************************************************************************************
 *
 *   PDI功能测试函数声明
 *
 ****************************************************************************************/
#ifdef PDI_HAL_TEST
/*****************************************************************************************
 * @fn           ESC_debugUpdateESCRegLogs
 * @brief        This function optional and loads the set of registers initialized by the
 *               function ESC_setupPDITestInterface() using PDI interface.
 *
 * @param       none
 *
 * @return      none
*****************************************************************************************/
extern void ESC_debugUpdateESCRegLogs(void);
/*****************************************************************************************
 * @fn          ESC_debugAddESCRegLogs
 * @brief       This function optional and adds a register to the array that can be read
 *              using the function ESC_debug_UpdateESCRegLogs()., loads the set of
 *              registers initialized by the function ESC_setupPDITestInterface()
 *              using PDI interface.
 *
 * @param
 *      uint16_t Address � address of the register that needs to be logged
 *
 * @return       none
*****************************************************************************************/
extern void ESC_debugAddESCRegsAddress(uint16_t address);


/*****************************************************************************************
 * @fn          ESC_ debugInitESCRegLogs
 * @brief       This function optional and initializes the registers read log array to default
 *              0xFFFF
 *
 * @param    none
 *
 * @return       none
*****************************************************************************************/
extern void ESC_debugInitESCRegLogs(void);

/***************************************数据结构定义*****************************************/
//-------------------------PDI_HAL_TEST功能测试ET1100寄存器数据--------------------------------
#if(HAL_ET1100)
typedef struct esc_et1100_regs
{
    uint16_t address;
    uint16_t data;
}esc_et1100_regs_t;
#endif
//-------------------------PDI_HAL_TEST功能测试ET1200寄存器数据--------------------------------
#if(HAL_ET1200)
typedef struct esc_et1200_regs
{
    uint16_t address;
    uint16_t data;
}esc_et1200_regs_t;
#endif
#endif  //PDI_HAL_TEST
/*************************************器件信息宏定义****************************************/
#if(HAL_ET1100)
        //ET1100用户RAM地址(8K)
        #define ESC_PDI_RAM_START_ADDRESS_OFFSET    0x1000
        #define ESC_PDI_RAM_END_ADDRESS_OFFSET      0x2FFF
#endif
#if(HAL_ET1200)
        //ET1200用户RAM地址(1K)
        #define ESC_PDI_RAM_START_ADDRESS_OFFSET    0x1000
        #define ESC_PDI_RAM_END_ADDRESS_OFFSET      0x13FF
#endif
/*****************************************************************************************
 * 设置硬件接口:
 * 在LaunchPad F28379D评估板上使用SPI接口
 *****************************************************************************************/
#ifdef _LAUNCHXL_F28379D  //LaunchPad F28379D评估板

    #ifdef INTERFACE_SPI  //使用SPI接口

#ifdef USE_EMIF1
    #error "user cannot use EMIF1 with SPI Configurations of project"
#endif
    extern void ESC_initSPI(void);
    extern void ESC_readSPI(uint16_t offset_addr,uint16_t numbytes, uint16_t* buffer);
    extern void ESC_writeSPI(uint16_t offset_addr,uint16_t *wrdata, uint16_t numwords);
//------------------------------使用LaunchPad F28379D SPIA接口-------------------------------
#ifdef USE_SPIA
    #define ESC_SPI_CS_GPIO                     61
//------------------------------使用LaunchPad F28379D SPIB接口-------------------------------
#elif  USE_SPIB
    #define ESC_SPI_CS_GPIO                     61
#endif
//------------------------------------SPI中断信号引脚编号--------------------------------------
    #define ESC_SPI_INT_GPIO                    22
//------------------------------------SYNC0中断信号引脚编号------------------------------------
    #define ESC_SYNC0_GPIO                      19
//------------------------------------SYNC1中断信号引脚编号------------------------------------
    #define ESC_SYNC1_GPIO                      0
//---------------------------------------器件复位引脚编号--------------------------------------
    #define ESC_DEVICE_RESET_GPIO               0
//------------------------------------EEPROM加载信号引脚编号-----------------------------------
    #define ESC_EEPROM_LOADED_GPIO              0
//---------------------------------------引脚输入控制宏定义------------------------------------
    #define ESC_SPI_CS_1()              GPIO_writePin(ESC_SPI_CS_GPIO, 1)
    #define ESC_SPI_CS_0()              GPIO_writePin(ESC_SPI_CS_GPIO, 0)
    #define ESC_DEVICE_RESET_1()        GPIO_writePin(ESC_SPI_CS_GPIO, 1)
    #define ESC_DEVICE_RESET_0()        GPIO_writePin(ESC_SPI_CS_GPIO, 0)

    #else //#ifdef INTERFACE_SPI

        extern void setup_emif1_pinmux_async_16bit(Uint16 cpu_sel);
        extern void ESC_EMIF2SetupPinmuxAsync16Bit(Uint16 cpu_sel);
        extern void ESC_writeBlockEMIF2(uint16_t* pData, uint16_t offset_addr,uint16_t numwords);

        #ifdef USE_EMIF1 //LaunchPAD XL EMIF1 J9 connector option
            #define ESC_EMIF_INT_GPIO                   107
            #define ESC_SYNC0_GPIO                      86
            #define ESC_SYNC1_GPIO                      87
            #define ESC_RESET_ET1100_GPIO               108
            #define ESC_EEPROM_LOADED_GPIO              52

        #else // #ifdef ESC_USE_EMIF1   //USE EMIF2
            #define ESC_EMIF_INT_GPIO   136
            //with EMIF2 we can access 4KW = 8KB = 0x1000 address locations

            //0x0 to 0x1000 of ET1100 will be 0x2000[0] to 0x2000[0x800] of EMIF2 address sapce
            //0x1000 to 0x2000 of ET1100 will be 0x2000[0x800] to 0x2000[0x1000] of EMIF2 address space
            #define ESC_SYNC0_GPIO              113
            #define ESC_SYNC1_GPIO              114
            #define ESC_RESET_ET1100_GPIO       137
            #define ESC_EEPROM_LOADED_GPIO      119

        #endif //// #ifdef USE_EMIF1

#endif //#ifdef INTERFACE_SPI

#else //#ifdef _LAUNCHXL_F28379D
      //用户自定义驱动板硬件
      #define ESC_SPI_CS_GPIO                     61


#endif

#define ESC_ETHERCAT_READ           0b010  //读数据
#define ESC_ETHERCAT_READ_WAIT      0b011  //读数据
#define ESC_ETHERCAT_WRITE          0b100
#define ESC_ETHERCAT_3BYTEADDR      0b110
#define ESC_ETHERCAT_WAIT           0xFF
#define ESC_ETHERCAT_CONTINUE       0x00   //帧格式读数据等待数据字节
#define ESC_ETHERCAT_RDTERMINATE    0xFF   //帧格式结尾字节

#define ESC_DEBUG_REGS_LENGTH       20

#ifdef __cplusplus
}
#endif /* extern "C" */


#endif // _ETHERCAT_SLAVE_C28x_HAL_H_
