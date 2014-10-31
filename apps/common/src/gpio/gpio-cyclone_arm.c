/**
********************************************************************************
\file   gpio-arm.c

\brief  GPIOs for Xilinx Zynq ARM

The file implements the GPIOs on Xilinx Zynq ARM core used by openPOWERLINK demo
applications.

\ingroup module_app_common
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2014, Kalycito Infotech Private Limited
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the copyright holders nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
------------------------------------------------------------------------------*/

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <alt_generalpurpose_io.h>
#include <alt_globaltmr.h>
#include <alt_clock_manager.h>

#include <oplk/oplk.h>

#include "gpio.h"

#include <system.h>

//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// module global vars
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// global function prototypes
//------------------------------------------------------------------------------

//============================================================================//
//            P R I V A T E   D E F I N I T I O N S                           //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#define GPIO_STATUS_LED_BIT                         1
#define GPIO_ERROR_LED_BIT                          2

#define LED_OUTPUT_DELAY_US                         500000      // 500ms

#define HPS_LED_ALL_BIT_MASK                        0x0000F000
#define HPS_LED_ALL_TURN_ON                         0x00000000
#define HPS_LED_ALL_TURN_OFF                        0x0000F000
#define HPS_LED_0_TURN_ON                           0x00007000  // GPIO[44] (HPS_LED_0) --> Error Led
#define HPS_LED_1_TURN_ON                           0x0000B000  // GPIO[43] (HPS_LED_1) --> Status Led
#define HPS_LED_2_TURN_ON                           0x0000D000  // GPIO[42] (HPS_LED_2)
#define HPS_LED_3_TURN_ON                           0x0000E000  // GPIO[41] (HPS_LED_3)

#define HPS_LED_0_TURN_OFF                          0x00008000
#define HPS_LED_1_TURN_OFF                          0x00004000
#define HPS_LED_2_TURN_OFF                          0x00002000
#define HPS_LED_3_TURN_OFF                          0x00001000

#define FPGA_LED_ALL_BIT_MASK                       0x0000000F
#define FPGA_LED_ALL_TURN_ON                        0x00000000
#define FPGA_LED_ALL_TURN_OFF                       0x0000000F
#define FPGA_LED_0_TURN_ON                          0x0000000E
#define FPGA_LED_1_TURN_ON                          0x0000000D
#define FPGA_LED_2_TURN_ON                          0x0000000B
#define FPGA_LED_3_TURN_ON                          0x00000007

#define FPGA_LED_0_TURN_OFF                         0x00000001
#define FPGA_LED_1_TURN_OFF                         0x00000002
#define FPGA_LED_2_TURN_OFF                         0x00000004
#define FPGA_LED_3_TURN_OFF                         0x00000008

#define HPS_PB_INT_ALL_BIT_MASK                     0x01E00000  // Interrupt bits for GPIO2

#define HPS_PB_0_ASSERT                             0x01C00000  // GPIO[8] (HPS_PB_0)
#define HPS_PB_1_ASSERT                             0x01A00000  // GPIO[9] (HPS_PB_1)
#define HPS_PB_2_ASSERT                             0x01600000  // GPIO[10](HPS_PB_2)
#define HPS_PB_3_ASSERT                             0x00E00000  // GPIO[11](HPS_PB_3)

#define HPS_PB_ALL_BIT_MASK                         0x01E00000

#define HPS_PB_0_BIT_MASK                           0x01000000  // GPIO[8] (HPS_PB_0)
#define HPS_PB_1_BIT_MASK                           0x00800000  // GPIO[9] (HPS_PB_1)
#define HPS_PB_2_BIT_MASK                           0x00400000  // GPIO[10](HPS_PB_2)
#define HPS_PB_3_BIT_MASK                           0x00200000  // GPIO[11](HPS_PB_3)

#define HPS_DIPSW_ALL_BIT_MASK                      0x001E0000
#define HPS_DIPSW_NET_VAL(portVal)    ((portVal >> 17) & 0xF)

#define HPS_DIPSW_0_BIT_MASK                        0x00100000  // GPIO[4] (HPS_DIPSW_0)
#define HPS_DIPSW_1_BIT_MASK                        0x00080000  // GPIO[5] (HPS_DIPSW_1)
#define HPS_DIPSW_2_BIT_MASK                        0x00040000  // GPIO[6](HPS_DIPSW_2)
#define HPS_DIPSW_3_BIT_MASK                        0x00020000  // GPIO[7](HPS_DIPSW_3)

#define FPGA_PB_ALL_BIT_MASK                        0x00000003

#define FPGA_PB_0_BIT_MASK                          0x00000001
#define FPGA_PB_1_BIT_MASK                          0x00000002

#define FPGA_DIPSW_ALL_BIT_MASK                     0x0000000F

#define FPGA_DIPSW_0_BIT_MASK                       0x00000001
#define FPGA_DIPSW_1_BIT_MASK                       0x00000002
#define FPGA_DIPSW_3_BIT_MASK                       0x00000001
#define FPGA_DIPSW_4_BIT_MASK                       0x00000002

#define FPGA_DIPSW_NET_VAL(portVal)     (portVal & FPGA_DIPSW_ALL_BIT_MASK)

// Determine size of an array
#define ARRAY_COUNT(array)              (sizeof(array) / sizeof(array[0]))

#ifdef XPAR_NODE_SWITCHES_BASEADDR
#define NODE_SWITCH_BASE                            XPAR_NODE_SWITCHES_BASEADDR
#endif  // XPAR_NODE_SWITCHES_BASEADDR

#ifdef XPAR_POWERLINK_LED_BASEADDR
#define STATUS_LEDS_BASE                            XPAR_POWERLINK_LED_BASEADDR
#endif  // XPAR_POWERLINK_LED_BASEADDR

#ifdef XPAR_GPIO_INPUTS_BASEADDR
#define GPIO_INPUTS_BASE                            XPAR_GPIO_INPUTS_BASEADDR
#endif  // XPAR_GPIO_INPUTS_BASEADDR

#ifdef XPAR_GPIO_OUTPUTS_BASEADDR
#define GPIO_OUTPUTS_BASE                           XPAR_GPIO_OUTPUTS_BASEADDR
#endif  // XPAR_GPIO_OUTPUTS_BASEADDR

#define FPGA_BUS_WIDTH                              32
#define __IO_CALC_ADDRESS_NATIVE(base, offset) \
    (base + offset * (FPGA_BUS_WIDTH / 8))
#define IORD16(base, offset)            alt_read_word(base + offset * (FPGA_BUS_WIDTH / 8))
#define IORD32(base, offset)            alt_read_word(base + offset * (FPGA_BUS_WIDTH / 8))
#define IOWR16(base, offset, val)       alt_write_word(base + offset * (FPGA_BUS_WIDTH / 8), val)
#define IOWR32(base, offset, val)       alt_write_word(base + offset * (FPGA_BUS_WIDTH / 8), val)

/* STATUS register */
#define ALTERA_AVALON_TIMER_STATUS_REG              0
#define IOADDR_ALTERA_AVALON_TIMER_STATUS(base) \
    __IO_CALC_ADDRESS_NATIVE(base, ALTERA_AVALON_TIMER_STATUS_REG)
#define IORD_ALTERA_AVALON_TIMER_STATUS(base) \
    IORD16(base, ALTERA_AVALON_TIMER_STATUS_REG)
#define IOWR_ALTERA_AVALON_TIMER_STATUS(base, data) \
    IOWR16(base, ALTERA_AVALON_TIMER_STATUS_REG, data)
#define ALTERA_AVALON_TIMER_STATUS_TO_MSK           (0x1)
#define ALTERA_AVALON_TIMER_STATUS_TO_OFST          (0)
#define ALTERA_AVALON_TIMER_STATUS_RUN_MSK          (0x2)
#define ALTERA_AVALON_TIMER_STATUS_RUN_OFST         (1)

/* CONTROL register */
#define ALTERA_AVALON_TIMER_CONTROL_REG             1
#define IOADDR_ALTERA_AVALON_TIMER_CONTROL(base) \
    __IO_CALC_ADDRESS_NATIVE(base, ALTERA_AVALON_TIMER_CONTROL_REG)
#define IORD_ALTERA_AVALON_TIMER_CONTROL(base) \
    IORD16(base, ALTERA_AVALON_TIMER_CONTROL_REG)
#define IOWR_ALTERA_AVALON_TIMER_CONTROL(base, data) \
    IOWR16(base, ALTERA_AVALON_TIMER_CONTROL_REG, data)
#define ALTERA_AVALON_TIMER_CONTROL_ITO_MSK         (0x1)
#define ALTERA_AVALON_TIMER_CONTROL_ITO_OFST        (0)
#define ALTERA_AVALON_TIMER_CONTROL_CONT_MSK        (0x2)
#define ALTERA_AVALON_TIMER_CONTROL_CONT_OFST       (1)
#define ALTERA_AVALON_TIMER_CONTROL_START_MSK       (0x4)
#define ALTERA_AVALON_TIMER_CONTROL_START_OFST      (2)
#define ALTERA_AVALON_TIMER_CONTROL_STOP_MSK        (0x8)
#define ALTERA_AVALON_TIMER_CONTROL_STOP_OFST       (3)

/* Period and SnapShot Register for COUNTER_SIZE = 32 */
/*----------------------------------------------------*/
/* PERIODL register */
#define ALTERA_AVALON_TIMER_PERIODL_REG             2
#define IOADDR_ALTERA_AVALON_TIMER_PERIODL(base) \
    __IO_CALC_ADDRESS_NATIVE(base, ALTERA_AVALON_TIMER_PERIODL_REG)
#define IORD_ALTERA_AVALON_TIMER_PERIODL(base) \
    IORD16(base, ALTERA_AVALON_TIMER_PERIODL_REG)
#define IOWR_ALTERA_AVALON_TIMER_PERIODL(base, data) \
    IOWR16(base, ALTERA_AVALON_TIMER_PERIODL_REG, data)
#define ALTERA_AVALON_TIMER_PERIODL_MSK             (0xFFFF)
#define ALTERA_AVALON_TIMER_PERIODL_OFST            (0)

/* PERIODH register */
#define ALTERA_AVALON_TIMER_PERIODH_REG             3
#define IOADDR_ALTERA_AVALON_TIMER_PERIODH(base) \
    __IO_CALC_ADDRESS_NATIVE(base, ALTERA_AVALON_TIMER_PERIODH_REG)
#define IORD_ALTERA_AVALON_TIMER_PERIODH(base) \
    IORD16(base, ALTERA_AVALON_TIMER_PERIODH_REG)
#define IOWR_ALTERA_AVALON_TIMER_PERIODH(base, data) \
    IOWR16(base, ALTERA_AVALON_TIMER_PERIODH_REG, data)
#define ALTERA_AVALON_TIMER_PERIODH_MSK             (0xFFFF)
#define ALTERA_AVALON_TIMER_PERIODH_OFST            (0)

/* SNAPL register */
#define ALTERA_AVALON_TIMER_SNAPL_REG               4
#define IOADDR_ALTERA_AVALON_TIMER_SNAPL(base) \
    __IO_CALC_ADDRESS_NATIVE(base, ALTERA_AVALON_TIMER_SNAPL_REG)
#define IORD_ALTERA_AVALON_TIMER_SNAPL(base) \
    IORD16(base, ALTERA_AVALON_TIMER_SNAPL_REG)
#define IOWR_ALTERA_AVALON_TIMER_SNAPL(base, data) \
    IOWR16(base, ALTERA_AVALON_TIMER_SNAPL_REG, data)
#define ALTERA_AVALON_TIMER_SNAPL_MSK               (0xFFFF)
#define ALTERA_AVALON_TIMER_SNAPL_OFST              (0)

/* SNAPH register */
#define ALTERA_AVALON_TIMER_SNAPH_REG               5
#define IOADDR_ALTERA_AVALON_TIMER_SNAPH(base) \
    __IO_CALC_ADDRESS_NATIVE(base, ALTERA_AVALON_TIMER_SNAPH_REG)
#define IORD_ALTERA_AVALON_TIMER_SNAPH(base) \
    IORD16(base, ALTERA_AVALON_TIMER_SNAPH_REG)
#define IOWR_ALTERA_AVALON_TIMER_SNAPH(base, data) \
    IOWR16(base, ALTERA_AVALON_TIMER_SNAPH_REG, data)
#define ALTERA_AVALON_TIMER_SNAPH_MSK               (0xFFFF)
#define ALTERA_AVALON_TIMER_SNAPH_OFST              (0)

#define IOADDR_ALTERA_AVALON_PIO_DATA(base)                 __IO_CALC_ADDRESS_NATIVE(base, 0)
#define IORD_ALTERA_AVALON_PIO_DATA(base)                   IORD32(base, 0)
#define IOWR_ALTERA_AVALON_PIO_DATA(base, data)             IOWR32(base, 0, data)

#define IOADDR_ALTERA_AVALON_PIO_DIRECTION(base)            __IO_CALC_ADDRESS_NATIVE(base, 1)
#define IORD_ALTERA_AVALON_PIO_DIRECTION(base)              IORD32(base, 1)
#define IOWR_ALTERA_AVALON_PIO_DIRECTION(base, data)        IOWR32(base, 1, data)

#define IOADDR_ALTERA_AVALON_PIO_IRQ_MASK(base)             __IO_CALC_ADDRESS_NATIVE(base, 2)
#define IORD_ALTERA_AVALON_PIO_IRQ_MASK(base)               IORD32(base, 2)
#define IOWR_ALTERA_AVALON_PIO_IRQ_MASK(base, data)         IOWR32(base, 2, data)

#define IOADDR_ALTERA_AVALON_PIO_EDGE_CAP(base)             __IO_CALC_ADDRESS_NATIVE(base, 3)
#define IORD_ALTERA_AVALON_PIO_EDGE_CAP(base)               IORD32(base, 3)
#define IOWR_ALTERA_AVALON_PIO_EDGE_CAP(base, data)         IOWR32(base, 3, data)

#define IOADDR_ALTERA_AVALON_PIO_SET_BIT(base)              __IO_CALC_ADDRESS_NATIVE(base, 4)
#define IORD_ALTERA_AVALON_PIO_SET_BITS(base)               IORD32(base, 4)
#define IOWR_ALTERA_AVALON_PIO_SET_BITS(base, data)         IOWR32(base, 4, data)

#define IOADDR_ALTERA_AVALON_PIO_CLEAR_BITS(base)           __IO_CALC_ADDRESS_NATIVE(base, 5)
#define IORD_ALTERA_AVALON_PIO_CLEAR_BITS(base)             IORD32(base, 5)
#define IOWR_ALTERA_AVALON_PIO_CLEAR_BITS(base, data)       IOWR32(base, 5, data)

//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------

static UINT32               plkStatusLeds_l = 0;                                ///< Local copy of the state of the POWERLINK status LEDs

ALT_GPIO_CONFIG_RECORD_t    pb_gpio_init[] =
{
    { ALT_HLGPI_4, ALT_GPIO_PIN_INPUT, 0, 0, 0, 0 },                            // HPS_DIPSW_0
    { ALT_HLGPI_5, ALT_GPIO_PIN_INPUT, 0, 0, 0, 0 },                            // HPS_DIPSW_1
    { ALT_HLGPI_6, ALT_GPIO_PIN_INPUT, 0, 0, 0, 0 },                            // HPS_DIPSW_2
    { ALT_HLGPI_7, ALT_GPIO_PIN_INPUT, 0, 0, 0, 0 },                            // HPS_DIPSW_3
    { ALT_HLGPI_8, ALT_GPIO_PIN_INPUT, 0, 0, 0, 0 },                            // HPS_PB_0
    { ALT_HLGPI_9, ALT_GPIO_PIN_INPUT, 0, 0, 0, 0 },                            // HPS_PB_1
    { ALT_HLGPI_10, ALT_GPIO_PIN_INPUT, 0, 0, 0, 0 }, // HPS_PB_2
    { ALT_HLGPI_11, ALT_GPIO_PIN_INPUT, 0, 0, 0, 0 } // HPS_PB_3
};

ALT_GPIO_CONFIG_RECORD_t    led_gpio_init[] =
{
    { ALT_GPIO_1BIT_44, ALT_GPIO_PIN_OUTPUT, 0, 0, 0, ALT_GPIO_PIN_DATAZERO },  // HPS_LED_0
    { ALT_GPIO_1BIT_43, ALT_GPIO_PIN_OUTPUT, 0, 0, 0, ALT_GPIO_PIN_DATAZERO },  // HPS_LED_1
    { ALT_GPIO_1BIT_42, ALT_GPIO_PIN_OUTPUT, 0, 0, 0, ALT_GPIO_PIN_DATAZERO },  // HPS_LED_2
    { ALT_GPIO_1BIT_41, ALT_GPIO_PIN_OUTPUT, 0, 0, 0, ALT_GPIO_PIN_DATAZERO }  // HPS_LED_3
};

uint32_t                    push_button_stat = 0;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Initialize GPIO module

The function initializes the GPIO module before being used.

\return Returns 0 if successful

\ingroup module_app_common
*/
//------------------------------------------------------------------------------
int gpio_init(void)
{
    ALT_STATUS_CODE    halRet = ALT_E_SUCCESS;

    /* Initialize HPS GPIO */

    // Initialize GPIO module
    if (halRet == ALT_E_SUCCESS)
    {
        halRet = alt_gpio_init();
    }

    printf("GPIO-INFO: Set up GPIO for LEDs.\n");

    // Setup GPIO LED
    if (halRet == ALT_E_SUCCESS)
    {
        halRet = alt_gpio_group_config(led_gpio_init, ARRAY_COUNT(led_gpio_init));
    }

    // clear the Leds
    if (halRet == ALT_E_SUCCESS)
    {
        halRet = alt_gpio_port_data_write(ALT_GPIO_PORTB, HPS_LED_ALL_BIT_MASK, HPS_LED_ALL_TURN_OFF);
    }

    printf("GPIO-INFO: Set up GPIO for Dip Switches.\n");

    // Setup GPIO PUSHBUTTON
    if (halRet == ALT_E_SUCCESS)
    {
        halRet = alt_gpio_group_config(pb_gpio_init, ARRAY_COUNT(pb_gpio_init));
    }

    // Enable GPIO interrupts
    if (halRet == ALT_E_SUCCESS)
    {
        halRet = alt_gpio_port_int_disable(ALT_GPIO_PORTC, HPS_PB_INT_ALL_BIT_MASK);
    }

    /* Initialize FPGA GPIO */

    // will be initialized in target intialization

    // clear the Leds
    IOWR_ALTERA_AVALON_PIO_DATA(LED_PIO_FLOW_CTRL_BASE, FPGA_LED_ALL_TURN_OFF);

    // Clear the dip switch and push button interrupt status registers

    IOWR_ALTERA_AVALON_PIO_IRQ_MASK(BUTTON_PIO_BASE, 0x0);
    IOWR_ALTERA_AVALON_PIO_EDGE_CAP(BUTTON_PIO_BASE, FPGA_PB_ALL_BIT_MASK);

    IOWR_ALTERA_AVALON_PIO_IRQ_MASK(DIPSW_PIO_BASE, 0x0);
    IOWR_ALTERA_AVALON_PIO_EDGE_CAP(DIPSW_PIO_BASE, FPGA_DIPSW_ALL_BIT_MASK);

    return 0;
}

//------------------------------------------------------------------------------
/**
\brief  Shutdown GPIO module

The function shuts down the GPIO module.

\ingroup module_app_common
*/
//------------------------------------------------------------------------------
void gpio_shutdown(void)
{
    /* Uninitialize HPS GPIO */
    alt_gpio_uninit();

    /* Uninitialize FPGA GPIO */

    // will be handled by target module

    IOWR_ALTERA_AVALON_PIO_IRQ_MASK(BUTTON_PIO_BASE, 0x0);
    IOWR_ALTERA_AVALON_PIO_EDGE_CAP(BUTTON_PIO_BASE, FPGA_PB_ALL_BIT_MASK);

    IOWR_ALTERA_AVALON_PIO_IRQ_MASK(DIPSW_PIO_BASE, 0x0);
    IOWR_ALTERA_AVALON_PIO_EDGE_CAP(DIPSW_PIO_BASE, FPGA_DIPSW_ALL_BIT_MASK);

    // clear the Leds
    IOWR_ALTERA_AVALON_PIO_DATA(LED_PIO_FLOW_CTRL_BASE, FPGA_LED_ALL_TURN_OFF);
}

//------------------------------------------------------------------------------
/**
\brief  Gets the node switch value

The function returns the node ID set by the node switches.

\return Returns the set node ID

\ingroup module_app_common
*/
//------------------------------------------------------------------------------
UINT8 gpio_getNodeid(void)
{
    UINT8       nodeId;
    UINT32      hpsSwStatus = 0x0;
    UINT32      fpgaSwStatus = 0x0;

    hpsSwStatus = alt_gpio_port_data_read(ALT_GPIO_PORTC, HPS_DIPSW_ALL_BIT_MASK);
    fpgaSwStatus = IORD_ALTERA_AVALON_PIO_DATA(DIPSW_PIO_BASE);

    nodeId = (UINT8) ((FPGA_DIPSW_NET_VAL(fpgaSwStatus) << DIPSW_PIO_DATA_WIDTH) | HPS_DIPSW_NET_VAL(hpsSwStatus));

    return nodeId;
}

//------------------------------------------------------------------------------
/**
\brief  Sets the status LED

The function sets the POWERLINK status LED.

\param  fOn_p               Determines the LED state

\ingroup module_app_common
*/
//------------------------------------------------------------------------------
void gpio_setStatusLed(BOOL fOn_p)
{
    ALT_STATUS_CODE     halRet = ALT_E_SUCCESS;
    UINT32              ledStatus = 0x0;

    ledStatus = alt_gpio_port_data_read(ALT_GPIO_PORTB, HPS_LED_ALL_BIT_MASK);

    if (fOn_p)
    {
        ledStatus &= (UINT32) (~HPS_LED_1_TURN_OFF & HPS_LED_ALL_BIT_MASK);
        halRet = alt_gpio_port_data_write(ALT_GPIO_PORTB, HPS_LED_ALL_BIT_MASK, ledStatus);
    }
    else
    {
        ledStatus |= HPS_LED_1_TURN_OFF;
        halRet = alt_gpio_port_data_write(ALT_GPIO_PORTB, HPS_LED_ALL_BIT_MASK, ledStatus);
    }
}

//------------------------------------------------------------------------------
/**
\brief  Sets the error LED

The function sets the POWERLINK error LED.

\param  fOn_p               Determines the LED state

\ingroup module_app_common
*/
//------------------------------------------------------------------------------
void gpio_setErrorLed(BOOL fOn_p)
{
    ALT_STATUS_CODE     halRet = ALT_E_SUCCESS;
    UINT32              ledStatus = 0x0;

    ledStatus = alt_gpio_port_data_read(ALT_GPIO_PORTB, HPS_LED_ALL_BIT_MASK);

    if (fOn_p)
    {
        ledStatus &= (UINT32) (~HPS_LED_0_TURN_OFF & HPS_LED_ALL_BIT_MASK);
        halRet = alt_gpio_port_data_write(ALT_GPIO_PORTB, HPS_LED_ALL_BIT_MASK, ledStatus);
    }
    else
    {
        ledStatus |= HPS_LED_0_TURN_OFF;
        halRet = alt_gpio_port_data_write(ALT_GPIO_PORTB, HPS_LED_ALL_BIT_MASK, ledStatus);
    }
}

//------------------------------------------------------------------------------
/**
\brief  Gets the application input

The function returns application inputs.

\return Returns the application inputs.

\ingroup module_app_common
*/
//------------------------------------------------------------------------------
UINT8 gpio_getAppInput(void)
{
    UINT8    key;

#ifdef DIPSW_PIO_BASE
    key = (UINT8) IORD_ALTERA_AVALON_PIO_EDGE_CAP(BUTTON_PIO_BASE);
    IOWR_ALTERA_AVALON_PIO_EDGE_CAP(BUTTON_PIO_BASE, FPGA_PB_ALL_BIT_MASK);
#else
    key = 0;
#endif

    return key;
}

//------------------------------------------------------------------------------
/**
\brief  Sets the application output

The function sets the application outputs.

\param  val_p               Determines the value to be set to the output

\ingroup module_app_common
*/
//------------------------------------------------------------------------------
void gpio_setAppOutputs(UINT32 val_p)
{
#ifdef LED_PIO_FLOW_CTRL_BASE
    IOWR_ALTERA_AVALON_PIO_DATA(LED_PIO_FLOW_CTRL_BASE, ~val_p);
#endif
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

///\}
