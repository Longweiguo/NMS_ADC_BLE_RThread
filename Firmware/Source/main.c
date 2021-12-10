#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include "apollo3p.h"
#include <core_cm4.h>
#include "main.h"
#include "ad7147.h"
#include "am_bsp_pins.h"
#include "am_util_stdio.h"
#include "am_mcu_apollo.h"
#include "heyos_hal_adc.h"
#include "heyos_hal_spi.h"

void *phUART;
volatile uint32_t ui32LastError;
uint8_t g_ui8TxBuffer[256];
uint8_t g_ui8RxBuffer[20];

const am_hal_uart_config_t g_sUartConfig =
{
    // Standard UART settings: 921600-8-N-1
    .ui32BaudRate = 921600,
    .ui32DataBits = AM_HAL_UART_DATA_BITS_8,
    .ui32Parity = AM_HAL_UART_PARITY_NONE,
    .ui32StopBits = AM_HAL_UART_ONE_STOP_BIT,
    .ui32FlowControl = AM_HAL_UART_FLOW_CTRL_NONE,

    // Set TX and RX FIFOs to interrupt at half-full.
    .ui32FifoLevels = (AM_HAL_UART_TX_FIFO_1_2 |
                                         AM_HAL_UART_RX_FIFO_1_2),

    // Buffers
    .pui8TxBuffer = g_ui8TxBuffer,
    .ui32TxBufferSize = sizeof(g_ui8TxBuffer),
    .pui8RxBuffer = g_ui8RxBuffer,
    .ui32RxBufferSize = sizeof(g_ui8RxBuffer),
};


//Catch HAL errors.
void inline error_handler(uint32_t ui32ErrorStatus)
{
    if ((ui32ErrorStatus) != AM_HAL_STATUS_SUCCESS)
    {
        ui32LastError = ui32ErrorStatus;
        while (1);
    }
}

void uart_print(char *pcStr)
{
    uint32_t ui32StrLen = 0;
    uint32_t ui32BytesWritten = 0;

    // Measure the length of the string.
    while (pcStr[ui32StrLen] != 0)
    {
        ui32StrLen++;
    }

    // Print the string via the UART.
    const am_hal_uart_transfer_t sUartWrite =
    {
        .ui32Direction = AM_HAL_UART_WRITE,
        .pui8Data = (uint8_t *) pcStr,
        .ui32NumBytes = ui32StrLen,
        .ui32TimeoutMs = 0,
        .pui32BytesTransferred = &ui32BytesWritten,
    };

    CHECK_ERRORS(am_hal_uart_transfer(phUART, &sUartWrite));

    if (ui32BytesWritten != ui32StrLen)
    {
        // Couldn't send the whole string!!
        while(1);
    }
}

void am_uart_isr(void)
{
    // Service the FIFOs as necessary, and clear the interrupts.
    uint32_t ui32Status, ui32Idle;
    am_hal_uart_interrupt_status_get(phUART, &ui32Status, true);
    am_hal_uart_interrupt_clear(phUART, ui32Status);
    am_hal_uart_interrupt_service(phUART, ui32Status, &ui32Idle);
}

static void UartInit(void)
{
    // Initialize the printf interface for UART output.
    CHECK_ERRORS(am_hal_uart_initialize(UART_Module, &phUART));
    CHECK_ERRORS(am_hal_uart_power_control(phUART, AM_HAL_SYSCTRL_WAKE, false));
    CHECK_ERRORS(am_hal_uart_configure(phUART, &g_sUartConfig));

    // Enable the UART pins.
    am_hal_gpio_pinconfig(GPIO_UART0_TX, g_AM_BSP_GPIO_COM_UART_TX);

    // Enable interrupts.
    NVIC_EnableIRQ((IRQn_Type)(UART0_IRQn));
    am_hal_interrupt_master_enable();

    // Set the main print interface to use the UART print function we defined.
    am_util_stdio_printf_init(uart_print);
	am_util_stdio_printf("UartInit OK\r\n");
}

void Uartdeinit(void)
{
    NVIC_DisableIRQ((IRQn_Type)(UART0_IRQn));
    am_hal_uart_interrupt_disable(phUART, AM_HAL_UART_INT_RX | AM_HAL_UART_INT_RX_TMOUT);
    am_hal_uart_power_control(phUART, AM_HAL_SYSCTRL_DEEPSLEEP, false);
    am_hal_uart_deinitialize(phUART);

    am_hal_gpio_pinconfig(AM_BSP_GPIO_COM_UART_TX, g_AM_HAL_GPIO_DISABLE);
    am_hal_gpio_pinconfig(AM_BSP_GPIO_COM_UART_RX, g_AM_HAL_GPIO_DISABLE);
}

static void BoardsInit(void)
{
    // Set the clock frequency.
    am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_SYSCLK_MAX, 0);

    // Set the default cache configuration
    am_hal_cachectrl_config(&am_hal_cachectrl_defaults);
    am_hal_cachectrl_enable();
    // Configure the board for low power operation.

    // Initialize for low power in the power control block
    am_hal_pwrctrl_low_power_init();

    // Disable the RTC.
    am_hal_rtc_osc_disable();

    am_hal_pwrctrl_memory_enable(AM_HAL_PWRCTRL_MEM_SRAM_768K);
    am_hal_pwrctrl_memory_enable(AM_HAL_PWRCTRL_MEM_FLASH_MAX);

    UartInit();
	heyos_hal_adc_init(ADC_INDEX_BAT);
}

void am_hal_read_flash(uint32_t addr, uint32_t* buf, int len)
{
    uint32_t ui32Critical;

    ui32Critical = am_hal_interrupt_master_disable();
    for(int i=0;i<len;i++)
    {
        buf[i] = am_hal_flash_load_ui32((uint32_t *)(addr+4*i));
    }
    am_hal_interrupt_master_set(ui32Critical);
}

void am_hal_erase_flash(uint32_t ui32Addr, int datasize)
{
    uint32_t ui32Critical;
    uint32_t ui32CurrentPage, ui32CurrentBlock;
    while(datasize)
    {
		ui32CurrentPage =  AM_HAL_FLASH_ADDR2PAGE(ui32Addr);
		ui32CurrentBlock = AM_HAL_FLASH_ADDR2INST(ui32Addr);
		//
		// Start a critical section.
		//
		ui32Critical = am_hal_interrupt_master_disable();
		am_hal_flash_page_erase(AM_HAL_FLASH_PROGRAM_KEY,
									ui32CurrentBlock, ui32CurrentPage);
		//
		// Exit the critical section.
		//
		am_hal_interrupt_master_set(ui32Critical);
        if(datasize>FIRMWARE_NAND_CELL)
        {
            datasize -= FIRMWARE_NAND_CELL;
        }
        else
        {
            break;
        }
        ui32Addr +=FIRMWARE_NAND_CELL;
    }
}

void am_hal_write_flash(uint32_t addr, uint32_t* buf, int len)
{
	uint32_t ui32Critical;
    while(len)
    {
		ui32Critical = am_hal_interrupt_master_disable();
		//
		// Program the flash page with the data we just received.
		//
		am_hal_flash_program_main(AM_HAL_FLASH_PROGRAM_KEY, buf,(uint32_t *)addr, len);
		//
		// Exit the critical section.
		//
		am_hal_interrupt_master_set(ui32Critical);
        if(len>FIRMWARE_NAND_CELL)
        {
            len -= FIRMWARE_NAND_CELL;
            buf +=FIRMWARE_NAND_CELL;
            addr +=FIRMWARE_NAND_CELL;
        }
        else
        {
            break;
        }
    }
}

struct ad714x_touchpad_plat touchpad_config = 
{
    .x_start_stage = 0,
    .x_end_stage = 11,
    .y_start_stage = 0,
    .y_end_stage = 0,
};

struct ad714x_platform_data hw_config = 
{
    .touchpad = &touchpad_config,
	.stage_cfg_reg = 
	{
		{0xFFFE,0x1FFF,0x0000,0x2626,1650,1650,1650,1650},          //CIN0
		{0xFFFB,0x1FFF,0x0000,0x2626,1650,1650,1650,1650},          //CIN1
		{0xFFEF,0x1FFF,0x0000,0x2626,1650,1650,1650,1650},
		{0xFFBF,0x1FFF,0x0000,0x2626,1650,1650,1650,1650},
		{0xFEFF,0x1FFF,0x0000,0x2626,1650,1650,1650,1650},
		{0xFBFF,0x1FFF,0x0000,0x2626,1650,1650,1650,1650},
		{0xEFFF,0x1FFF,0x0000,0x2626,1650,1650,1650,1650},

		{0xFFFF,0x1FFE,0x0000,0x2626,1650,1650,1650,1650},         //CIN7
		{0xFFFF,0x1FFB,0x0000,0x2626,1650,1650,1650,1650},
        {0xFFFF,0x1FEF,0x0000,0x2626,1650,1650,1650,1650},
		{0xFFFF,0x1FBF,0x0000,0x2626,1650,1650,1650,1650},
		{0xFFFF,0x1EFF,0x0000,0x2626,1650,1650,1650,1650},
	},
//===================PWR_CONTROL Register = 0x00=======================
//full power mode (normal operation, CDC conversions approximately every 36 ms)
//Number of stages in sequence (N + 1) = 11	, ADC decimation factor = 64
//Interrupt polarity control 0 = active low , enable excitation source to CINx pins
//===================STAGE_CAL_EN Register = 0x01=======================
//STAGE0-STAGE11 calibration enable
//===================AMB_COMP_CTRL0 Register = 0x02=======================
//Fast filter skip control (N + 1) = 11,	
//FP_PROXIMITY_CNT * 16 * time for one conversion sequence in full power mode
	.sys_cfg_reg = {0x02B0,0x0000,0x3BBB,0x0819,0x0832,0x0000,0x0000,0x0000},
	
};

struct ad714x_chip ad714x_config = 
{
    .idx = 0,
	.is_ad7417 = {false,false},
	.read = heyos_hal_spi_read,
	.write = heyos_hal_spi_write,
    .hw = &hw_config,
};

int main(void)
{
    BoardsInit();
	heyos_hal_spim_init(SPI_IDX_AD7174_1);
	ad7147_init(&ad714x_config);
    ad714x_config.idx =1;
	heyos_hal_spim_init(SPI_IDX_AD7174_2);
    ad7147_init(&ad714x_config);
	while(1)
	{
        am_hal_flash_delay(500000);
        ad714x_probe(&ad714x_config,0);
		am_hal_flash_delay(500000);
		ad714x_probe(&ad714x_config,1);
	}
}

