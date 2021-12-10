#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "AD7147.h"
#include "heyos_hal_spi.h"
#include "am_mcu_apollo.h"
#include "am_bsp.h"
#include "am_util.h"

typedef struct {
    void* handle;
    uint8_t ChipSelect;
    uint8_t iom_moudle;
}iom_spi_desc;

static iom_spi_desc spi_idx_dev[SPI_IDX_MAX] = {
    [SPI_IDX_AD7174_1] = {
        .iom_moudle = 0,
        .handle = NULL,
        .ChipSelect = 2
    },
    [SPI_IDX_AD7174_2] = {
        .iom_moudle = 1,
        .handle = NULL,
        .ChipSelect = 0
    }
};

static am_hal_iom_config_t g_sIOMSpiConfig = {
    .eInterfaceMode = AM_HAL_IOM_SPI_MODE,
    .ui32ClockFreq  = AM_HAL_IOM_1MHZ,
    .eSpiMode = AM_HAL_IOM_SPI_MODE_0,
};

void heyos_hal_spim_init(heyos_spim_inst_t idx)
{
    iom_spi_desc* spi_dev;

    spi_dev = &spi_idx_dev[idx];
    am_hal_iom_initialize(spi_dev->iom_moudle, &spi_dev->handle);
    am_hal_iom_power_ctrl(spi_dev->handle, AM_HAL_SYSCTRL_WAKE, false);

    am_hal_iom_configure(spi_dev->handle, &g_sIOMSpiConfig);
    am_bsp_iom_pins_enable(spi_dev->iom_moudle, AM_HAL_IOM_SPI_MODE);

    am_hal_iom_enable(spi_dev->handle);
}

void heyos_hal_spim_deinit(heyos_spim_inst_t idx)
{
    iom_spi_desc* spi_dev;

    spi_dev = &spi_idx_dev[idx];
    am_hal_iom_disable(spi_dev->handle);

    am_hal_iom_power_ctrl(spi_dev->handle, AM_HAL_SYSCTRL_DEEPSLEEP, true);
    am_bsp_iom_pins_disable(spi_dev->iom_moudle, AM_HAL_IOM_SPI_MODE);

    am_hal_iom_uninitialize(spi_dev->handle);
}

static uint32_t spim_tx_rx(heyos_spim_inst_t idx, uint32_t reg, uint8_t* pTxData, uint8_t* pRxData, uint32_t len)
{
    am_hal_iom_transfer_t       Transaction;
    am_hal_iom_dir_e            Tran_dir = AM_HAL_IOM_FULLDUPLEX;

    if(pRxData != NULL && pTxData != NULL) {
        Tran_dir = AM_HAL_IOM_FULLDUPLEX;
    } else if (pRxData != NULL) {
        Tran_dir = AM_HAL_IOM_RX;
    } else if (pTxData != NULL) {
        Tran_dir = AM_HAL_IOM_TX;
    }

    Transaction.ui32InstrLen    = 2;
    Transaction.ui32Instr       = reg;
    Transaction.eDirection      = Tran_dir;
    Transaction.ui32NumBytes    = len;
    Transaction.pui32TxBuffer   = (uint32_t*)pTxData;
    Transaction.pui32RxBuffer   = (uint32_t*)pRxData;
    Transaction.bContinue       = false;
    Transaction.ui8RepeatCount  = 0;
    Transaction.ui32PauseCondition = 0;
    Transaction.ui32StatusSetClr = 0;

    Transaction.uPeerInfo.ui32SpiChipSelect = spi_idx_dev[idx].ChipSelect;


    return am_hal_iom_blocking_transfer(spi_idx_dev[idx].handle, &Transaction);
}

uint32_t heyos_hal_spi_read(struct ad714x_chip *ad714x, unsigned short reg, unsigned short *buf, unsigned char len)
{
	uint32_t spi_ret;
    uint8_t data[40] = {0};
    uint16_t addr = 0xE400 | reg;
    spi_ret = spim_tx_rx((heyos_spim_inst_t)ad714x->idx,addr,NULL,data,len*2);
    for(char i = 0; i < len; i++){
        buf[i] = data[i*2]<<8 | data[i*2+1];
        if(len>20)break;
    }
	
	return spi_ret;
}

uint32_t heyos_hal_spi_write(struct ad714x_chip *ad714x, unsigned short reg, unsigned short data)
{
	uint32_t spi_ret;
    uint8_t buf[2]; 

    buf[1] = data&0xff;
    buf[0] = (data&0xff00)>>8;
    uint16_t addr = 0xE000 | reg;
    spi_ret = spim_tx_rx((heyos_spim_inst_t)ad714x->idx,addr,buf,NULL,2);
	
	return spi_ret;
}
