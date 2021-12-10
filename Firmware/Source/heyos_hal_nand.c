#if 0
#include "heyos.h"
#include "heyos_def.h"
//#include "heyos_utils.h"
#include "heyos_hal_nand_flash.h"
#include "heyos_hal_mcu.h"

#include "am_bsp.h"
#include "am_util_delay.h"
#include "am_mcu_apollo.h"
//#include "am_devices_mspi_flash.h"
//#include "nand/nand_flash.h"
#include <string.h>

#define NAND_FLASHS_MSPI_NAND_TIMEOUT           10000
#define NAND_MSPI_MODULE                        0

#define    DMA_SIZE                             256
uint32_t   nand_DMATCBBuffer[DMA_SIZE];

#define   heyos_delay_ms(ms)                    am_util_delay_ms(ms);
#define   heyos_block_us(us)                    am_util_delay_us(us);

void *pMSPI0Handle = NULL;
//static heyos_sem_t sem_mspi0 = NULL;
//static bool dma_flg = false;
static bool debug_en = false;

static uint8_t temp_buf[FLASH_PAGE_SIZE + 64];

#define NAND_DEBUG(_fmt_, ...)                  //if(debug_en){LOG_DEBUG_RAW(_fmt_, ##__VA_ARGS__);}
#define NAND_INFO(x, ...)                   do{am_util_stdio_printf(x, ##__VA_ARGS__);}while(0)

#define heyos_memcpy        memcpy
#define heyos_memset        memset
#define LOG_DEBUG_RAW(_fmt_, ...)

static void flash_delay_100ns(void)
{
    __nop();
    __nop();
    __nop();
    __nop();
    __nop();
}

static void quad_dma_r_cb(void* callback, uint32_t status)
{
    //    LOG_DEBUG_RAW("R~INT\n");
    //heyos_sem_post(sem_mspi0);
}

static void quad_dma_w_cb(void* callback, uint32_t status)
{
    //    LOG_DEBUG_RAW("W~INT\n");
    //heyos_sem_post(sem_mspi0);
}

static void quad_dma_flg_cb(void* callback, uint32_t status)
{
    //    LOG_DEBUG_RAW("W~INT\n");
    bool *flag = (bool*)callback;
    *flag = true;
}


static am_hal_mspi_dev_config_t  SerialCE0MSPIConfig =
{
    .ui8TurnAround        = 8,
    .eSpiMode             = AM_HAL_MSPI_SPI_MODE_0,
    .eClockFreq           = AM_HAL_MSPI_CLK_48MHZ,  // AM_HAL_MSPI_CLK_48MHZ
    .eAddrCfg             = AM_HAL_MSPI_ADDR_2_BYTE,
    .eInstrCfg            = AM_HAL_MSPI_INSTR_1_BYTE,
    .eDeviceConfig        = AM_HAL_MSPI_FLASH_SERIAL_CE0,
    .bSeparateIO          = true,
    .bSendInstr           = true,
    .bSendAddr            = true,
    .bTurnaround          = true,
    .ui8ReadInstr         = NAND_MSPI_FLASH_QUAD_READ,
    .ui8WriteInstr        = NAND_MSPI_FLASH_QUAD_PROGRAM,
    .ui32TCBSize          = DMA_SIZE,
    .pTCB                 = nand_DMATCBBuffer,
    .scramblingStartAddr  = 0,
    .scramblingEndAddr    = 0,
};

static am_hal_mspi_dev_config_t  QuadPairedSerialMSPIConfig =
{
    .ui8TurnAround        = 8,
    .eSpiMode             = AM_HAL_MSPI_SPI_MODE_0,
    .eClockFreq           = AM_HAL_MSPI_CLK_24MHZ,    // AM_HAL_MSPI_CLK_48MHZ
    .eAddrCfg             = AM_HAL_MSPI_ADDR_2_BYTE,
    .eInstrCfg            = AM_HAL_MSPI_INSTR_1_BYTE,
    .eDeviceConfig        = AM_HAL_MSPI_FLASH_QUAD_CE0,//AM_HAL_MSPI_FLASH_QUADPAIRED_SERIAL,
    .bSendAddr            = true,
    .bSendInstr           = true,
    .bSeparateIO          = false,
    .bTurnaround          = true,
    .ui8ReadInstr         = NAND_MSPI_FLASH_QUAD_READ,
    .ui8WriteInstr        = NAND_MSPI_FLASH_QUAD_PROGRAM,
    .ui32TCBSize          = DMA_SIZE,
    .pTCB                 = nand_DMATCBBuffer,
    .scramblingStartAddr  = 0,
    .scramblingEndAddr    = 0,
};

void am_mspi0_isr(void)
{
    uint32_t      ui32Status;

    am_hal_mspi_interrupt_status_get(pMSPI0Handle, &ui32Status, false);
    am_hal_mspi_interrupt_clear(pMSPI0Handle, ui32Status);
    am_hal_mspi_interrupt_service(pMSPI0Handle, ui32Status);
}

static uint32_t nand_flash_command_read(void *pMspiHandle,
                                       bool bSendInstr,
                                       uint8_t ui8Instr,
                                       bool bSendAddr,
                                       uint32_t ui32Addr,
                                       uint32_t *pData,
                                       uint32_t ui32NumBytes)
{
  am_hal_mspi_pio_transfer_t  Transaction;

  // Create the individual write transaction.
  Transaction.ui32NumBytes            = ui32NumBytes;
  Transaction.bScrambling             = false;
  Transaction.eDirection              = AM_HAL_MSPI_RX;
  Transaction.bSendAddr               = bSendAddr;
  Transaction.ui32DeviceAddr          = ui32Addr;
  Transaction.bSendInstr              = bSendInstr;
  Transaction.ui16DeviceInstr         = ui8Instr;
  Transaction.bTurnaround             = false;
#if defined(AM_PART_APOLLO3P)
  Transaction.bDCX                    = false;
  Transaction.bEnWRLatency            = false;
  Transaction.bContinue               = false;
#endif
  Transaction.bQuadCmd                = false;
  Transaction.pui32Buffer             = pData;

  // Execute the transction over MSPI.
  uint32_t status =  am_hal_mspi_blocking_transfer(pMspiHandle,
                                       &Transaction,
                                       NAND_FLASHS_MSPI_NAND_TIMEOUT);

  if(status != AM_HAL_STATUS_SUCCESS) {
      //LOG_DEBUG_RAW("nand mspi fail\n");
  }
  return status;
}

static uint32_t nand_flash_command_write(void *pMspiHandle,
                                        bool bSendInstr,
                                        uint8_t ui8Instr,
                                        bool bSendAddr,
                                        uint32_t ui32Addr,
                                        uint32_t *pData,
                                        uint32_t ui32NumBytes)
{
  am_hal_mspi_pio_transfer_t  Transaction;

  // Create the individual write transaction.
  Transaction.ui32NumBytes            = ui32NumBytes;
  Transaction.bScrambling             = false;
  Transaction.eDirection              = AM_HAL_MSPI_TX;
  Transaction.bSendAddr               = bSendAddr;
  Transaction.ui32DeviceAddr          = ui32Addr;
  Transaction.bSendInstr              = bSendInstr;
  Transaction.ui16DeviceInstr         = ui8Instr;
  Transaction.bTurnaround             = false;
#if defined(AM_PART_APOLLO3P)
  Transaction.bDCX                    = false;
  Transaction.bEnWRLatency            = false;
  Transaction.bContinue               = false;
#endif
  Transaction.bQuadCmd                = false;
  Transaction.pui32Buffer             = pData;

  // Execute the transction over MSPI.
  uint32_t status = am_hal_mspi_blocking_transfer(pMspiHandle,
                                &Transaction,
                                NAND_FLASHS_MSPI_NAND_TIMEOUT);

  if(status != AM_HAL_STATUS_SUCCESS) {
      LOG_DEBUG_RAW("[%s] nand mspi fail: %d\n", __FUNCTION__, status);
  }
  return status;
}

static uint32_t nand_flash_quad_write(void *pMspiHandle,
                                      uint32_t ui32Addr,
                                      uint8_t *pData,
                                      uint32_t ui32NumBytes)
{
    am_hal_mspi_dma_transfer_t  Transaction;

    Transaction.ui8Priority             = 1;
    Transaction.eDirection              = AM_HAL_MSPI_TX;
    Transaction.ui32TransferCount       = ui32NumBytes;
    Transaction.ui32SRAMAddress         = (uint32_t)pData;
    Transaction.ui32DeviceAddress       = ui32Addr;
    // Clear the CQ stimulus.
    Transaction.ui32PauseCondition = 0;
    // Clear the post-processing
    Transaction.ui32StatusSetClr = 0;

    // Execute the transction over MSPI.
    uint32_t status = am_hal_mspi_nonblocking_transfer(pMspiHandle,
                                          &Transaction,
                                          AM_HAL_MSPI_TRANS_DMA,
                                          quad_dma_w_cb, NULL);

    if(status != AM_HAL_STATUS_SUCCESS) {
        LOG_DEBUG_RAW("[%s] nand mspi fail: %d\n", __FUNCTION__, status);
    }
    return status;
}

static uint32_t nand_flash_quad_read(void *pMspiHandle,
                                     uint32_t ui32Addr,
                                     uint8_t *pData,
                                     uint32_t ui32NumBytes)
{
    am_hal_mspi_dma_transfer_t  Transaction;

    Transaction.ui8Priority             = 1;
    Transaction.eDirection              = AM_HAL_MSPI_RX;
    Transaction.ui32TransferCount       = ui32NumBytes;
    Transaction.ui32SRAMAddress         = (uint32_t)pData;
    Transaction.ui32DeviceAddress       = ui32Addr;
    // Clear the CQ stimulus.
    Transaction.ui32PauseCondition = 0;
    // Clear the post-processing
    Transaction.ui32StatusSetClr = 0;

    // Execute the transction over MSPI.

#ifdef RT_USING_SEMAPHORE
        uint32_t status = am_hal_mspi_nonblocking_transfer(pMspiHandle,
                                                           &Transaction,
                                                           AM_HAL_MSPI_TRANS_DMA,
                                                           quad_dma_r_cb, NULL);
#else
        bool dma_flg = false;
        uint32_t status = am_hal_mspi_nonblocking_transfer(pMspiHandle,
                                                           &Transaction,
                                                           AM_HAL_MSPI_TRANS_DMA,
                                                           quad_dma_flg_cb, &dma_flg);
        while(dma_flg != true) {
            am_util_delay_us(100);
        }
#endif

    if(status != AM_HAL_STATUS_SUCCESS) {
        LOG_DEBUG_RAW("[%s] nand mspi fail: %d\n", __FUNCTION__, status);
    }
    return status;
}

static int nand_flash_read_reg(uint8_t Instr, uint32_t* value, uint32_t len)
{
    uint32_t sta;
    sta = nand_flash_command_read(pMSPI0Handle, true, Instr, false , NULL, value, len);

    return (sta == AM_HAL_STATUS_SUCCESS) ? HEYOS_SUCC : HEYOS_SET_STS_VAL(HEYOS_CID_HAL_FLASH, HEYOS_ERR_READ);
}

static int nand_flash_write_reg(uint8_t Instr, uint32_t* value, uint32_t len)
{
    uint32_t sta;
    sta = nand_flash_command_write(pMSPI0Handle, true, Instr, false, NULL, value, len);

    return (sta == AM_HAL_STATUS_SUCCESS) ? HEYOS_SUCC : HEYOS_SET_STS_VAL(HEYOS_CID_HAL_FLASH, HEYOS_ERR_WRITE);
}

static int nand_flash_read_config(uint8_t Instr, uint8_t addr, uint32_t* value, uint32_t len)
{
    uint32_t addr_instr = (Instr << 8) | addr;
    uint32_t sta;
    sta = nand_flash_command_read(pMSPI0Handle, false, NULL, true , addr_instr, value, len);

    return (sta == AM_HAL_STATUS_SUCCESS) ? HEYOS_SUCC : HEYOS_SET_STS_VAL(HEYOS_CID_HAL_FLASH, HEYOS_ERR_READ);
}

static int nand_flash_write_config(uint8_t Instr, uint8_t addr, uint32_t* value, uint32_t len)
{
    uint32_t addr_instr = (Instr << 8) | addr;
    uint32_t sta;
    sta = nand_flash_command_write(pMSPI0Handle, false, NULL, true , addr_instr, value, len);

    return (sta == AM_HAL_STATUS_SUCCESS) ? HEYOS_SUCC : HEYOS_SET_STS_VAL(HEYOS_CID_HAL_FLASH, HEYOS_ERR_WRITE);
}

static int nand_flash_command(uint8_t Instr, uint32_t* value, uint32_t len)
{
    uint32_t sta;
    sta = nand_flash_command_write(pMSPI0Handle, true, Instr, false , NULL, value, len);

    return (sta == AM_HAL_STATUS_SUCCESS) ? HEYOS_SUCC : HEYOS_SET_STS_VAL(HEYOS_CID_HAL_FLASH, HEYOS_ERR_WRITE);
}

static int nand_flash_wait(uint8_t sta, uint32_t us)
{
    uint8_t r_data[3] = {0};
    uint8_t status = 0;
    uint8_t nr_try = 10;

    while (us) {
        us--;
        nand_flash_read_config(NAND_MSPI_FLASH_GET_FEATURE, 0xC0, (uint32_t*)&r_data[0], 1);
        if ((r_data[0] == 0) || (nr_try == 0)) {
            break;
        }
        if((r_data[0] & NAND_MSPI_FLASH_STATUS_BUSY) == 0) { // only try 10 times
            nr_try--;
        }
    }

    if (r_data[0] != 0x00) {
       NAND_DEBUG("\033[31msat-0x%02x,tried=%d\033[0m\n",r_data[0], 10 - nr_try);
    }

    //if(r_data[0] != 0x00) {
    //    LOG_DEBUG_RAW("\033[31msat-0x%02x\033[0m\n",r_data[0]);
    //}

    //if((sta & 0xFE) && us != 0) {
    if(sta & 0xFE) {
        status =  r_data[0] & (sta & 0xFE);
    }
    return status;
}

static int nand_flash_reset(void)
{
    return nand_flash_write_reg(NAND_MSPI_FLASH_RESET, NULL, 0);
}

void flash_config(uint32_t conf)
{
    uint8_t w_data[3];
    uint8_t r_data[3];

    nand_flash_read_config(NAND_MSPI_FLASH_GET_FEATURE,0xA0, (uint32_t*)&r_data[0],1);
    nand_flash_read_config(NAND_MSPI_FLASH_GET_FEATURE,0xB0, (uint32_t*)&r_data[1],1);
    nand_flash_read_config(NAND_MSPI_FLASH_GET_FEATURE,0xC0, (uint32_t*)&r_data[2],1);
    // set qe
    //    w_data[0] = r_data[1] | 0x01;
    w_data[0] = 0x01;
    nand_flash_write_config(NAND_MSPI_FLASH_SET_FEATURE,0xB0, (uint32_t*)w_data,1);

    // block_protection
    w_data[0] = 0x00;  // 1100 0111
    nand_flash_write_config(NAND_MSPI_FLASH_SET_FEATURE,0xA0, (uint32_t*)w_data,1);

    NAND_DEBUG("flash conf r_reg A0 0x%x\n", r_data[0]);
    NAND_DEBUG("flash conf r_reg B0 0x%x\n", r_data[1]);
    NAND_DEBUG("flash conf r_reg C0 0x%x\n", r_data[2]);

    nand_flash_read_config(NAND_MSPI_FLASH_GET_FEATURE,0xA0, (uint32_t*)&r_data[0],1);
    nand_flash_read_config(NAND_MSPI_FLASH_GET_FEATURE,0xB0, (uint32_t*)&r_data[1],1);
    nand_flash_read_config(NAND_MSPI_FLASH_GET_FEATURE,0xC0, (uint32_t*)&r_data[2],1);
    NAND_DEBUG("=========================\n");
    NAND_DEBUG("flash conf r_reg A0 0x%x\n", r_data[0]);
    NAND_DEBUG("flash conf r_reg B0 0x%x\n", r_data[1]);
    NAND_DEBUG("flash conf r_reg C0 0x%x\n", r_data[2]);
}

void heyos_hal_nand_flash_init(void)
{
    am_hal_mspi_dev_config_t    *psMSPISettings0;
    am_hal_mspi_dev_config_t    *psMSPISettings1;
    am_hal_gpio_pincfg_t g_AM_BSP_GPIO = {
        .uFuncSel            = 3,
        .eDriveStrength      = AM_HAL_GPIO_PIN_DRIVESTRENGTH_2MA,
        .eGPOutcfg           = AM_HAL_GPIO_PIN_OUTCFG_PUSHPULL,
        .ePullup             = AM_HAL_GPIO_PIN_PULLUP_WEAK,
    };

    // hold wp reset pin
    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI0_D0,  g_AM_BSP_GPIO);
    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI0_D1,  g_AM_BSP_GPIO);
    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI0_D2,  g_AM_BSP_GPIO);
    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI0_D3,  g_AM_BSP_GPIO);

    am_hal_gpio_state_write(AM_BSP_GPIO_MSPI0_D0, AM_HAL_GPIO_OUTPUT_SET);
    am_hal_gpio_state_write(AM_BSP_GPIO_MSPI0_D1, AM_HAL_GPIO_OUTPUT_SET);
    am_hal_gpio_state_write(AM_BSP_GPIO_MSPI0_D2, AM_HAL_GPIO_OUTPUT_SET);
    am_hal_gpio_state_write(AM_BSP_GPIO_MSPI0_D3, AM_HAL_GPIO_OUTPUT_SET);
    //--------------------------------------------------
    psMSPISettings0 = &SerialCE0MSPIConfig;
    psMSPISettings1 = &QuadPairedSerialMSPIConfig;
    // creat sem
    //sem_mspi0 = heyos_sem_create("nandMspi",0);

#if AM_APOLLO3_MCUCTRL
    am_hal_mcuctrl_control(AM_HAL_MCUCTRL_CONTROL_FAULT_CAPTURE_ENABLE, 0);
#else // AM_APOLLO3_MCUCTRL
    am_hal_mcuctrl_fault_capture_enable();
#endif // AM_APOLLO3_MCUCTRL

    if(AM_HAL_STATUS_SUCCESS != am_hal_mspi_initialize(NAND_MSPI_MODULE, &pMSPI0Handle)) {
        NAND_DEBUG("nand mspi init fail\n");
        return;
    }

    //    am_hal_mspi_disable(pMSPI0Handle);
    if(AM_HAL_STATUS_SUCCESS != am_hal_mspi_power_control(pMSPI0Handle, AM_HAL_SYSCTRL_WAKE, false)) {
        NAND_DEBUG("nand mspi control \n");
        return;
    }

    if(AM_HAL_STATUS_SUCCESS != am_hal_mspi_device_configure(pMSPI0Handle, psMSPISettings0)){
        NAND_DEBUG("nand mspi configure fail\n");
        return;
    }
    if(AM_HAL_STATUS_SUCCESS != am_hal_mspi_enable(pMSPI0Handle)) {
        NAND_DEBUG("nand mspi enable fail\n");
        return;
    }
    am_bsp_mspi_pins_enable(NAND_MSPI_MODULE, psMSPISettings0->eDeviceConfig);
    am_util_delay_us(150);
    // reset
    nand_flash_reset();
    am_util_delay_ms(5);

    heyos_hal_nand_flash_get_id();
    //    am_util_delay_us(10);

    flash_config(0);
    am_util_delay_us(10);
    // config 1:1:4
    am_hal_mspi_disable(pMSPI0Handle);


    mspi_device_info_t pConfig = {
        .eDeviceConfig = AM_HAL_MSPI_FLASH_SERIAL_CE0,
        .eXipMixedMode = AM_HAL_MSPI_XIPMIXED_D4,
        .bSeparateIO = 1
    };
    if (AM_HAL_STATUS_SUCCESS != am_hal_mspi_control(pMSPI0Handle, AM_HAL_MSPI_REQ_DEVICE_CONFIG, (void *)&pConfig))
    {
        NAND_DEBUG("Error - Failed to put MSPI into mixed mode.\n");
        return;
    }
    am_hal_mspi_enable(pMSPI0Handle);
    am_bsp_mspi_pins_enable(NAND_MSPI_MODULE, psMSPISettings1->eDeviceConfig);

    if (AM_HAL_STATUS_SUCCESS != am_hal_mspi_interrupt_clear(pMSPI0Handle, AM_HAL_MSPI_INT_CQUPD |
                                                             AM_HAL_MSPI_INT_ERR ))
    {
        return;
    }

    if (AM_HAL_STATUS_SUCCESS !=  am_hal_mspi_interrupt_enable(pMSPI0Handle, AM_HAL_MSPI_INT_CQUPD |
                                                               AM_HAL_MSPI_INT_ERR ))
    {
        return;
    }
    NVIC_EnableIRQ(MSPI0_IRQn);
}

void heyos_hal_nand_flash_deinit(void);

void heyos_hal_nand_flash_poweron(void);

void heyos_hal_nand_flash_powerdown(void);

uint16_t heyos_hal_nand_flash_get_id(void) {

    uint8_t  data[8];
    uint32_t ui32DeviceID = 0;
    nand_flash_read_reg(NAND_MSPI_FLASH_READ_ID, (uint32_t*)data, 3);
    ui32DeviceID = (data[1] << 8) | data[2];
    LOG_DEBUG_RAW("\033[32mFlash ID 0x%x\033[0m\n",ui32DeviceID);
    return ui32DeviceID;
}

heyos_ret_t heyos_hal_nand_addr_convt(uint32_t addr,uint16_t *column_addr,uint32_t* row_addr)
{
//    uint32_t row_address = 0x00;

    *column_addr = addr & 0x07FF;
    *row_addr = addr >> 11;

    return HEYOS_SUCC;
}

/**
 * @brief Read oob data
 *
 * @param block
 * @param page
 * @param spare_data
 * @param spare_len
 */
static void read_page_oob(int block, int page, uint8_t* spare_data, int spare_len)
{
    // uint8_t status = 0;
    uint8_t row_addr[4] = {0};
    uint16_t column_address = 0x00;
    uint32_t row_address = 0x00;
    uint8_t oob_data[64] = {0};

    /* Resolve address */
    uint32_t addr = block*FLASH_BLOCK_SIZE + page * FLASH_PAGE_SIZE;
    heyos_hal_nand_addr_convt(addr, &column_address, &row_address);
    row_addr[0] = (uint8_t)((row_address >> 16) & 0xFF);
    row_addr[1] = (uint8_t)((row_address >> 8) & 0xFF);
    row_addr[2] = (uint8_t)((row_address) & 0xFF);
    //NAND_DEBUG("R:row addr 0x%02x%02x%02x, col0x%x\n",row_addr[0],row_addr[1],row_addr[2], col_address);

    /* Do read sequence */
    nand_flash_command(NAND_MSPI_FLASH_PAGE_READ, (uint32_t*)row_addr, 3);
    if (nand_flash_wait(NAND_MSPI_FLASH_STATUS_BUSY, 1000)) {
        //        LOG_ERROR("R:Nand read fail\n");
        return;
    }

    column_address = FLASH_PAGE_SIZE;
    /* support 2G ,set */
    if((addr & 0x20000) != 0 ){
        column_address |= 0x1000;
    } else {
        column_address &= 0xEFFF;
    }
    nand_flash_quad_read(pMSPI0Handle, column_address, oob_data, 64);
    heyos_block_us(200);

#ifdef RT_USING_SEMAPHORE
    heyos_sem_wait(sem_mspi0); // wait dma finsh
#endif

    /* Wait ready ready */
    nand_flash_wait(NAND_MSPI_FLASH_STATUS_BUSY, 1000);

    memcpy(spare_data, oob_data, spare_len);
}

/**
 * @brief Write oob data
 *
 * @param block
 * @param page
 * @param spare_data
 * @param spare_len
 */
static void write_page_oob(int block, int page, uint8_t* spare_data, int spare_len)
{
    uint8_t status = 0;
    uint8_t row_addr[4] = {0};
    uint16_t column_address = 0x00;
    uint32_t row_address = 0x00;
#ifdef RT_USING_SEMAPHORE
    heyos_mutex_take(nand_mutex);
#endif
    /* Resolve address */
    uint32_t addr = block*FLASH_BLOCK_SIZE + page * FLASH_PAGE_SIZE;
    heyos_hal_nand_addr_convt(addr, &column_address, &row_address);
    row_addr[0] = (uint8_t)((row_address >> 16) & 0xFF);
    row_addr[1] = (uint8_t)((row_address >> 8) & 0xFF);
    row_addr[2] = (uint8_t)((row_address) & 0xFF);
    //NAND_DEBUG("R:row addr 0x%02x%02x%02x, col0x%x\n",row_addr[0],row_addr[1],row_addr[2], col_address);

    /* Do write sequence */
    heyos_memset(temp_buf, 0xFF, 64);
    heyos_memcpy(temp_buf, spare_data, spare_len);
    nand_flash_write_reg(NAND_MSPI_FLASH_WRITE_ENABLE, NULL, 0);
    NAND_DEBUG("W:row addr 0x%02x%02x%02x, col0x%x\n",row_addr[0],row_addr[1],row_addr[2], column_address);

    /* support 2G ,set */
    if((addr & 0x20000) != 0 ){
        column_address |= 0x1000;
    } else {
        column_address &= 0xEFFF;
    }
    nand_flash_command_write(pMSPI0Handle, true,
            NAND_MSPI_FLASH_PAGE_PROGRAM, true, column_address, (uint32_t*)temp_buf, 64);
    nand_flash_command(NAND_MSPI_FLASH_PROGRAM_EXECUTE, (uint32_t*)row_addr, 3);
    heyos_block_us(300);

    /* Read status register */
    status = nand_flash_wait(NAND_MSPI_FLASH_STATUS_BUSY | NAND_MSPI_FLASH_STATUS_PROGRAM | NAND_MSPI_FLASH_STATUS_ECC, 1000);
    if (status != 0) {
        //        LOG_ERROR("\033[31mWrite fail But corrected. addr = %d, block %d, page = %d, len = %d, status = %d\033[0m\n",
        //                addr, (row_address >> 6), row_address & 0x3F, FLASH_PAGE_SIZE, status);
    }
}

/**
 * @brief API to check is bad block
 *
 * @param block
 * @return true
 * @return false
 */
bool heyos_hal_nand_flash_is_bad_block(int block)
{
    uint8_t spare_data[64] = {0};
    bool is_bad = false;
    // mutex inside read_page_oob
    heyos_memset(spare_data, 0xFF, 64);
    for (int i = 0; i < 2; i++) {
        read_page_oob(block, i, spare_data, 64);
        if (spare_data[0] != 0xFF) {
            is_bad = true;
            break;
        }
    }

    return is_bad;
}

uint32_t heyos_hal_nand_flash_read(uint8_t *p_buff, uint32_t addr, uint32_t len)
{
    uint8_t row_addr[4] = {0};
    uint16_t column_address = 0x00;
    uint32_t row_address = 0x00;
    uint32_t r_block_cnt = 0;
    uint32_t r_length = 0;
    uint32_t read_len = len;
    uint8_t read_buf[FLASH_PAGE_SIZE];
    uint8_t status = 0;

    heyos_hal_nand_addr_convt(addr, &column_address, &row_address);
    row_addr[0] = (uint8_t)((row_address >> 16) & 0xFF);
    row_addr[1] = (uint8_t)((row_address >> 8) & 0xFF);
    row_addr[2] = (uint8_t)((row_address) & 0xFF);
    NAND_DEBUG("R:addr[0x%08x]block[%d]pageNum[%d]page[0x%04x]-len[%d]\n",
                  addr, (row_address >> 6),
                  row_address & 0x3F,column_address,
                  len);

    if((len + column_address) > FLASH_PAGE_SIZE) {
        r_block_cnt = len > FLASH_PAGE_SIZE ? (len / FLASH_PAGE_SIZE - 1) : 1;
        read_len = FLASH_PAGE_SIZE - column_address;
        NAND_DEBUG("r_blocl_cnt %d,first read_len\n",r_block_cnt,read_len);
    }

    do {
        // set read block
        NAND_DEBUG("R:row addr 0x%02x%02x%02x, col0x%x\n",row_addr[0],row_addr[1],row_addr[2], column_address);
        nand_flash_command(NAND_MSPI_FLASH_PAGE_READ, (uint32_t*)row_addr, 3);
        // wait status
        //        am_util_delay_us(20);
        if(nand_flash_wait(NAND_MSPI_FLASH_STATUS_BUSY, 1000)) {
            NAND_INFO("R:Nand read fail\n");
            // error wait
            break;
        }

        /* support 2G ,set */
        if((addr & 0x20000) != 0 ){
            column_address |= 0x1000;
        } else {
            column_address &= 0xEFFF;
        }
        // read data in page address
        nand_flash_quad_read(pMSPI0Handle, column_address, read_buf, read_len);

        /* Read status register */
        status = nand_flash_wait(NAND_MSPI_FLASH_STATUS_BUSY | NAND_MSPI_FLASH_STATUS_ECC, 1000);
        if (status != 0) {
            if (((status & NAND_MSPI_FLASH_STATUS_ECC) >> 4) >  NAND_ECC_1_4_BIT_OK) {
                NAND_INFO("\033[31mNand Read fail. block %d, page = %d, len = %d, status = %d\033[0m\n",
                    (row_address >> 6), row_address & 0x3F, read_len, status);
                break;
            } else {
                NAND_DEBUG("\033[31mNand Read fail But corrected. block %d, page = %d, len = %d, status = %d\033[0m\n",
                    (row_address >> 6), row_address & 0x3F, read_len, status);
            }
        }
#ifdef RT_USING_SEMAPHORE
        heyos_sem_wait(sem_mspi0);         // wait dma finsh
#endif

        memcpy(p_buff, read_buf, read_len);
        r_length += read_len;
        p_buff += read_len;
        // len is more than flash page size, repeat to read
        if(r_length < len) {
            row_address++;
            row_addr[0] = (uint8_t)((row_address >> 16) & 0xFF);
            row_addr[1] = (uint8_t)((row_address >> 8) & 0xFF);
            row_addr[2] = (uint8_t)((row_address) & 0xFF);

            column_address = 0x0000;
            read_len = (len - r_length) > FLASH_PAGE_SIZE ? FLASH_PAGE_SIZE : (len - r_length);

            NAND_DEBUG("R:block[%d]pageNum[%d]page[0x%04x]-len[%d]\n",
                       (row_address >> 6),
                       row_address & 0x3F,column_address,
                       read_len);
        }
    } while(r_block_cnt--);

    return r_length;
}

/**
 * @brief Raw API to read nand data by specified address and length
 *
 * @param row_address
 * @param col_address
 * @param buf
 * @param len
 *
 * @return Status. 0 means success and -1 means fail.
 */
int _nand_flash_read(uint32_t row_address, uint32_t col_address, uint8_t* buf, int len)
{
    uint32_t status = 0;
    uint8_t row_addr[4] = {0};
    uint8_t temp_buf[FLASH_PAGE_SIZE] = {0};

    row_addr[0] = (uint8_t)((row_address >> 16) & 0xFF);
    row_addr[1] = (uint8_t)((row_address >> 8) & 0xFF);
    row_addr[2] = (uint8_t)((row_address) & 0xFF);

    nand_flash_command(NAND_MSPI_FLASH_PAGE_READ, (uint32_t*)row_addr, 3);

    if (nand_flash_wait(NAND_MSPI_FLASH_STATUS_BUSY, 1000)) {
        NAND_INFO("R:Nand read fail\n");
        return -1;
    }

    nand_flash_quad_read(pMSPI0Handle, col_address, temp_buf, len);

    /* Read status register */
    status = nand_flash_wait(NAND_MSPI_FLASH_STATUS_BUSY | NAND_MSPI_FLASH_STATUS_ECC, 1000);
    if (status != 0) {
        if (((status & NAND_MSPI_FLASH_STATUS_ECC) >> 4) >  NAND_ECC_1_4_BIT_OK) {
            NAND_INFO("\033[31mNand Read fail. block %d, page = %d, len = %d, status = %d\033[0m\n",
                (row_address >> 6), row_address & 0x3F, len, status);
            return -1;
        } else {
            NAND_DEBUG("\033[31mNand Read fail But corrected. block %d, page = %d, len = %d, status = %d\033[0m\n",
                (row_address >> 6), row_address & 0x3F, len, status);
        }
    }

#ifdef RT_USING_SEMAPHORE
    heyos_sem_wait(sem_mspi0);         // wait dma finsh
#endif

    memcpy(buf, temp_buf, len);

    return 0;
}

/**
 * @brief Wrapper API to read any address and any length Nand flash data.
 *
 * @param p_buff
 * @param addr
 * @param len
 *
 * @return int
 */
int heyos_hal_nand_flash_read_new(uint8_t *buf, uint32_t addr, int len)
{
    int status = 0;
    int len_to_read = 0;
    int offset = 0;
    int total_len = len;
    uint16_t col_address = 0;
    uint32_t row_address = 0;
    heyos_hal_nand_addr_convt(addr, &col_address, &row_address);

    /* Read current page data first. */
    int len_in_cur_page = FLASH_PAGE_SIZE - col_address;
    len_to_read = (len <= len_in_cur_page) ? len:len_in_cur_page;

    /* support 2G ,set */
    if((addr & 0x20000) != 0 ){
        col_address |= 0x1000;
    } else {
        col_address &= 0xEFFF;
    }

    status = _nand_flash_read(row_address, col_address, buf, len_to_read);
    if (status == -1) {
        return -1;
    }

    len -= len_to_read;
    offset += len_to_read;
    row_address++;

    /* Read left with whole page */
    while (len) {
        len_to_read = (len <= FLASH_PAGE_SIZE) ? len : FLASH_PAGE_SIZE;
        status = _nand_flash_read(row_address, 0, buf+offset, len_to_read);
        if (status == -1) {
            return -1;
        }

        len -= len_to_read;
        offset += len_to_read;
        row_address++;
    }

    return total_len;
}


uint32_t heyos_hal_nand_flash_write(uint8_t *p_buff, uint32_t addr, uint32_t len)
{
    uint8_t row_addr[4] = {0};
    uint16_t column_address = 0x00;
    uint32_t row_address = 0x00;
    uint32_t w_block_cnt = 0;
    uint32_t w_length = 0;
    uint32_t write_len = len;
    uint8_t write_buf[FLASH_PAGE_SIZE];

    heyos_hal_nand_addr_convt(addr, &column_address, &row_address);
    row_addr[0] = (uint8_t)((row_address >> 16) & 0xFF);
    row_addr[1] = (uint8_t)((row_address >> 8) & 0xFF);
    row_addr[2] = (uint8_t)((row_address) & 0xFF);

    NAND_DEBUG("W:addr[0x%08x]block[%d]pageNum[%d]page[0x%04x]-len[%d]\n",
                  addr, (row_address >> 6),
                  row_address & 0x3F,column_address,
                  len);

    if((len + column_address) > FLASH_PAGE_SIZE) {
        w_block_cnt = len > FLASH_PAGE_SIZE ? (len / FLASH_PAGE_SIZE - 1) : 1;
        write_len = FLASH_PAGE_SIZE - column_address;
        NAND_DEBUG("r_blocl_cnt %d,first read_len\n",w_block_cnt,write_len);
    }

    do {
        heyos_memcpy(write_buf, p_buff, write_len);
        nand_flash_write_reg(NAND_MSPI_FLASH_WRITE_ENABLE, NULL, 0);
        //        am_util_delay_us(5);
        NAND_DEBUG("W:row addr 0x%02x%02x%02x, col0x%x\n",row_addr[0],row_addr[1],row_addr[2], column_address);

        /* support 2G ,set */
        if((addr & 0x20000) != 0 ){
            column_address |= 0x1000;
        } else {
            column_address &= 0xEFFF;
        }

        nand_flash_command_write(pMSPI0Handle, true, NAND_MSPI_FLASH_PAGE_PROGRAM, true, column_address, (uint32_t*)write_buf, write_len);
//        nand_flash_quad_write(pMSPI0Handle, column_address, write_buf, write_len);
//        heyos_sem_wait(sem_mspi0);         // wait dma finsh
//       am_util_delay_us(100);
        nand_flash_command(NAND_MSPI_FLASH_PROGRAM_EXECUTE, (uint32_t*)row_addr, 3);
        //        LOG_DEBUG_RAW("W~\n");
        am_util_delay_us(300);
        nand_flash_wait(NAND_MSPI_FLASH_STATUS_BUSY, 1000);
//        am_util_delay_us(10);
//        nand_flash_write_reg(NAND_MSPI_FLASH_WRITE_DISABLE, NULL, 0);
        w_length += write_len;
        p_buff += write_len;
        // len is more than flash page size, repeat to read
        if(w_length < len) {
            row_address++;
            row_addr[0] = (uint8_t)((row_address >> 16) & 0xFF);
            row_addr[1] = (uint8_t)((row_address >> 8) & 0xFF);
            row_addr[2] = (uint8_t)((row_address) & 0xFF);

            column_address = 0x0000;
            write_len = (len - w_length) > FLASH_PAGE_SIZE ? FLASH_PAGE_SIZE : (len - w_length);

            NAND_DEBUG("W:block[%d]pageNum[%d]page[0x%04x]-len[%d]\n",
                       (row_address >> 6),
                       row_address & 0x3F,column_address,
                       write_len);
        }
    } while(w_block_cnt--);


    return w_length;
}

void heyos_hal_nand_flash_chip_erase(void);

int heyos_hal_nand_flash_block_erase(uint32_t addr)
{
    uint8_t row_addr[4] = {0};
    uint16_t column_address = 0x00;
    uint32_t row_address = 0x00;
    int status = 0x00;

    heyos_hal_nand_addr_convt(addr, &column_address, &row_address);
    row_addr[0] = (uint8_t)((row_address >> 16) & 0xFF);
    row_addr[1] = (uint8_t)((row_address >> 8) & 0xFF);
    row_addr[2] = (uint8_t)((row_address) & 0xFF);
    NAND_DEBUG("E:addr[0x%08x]block[%d]\n",
                  addr, (row_address >> 6));
    NAND_DEBUG("E:row addr 0x%02x%02x%02x, col0x%x\n",row_addr[0],row_addr[1],row_addr[2], column_address);
    // write enable
    nand_flash_write_reg(NAND_MSPI_FLASH_WRITE_ENABLE, NULL, 0);
    am_util_delay_us(10);
    // erase block
    nand_flash_command(NAND_MSPI_FLASH_BLOCK_ERASE, (uint32_t*)row_addr, 3);
    am_util_delay_ms(3);
    //    heyos_delay_ms(3);
    status = nand_flash_wait(NAND_MSPI_FLASH_STATUS_BUSY , 1000);
//    nand_flash_write_reg(NAND_MSPI_FLASH_WRITE_DISABLE, NULL, 0);
    am_util_delay_us(10);
    NAND_DEBUG("erase  status 0x%x\n",status);

    return status;
}

int heyos_hal_nand_flash_erase_with_len(uint32_t addr, int length)
{
    int block_size = FLASH_PAGE_SIZE * FLASH_PAGE_NUM_ONE_BLOCK;

    while (length > 0) {
        // LOG_DEBUG("[nand] erase 0x%X, %d\n", addr, length);
        heyos_hal_nand_flash_block_erase(addr);
        addr += block_size;
        length -= block_size;
    }
    return length;
}

void heyos_hal_nand_flash_wakeup(void);


// ---------------test func-------------------------------------------------------
#if 0
void flash_init(void)
{
    NAND_DEBUG("-----flash init------\n");
    heyos_hal_nand_flash_init();
}
MSH_CMD_EXPORT(flash_init, flash init API);

void flash_erase(void)
{
    uint32_t addr = 0;
    uint32_t block = DS35x_FLASH_1GBLOCK_SIZE - 1;
    while(block--) {
        addr += (DS35x_FLASH_PAGE_SIZE * DS35x_FLASH_PAGE_NUM);
        heyos_hal_nand_flash_block_erase(addr);
    }
}
MSH_CMD_EXPORT(flash_erase, get flash id);

void flash_id(void)
{
    heyos_hal_nand_flash_get_id();
}
MSH_CMD_EXPORT(flash_id, get flash id);

void flash_debug(void)
{
    debug_en = (debug_en == true)? false : true  ;
}
MSH_CMD_EXPORT(flash_debug, get flash id);

static void nand_write_test(void)
{
    uint8_t* w_buf = NULL;
    uint32_t addr =  1024 * 2 * 64;

    w_buf = (uint8_t*)heyos_sram_malloc((1024 * 2 * 64));

    if(w_buf == NULL) {
        LOG_DEBUG_RAW("malloc fail\n");
        return;
    }
    LOG_DEBUG_RAW("buff address 0x%x\n", w_buf);
    LOG_DEBUG_RAW("write addr 0x%x , write len %d\n", addr, 1024 * 2 * 64);
    uint8_t* temp_buf = w_buf;
    heyos_memset(temp_buf, 0xC5, 1024 * 2 * 64); // one page
//   uint8_t cnt = 0;
//   for(int i = 0 ; i < 1024 * 128 * 2; i++) {
//       *temp_buf = cnt++;
//       temp_buf++;
//   }
    heyos_hal_nand_flash_block_erase(addr);
    heyos_hal_nand_flash_block_erase(addr + (FLASH_PAGE_SIZE * FLASH_PAGE_NUM_ONE_BLOCK));
    heyos_hal_nand_flash_write(w_buf, addr, 1024 * 2 * 64);

    heyos_free(w_buf);
}

static void nand_read_test(void)
{
    uint8_t* r_buf = NULL;
    //    uint32_t addr =  1024 * 128;
    uint32_t addr =  1024 * 128 * 8;
    r_buf = (uint8_t*)heyos_sram_malloc((1024 * 128 * 2));
    heyos_memset(r_buf,0, 1024 * 128 * 2);
    if(r_buf == NULL) {
        LOG_DEBUG_RAW("malloc fail\n");
        return;
    }
    LOG_DEBUG_RAW("buff address 0x%x\n", r_buf);
    uint32_t start_tk = heyos_hal_clock_time();
    heyos_hal_nand_flash_read(r_buf, addr, 1024 * 128 * 2);
    LOG_DEBUG_RAW("nand read 128 * 2k use time %dms\n", heyos_hal_calc_ms(heyos_hal_clock_time() - start_tk));

//    uint8_t* temp_buf;
//    temp_buf = r_buf;
//    for(int i = 0; i < 1024 * 128 * 2; i++) {
//        LOG_DEBUG_RAW("0x%02x ", *temp_buf++);
//        if(i%16 == 0 && i > 0) LOG_DEBUG_RAW("\n");
//    }

    heyos_free(r_buf);
}

typedef struct {
    uint32_t bad_page_addr[1024];
    uint32_t bad_page_cnt;
}nand_black_t;

void nand_wr_test(void)
{
    uint32_t addr =  1024 * 128 * 1;
    uint8_t* r_buf = (uint8_t*)heyos_sram_malloc((1024 * 128));
    uint8_t* w_buf = (uint8_t*)heyos_sram_malloc((1024 * 128));
    uint8_t ck_data[] = { 0xAA, 0x73, 0x55, 0xCF,0x85};
    //    uint8_t* temp_buf = w_buf;
    //    uint8_t cnt = 0;
    uint8_t j = 0;
    uint8_t success = true;

    nand_black_t* block_t = heyos_malloc(sizeof(nand_black_t));
    heyos_memset(block_t, 0, sizeof(nand_black_t));

    if(r_buf == NULL || w_buf == NULL) {
        LOG_DEBUG_RAW("malloc fail\n");
        return;
    }

    for(int a = 0; a < 1023; a ++ ) {

        heyos_hal_nand_flash_block_erase(addr);
        //        for(int i = 0; i < 64; i++) {
            success = true;
            heyos_memset(r_buf,0, 1024*128);
            heyos_memset(w_buf, ck_data[j], 1024*128);
            //            LOG_DEBUG_RAW("#W~0x%02x\n",ck_data[j]);
            heyos_hal_nand_flash_write(w_buf, addr, 1024 * 128);
            heyos_hal_nand_flash_read(r_buf, addr, 1024 * 128);

            uint8_t* ck_buf;
            ck_buf = r_buf;
            for(int k = 0 ; k < 1024 *128; k++) {
                if(*ck_buf != ck_data[j]) {
                    success= false;
                }
                ck_buf++;
            }
            j = j >= 4 ? 0 : (j+1);
            if(success != true) {
                block_t->bad_page_cnt++;
                block_t->bad_page_addr[block_t->bad_page_cnt] = addr;
                LOG_DEBUG_RAW("\033[31m!!read fail\033[0m\n");
            }
            addr += 1024*128;
            //        }
        //        addr += FLASH_BLOCK_SIZE;
    }

    LOG_DEBUG_RAW("\033[31mbad page cnt %d\n", block_t->bad_page_cnt);
    for(int i = 0; i < block_t->bad_page_cnt; i++) {
        LOG_DEBUG_RAW("address: 0x%08x\n", block_t->bad_page_addr[block_t->bad_page_cnt++]);
    }
    LOG_DEBUG_RAW("\033[0m\n");

    heyos_free(r_buf);
    heyos_free(w_buf);
}


void bad_block_check(void)
{
    uint32_t addr =  1024 * 2;//1024 * 128 * 1;
    uint8_t spare_data[100] = {0};

    heyos_memset(spare_data, 0, 100);
    for(int i = 0; i < 64; i++) {
        heyos_hal_nand_flash_read(spare_data, addr, 64);
    }
}

void nand(int argc, char** argv)
{

    if(strcmp("erase", argv[1]) == NULL) {
        LOG_DEBUG_RAW("## nand erase\n");

    } else if(strcmp("badblock", argv[1]) == NULL) {
        LOG_DEBUG_RAW("## bad block check\n");


    }
    else if(strcmp("read", argv[1]) == NULL) {
        LOG_DEBUG_RAW("## read\n");
        nand_read_test();

    } else if(strcmp("write", argv[1]) == NULL) {
        LOG_DEBUG_RAW("## write\n");
        nand_write_test();
    } else if(strcmp("wr", argv[1]) == NULL) {
        LOG_DEBUG_RAW("## write\n");
        nand_wr_test();
    }
}
MSH_CMD_EXPORT(nand, nand flash operation);

#endif
#endif
