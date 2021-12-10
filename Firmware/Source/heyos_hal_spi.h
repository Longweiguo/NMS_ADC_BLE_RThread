/*
* Copyright (c), RY Inc.
*
* All rights are reserved. Reproduction in whole or in part is
* prohibited without the written consent of the copyright owner.
*
*/
#ifndef __HEYOS_HAL_SPI_H__
#define __HEYOS_HAL_SPI_H__


/*********************************************************************
 * INCLUDES
 */
#include <stdint.h>

/*
 * CONSTANTS
 *******************************************************************************
 */

/* None */

/*
 * TYPES
 *******************************************************************************
 */

/**
 * @brief  Definition for SPI peripheral instance
 */
typedef enum {
    SPI_IDX_AD7174_1,
	SPI_IDX_AD7174_2,
    SPI_IDX_MAX
} heyos_spim_inst_t;

/*
 * FUNCTIONS
 *******************************************************************************
 */

/**
 * @brief   API to init and enable specified SPI master module
 *
 * @param   idx  - The specified spi instance
 *
 * @return  None
 */
void heyos_hal_spim_init(heyos_spim_inst_t idx);

/**
 * @brief   API to init and enable specified SPI master module
 *
 * @param   idx  - The specified spi instance
 *
 * @return  None
 */
void heyos_hal_spim_deinit(heyos_spim_inst_t idx);


/**
 * @brief   API to power up specified SPI module
 *
 * @param   idx  - The specified SPI instance
 *
 * @return  None
 */
void heyos_hal_spim_powerup(heyos_spim_inst_t idx);

/**
 * @brief   API to power down specified SPI module
 *
 * @param   idx  - The specified SPI instance
 *
 * @return  None
 */
void heyos_hal_spim_powerdown(heyos_spim_inst_t idx);

/**
 * @brief   API to send/receive SPI data
 *
 * @param   idx     - The specified spi instance
 * @param   pTxData - Pointer to the TX data to be sent
 * @param   pRxData - Pointer to the RX data to receive
 * @param   len     - Length of buffer
 *
 * @return  Status  - result of SPI read or wirte
 *			          0: HEYOS_SUCC, else: fail
 */
uint32_t heyos_hal_spim_txrx(heyos_spim_inst_t idx, uint16_t* pTxData, uint16_t* pRxData, uint32_t len);

/**
 * @brief   API to send/receive SPI data
 *
 * @param   idx     - The specified spi instance
 * @param   reg     - device register
 * @param   pTxData - Pointer to the TX data to be sent
 * @param   pRxData - Pointer to the RX data to receive
 * @param   len     - Length of buffer
 *
 * @return  Status  - result of SPI read or wirte
 *			          0: HEYOS_SUCC, else: fail
 */
uint8_t heyos_hal_spim_reg_txrx(heyos_spim_inst_t idx,uint32_t reg, uint8_t* pTxData, uint8_t* pRxData, uint32_t len);

struct ad714x_chip;
uint32_t heyos_hal_spi_read(struct ad714x_chip *ad714x, unsigned short reg, unsigned short *buf, unsigned char len);
uint32_t heyos_hal_spi_write(struct ad714x_chip *ad714x, unsigned short reg, unsigned short data);
#endif  /* __HEYOS_HAL_SPI_MASTER_H__ */
