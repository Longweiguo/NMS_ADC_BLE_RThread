/*
* Copyright (c), HEYOS Inc.
*
* All rights are reserved. Reproduction in whole or in part is
* prohibited without the written consent of the copyright owner.
*
*/
#ifndef __HEYOS_HAL_SPI_FLASH_H__
#define __HEYOS_HAL_SPI_FLASH_H__

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */

#include "heyos_type.h"
#include "nand/nand_flash.h"

/*
 * CONSTANTS
 *******************************************************************************
 */

/**
 * @brief  Type definition for FLASH parameters
 */
#define EXFLASH_SECTOR_SIZE             4096
#define EXFLASH_SECTOR_NUM              1024
#define EXFLASH_BYTES_SIZE              (1024*1024*4)
#define FONT_BLOCKS_LOCK_ENABLE         (FALSE)

/*
 * TYPES
 *******************************************************************************
 */


/*
 * FUNCTIONS
 *******************************************************************************
 */

/**
 * @brief   API to init SPI flash module
 *
 * @param   None
 *
 * @return  None
 */
void heyos_hal_nand_flash_init(void);

/**
 * @brief   API to deinit SPI flash module
 *
 * @param   None
 *
 * @return  None
 */
void heyos_hal_nand_flash_deinit(void);

/**
 * @brief   API to power on SPI flash module
 *
 * @param   None
 *
 * @return  None
 */
void heyos_hal_nand_flash_poweron(void);

/**
 * @brief   API to power down SPI flash module
 *
 * @param   None
 *
 * @return  None
 */
void heyos_hal_nand_flash_powerdown(void);

/**
 * @brief   API to get flash ID
 *
 * @param   None
 *
 * @return  None
 */
uint32_t heyos_hal_nand_flash_get_id(void);

/**
 * @brief   API to read SPI Flash data
 *
 * @param   p_buff  - Buffer to the read data
 * @param   addr    - The target address to read
 * @param   len     - Length to read
 *
 * @return  Success read number
 */
uint32_t heyos_hal_nand_flash_read(uint8_t *p_buff, uint32_t addr, uint32_t len);

/**
 * @brief   API to write SPI Flash data
 *
 * @param   p_buff  - Buffer to the write data
 * @param   addr    - The target address to write
 * @param   len     - Length to write
 *
 * @return  Success write numbe
 */
uint32_t heyos_hal_nand_flash_write(uint8_t *p_buff, uint32_t addr, uint32_t len);

/**
 * @brief   API to write SPI Flash data
 *
 * @param   p_buff  - Buffer to the write data
 * @param   addr    - The target address to write
 * @param   len     - Length to write
 *
 * @return  Success write numbe
 */
int heyos_hal_nand_flash_write_page_raw(int page, uint8_t* data);

/**
 * @brief   API to erase whole flash
 *
 * @param   None
 *
 * @return  None
 */
void heyos_hal_nand_flash_chip_erase(void);


/**
 * @brief   API to erase specified block
 *
 * @param   block_addr - The block address to erase
 *
 * @return  0: Success, -1: Fail.
 */
int heyos_hal_nand_flash_block_erase(uint32_t block_addr);

/**
 * @brief API to erase a block
 *
 * @param block
 * @return int
 */
int heyos_hal_nand_flash_block_erase2(int block);
/**
 * @brief   API to erase specified address and length
 *
 * @param   addr - The nand flash address to erase
 * @param   length - Length to erase
 *
 * @return  0: Success, -1: Fail.
 */
int heyos_hal_nand_flash_erase_with_len(uint32_t addr, int length);



/**
 * @brief   API to wakeup the flash
 *
 * @param   None
 *
 * @return  None
 */
void heyos_hal_nand_flash_wakeup(void);


/**
 * @brief   API to check is a block #N is bad block
 *
 * @param   block - the block id
 *
 * @return  0: not bad block, 1: bad block
 */
bool heyos_hal_nand_flash_is_bad_block(int block);

/**
 * @brief   API to mark the block as bad block
 * @param   block - the block id
 *
 * @return  None
 */
void heyos_hal_nand_flash_bad_block_mark(int block);

/**
 * @brief   API to read flash size;
 * @param   block - the block id
 *
 * @return  None
 */
uint32_t heyos_hal_nand_flash_size(void);

/**
 * @brief   API to check if the page is free.
 * @param   page = page number.
 *
 * @return  None
 */
int heyos_hal_nand_flash_is_page_free(int page);


/**
 * @brief   API to read data with page number and offset.
 * 
 * @param   page - page number.
 * @param   data - pointer of the read back data.
 * @param   offset - offset of the page.
 * @param   len - length to read
 *
 * @return  0 means success and -1 means error.
 */
int heyos_hal_nand_flash_read_page_with_offset(int page, uint8_t* data, int offset, int len);


#ifdef __cplusplus
}
#endif

#endif // __HEYOS_HAL_NAND_FLASH_H__
