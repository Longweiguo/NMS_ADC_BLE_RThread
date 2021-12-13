
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

typedef uint32_t heyos_ret_t;

#define HEYOS_SUCC                               0x00

/*
 * Definition for HEYOS Component ID
 */
#define HEYOS_CID_NONE                           0x00
#define HEYOS_CID_OS                             0x01
#define HEYOS_CID_MODULE                         0x02
#define HEYOS_CID_FP                             0x03
#define HEYOS_CID_SE                             0x04
#define HEYOS_CID_GUI                            0x05
#define HEYOS_CID_GSENSOR                        0x06
#define HEYOS_CID_GUP                            0x07
#define HEYOS_CID_APP                            0x08
#define HEYOS_CID_HAL_FLASH                      0x09
#define HEYOS_CID_I2C                            0x0A
#define HEYOS_CID_SPI                            0x0B
#define HEYOS_CID_PROTOC                         0x0C
#define HEYOS_CID_BLE                            0x0D
#define HEYOS_CID_R_XFER                         0x0E
#define HEYOS_CID_PB                             0x0F
#define HEYOS_CID_TBL                            0x10
#define HEYOS_CID_SCHED                          0x11
#define HEYOS_CID_WMS                            0x12
#define HEYOS_CID_MSG                            0x13
#define HEYOS_CID_CMS                            0x14
#define HEYOS_CID_ACTIVITY_MENU                  0x15
#define HEYOS_CID_TMS                            0x16
#define HEYOS_CID_AMS                            0x17
#define HEYOS_CID_SURFACE                        0x18
#define HEYOS_CID_AES                            0x19
#define HEYOS_CID_OTA                            0x1A
#define HEYOS_CID_MIJIA                          0x1B
#define HEYOS_CID_PSM                            0x1C
#define HEYOS_CID_SERVICE_RYEEX                  0x1D
#define HEYOS_CID_RAM                            0x1E

/*
 * Definition for HEYOS Component ID
 */
#define HEYOS_ERR_INVALID_PARA                   0x01
#define HEYOS_ERR_NO_MEM                         0x02
#define HEYOS_ERR_TIMEOUT                        0x03
#define HEYOS_ERR_INVALID_TYPE                   0x04
#define HEYOS_ERR_INVALID_CMD                    0x05
#define HEYOS_ERR_INVALID_EVT                    0x06
#define HEYOS_ERR_INVALID_FCS                    0x07
#define HEYOS_ERR_BUSY                           0x08
#define HEYOS_ERR_ERASE                          0x09
#define HEYOS_ERR_WRITE                          0x0A
#define HEYOS_ERR_READ                           0x0B
#define HEYOS_ERR_INIT_FAIL                      0x0C
#define HEYOS_ERR_TEST_FAIL                      0x0D
#define HEYOS_ERR_INIT_DEFAULT                   0x0F
#define HEYOS_ERR_TABLE_FULL                     0x10
#define HEYOS_ERR_TABLE_EXISTED                  0x11
#define HEYOS_ERR_TABLE_NOT_FOUND                0x12
#define HEYOS_ERR_PARA_RESTORE                   0x13
#define HEYOS_ERR_PARA_SAVE                      0x14
#define HEYOS_ERR_QUEUE_FULL                     0x15
#define HEYOS_ERR_INVALID_STATE                  0x16
#define HEYOS_ERR_OPEN                           0x17
#define HEYOS_ERR_ENCHEYOSPTION                  0x18
#define HEYOS_ERR_DECHEYOSPTION                  0x19
#define HEYOS_ERR_TIMESTAMP                      0x20
#define HEYOS_ERR_NOT_CONNECTED                  0x21
#define HEYOS_ERR_CONTORL                        0x22
#define HEYOS_ERR_THREAD_CREATION                0x23
#define HEYOS_ERR_TIMER                          0x24
#define HEYOS_ERR_SEMAPHORE                      0x25
#define HEYOS_ERR_MUTEX                          0x26
#define HEYOS_ERR_QUEUE                          0x27
#define HEYOS_ERR_EVENT_TIMEOUT                  0x28
#define HEYOS_ERR_ENCRYPTION                     0x29
#define HEYOS_ERR_DECRYPTION                     0x2A
#define HEYOS_ERR_NOT_SUPPORT                    0x2B
#define HEYOS_ERR_RAM_HEAP_INIT                  0x2C
#define HEYOS_ERR_CONSUMED                       0x2D
/*
 * Marco to build and parse Error code
 */
#define HEYOS_SET_STS_VAL(cid, status)           (((uint16_t)cid)<<8 | (status))
#define HEYOS_GET_CID_FROM_ERR(e)                (((e)&0xff00)>>8)
#define HEYOS_GET_STS_FROM_ERR(e)                ((e)&0x00ff)
#ifdef __cplusplus
}
#endif
