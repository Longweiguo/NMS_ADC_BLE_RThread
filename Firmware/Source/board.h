#pragma once

#define SYSTICK_PERIOD_MS       10
#define XT_PERIOD               32768
#define AM_SRAM_SIZE            (768*1024)
#define AM_SRAM_END             (0x10000000 + AM_SRAM_SIZE * 1024)
#define WAKE_INTERVAL           XT_PERIOD * SYSTICK_PERIOD_MS * 1e-3
void heyos_board_init(void);

