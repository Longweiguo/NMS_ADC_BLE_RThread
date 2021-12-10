#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include "AD7147.h"
#include "am_util_stdio.h"

#define AD714X_PWR_CTRL           0x0
#define AD714X_STG_CAL_EN_REG     0x1
#define AD714X_AMB_COMP_CTRL0_REG 0x2
#define AD714X_PARTID_REG         0x17
#define AD7142_PARTID             0xE620
#define AD7143_PARTID             0xE630
#define AD7147_PARTID             0x1470
#define AD7148_PARTID             0x1480
#define AD714X_STAGECFG_REG       0x80
#define AD714X_SYSCFG_REG         0x0

#define STG_LOW_INT_EN_REG     0x5
#define STG_HIGH_INT_EN_REG    0x6
#define STG_COM_INT_EN_REG     0x7
#define STG_LOW_INT_STA_REG    0x8
#define STG_HIGH_INT_STA_REG   0x9
#define STG_COM_INT_STA_REG    0xA

#define CDC_RESULT_S0          0xB
#define CDC_RESULT_S1          0xC
#define CDC_RESULT_S2          0xD
#define CDC_RESULT_S3          0xE
#define CDC_RESULT_S4          0xF
#define CDC_RESULT_S5          0x10
#define CDC_RESULT_S6          0x11
#define CDC_RESULT_S7          0x12
#define CDC_RESULT_S8          0x13
#define CDC_RESULT_S9          0x14
#define CDC_RESULT_S10         0x15
#define CDC_RESULT_S11         0x16

#define STAGE0_AMBIENT		0xF1
#define STAGE1_AMBIENT		0x115
#define STAGE2_AMBIENT		0x139
#define STAGE3_AMBIENT		0x15D
#define STAGE4_AMBIENT		0x181
#define STAGE5_AMBIENT		0x1A5
#define STAGE6_AMBIENT		0x1C9
#define STAGE7_AMBIENT		0x1ED
#define STAGE8_AMBIENT		0x211
#define STAGE9_AMBIENT		0x234
#define STAGE10_AMBIENT		0x259
#define STAGE11_AMBIENT		0x27D

#define PER_STAGE_REG_NUM      36

enum ad714x_device_state { IDLE, JITTER, ACTIVE, SPACE };
struct ad714x_touchpad_drv {
	int x_highest_stage;
	int x_flt_pos;
	int x_abs_pos;
	int y_highest_stage;
	int y_flt_pos;
	int y_abs_pos;
	int left_ep;
	int left_ep_val;
	int right_ep;
	int right_ep_val;
	int top_ep;
	int top_ep_val;
	int bottom_ep;
	int bottom_ep_val;
	enum ad714x_device_state state;
};

struct ad714x_driver_data {
	struct ad714x_touchpad_drv *touchpad;
};

//set STAGE_COMPLETE_INT_ENABLE, STAGE_HIGH_INT_ENABLE Register
static void ad714x_use_com_int(struct ad714x_chip *ad714x,
				int start_stage, int end_stage)
{
	unsigned short data;
	unsigned short mask;

	mask = ((1 << (end_stage + 1)) - 1) - ((1 << start_stage) - 1);

	ad714x->read(ad714x, STG_COM_INT_EN_REG, &data, 1);
	data |= 1 << end_stage;
	ad714x->write(ad714x, STG_COM_INT_EN_REG, data);

	ad714x->read(ad714x, STG_HIGH_INT_EN_REG, &data, 1);
	data &= ~mask;
	ad714x->write(ad714x, STG_HIGH_INT_EN_REG, data);
}

static void ad714x_use_thr_int(struct ad714x_chip *ad714x,
				int start_stage, int end_stage)
{
	unsigned short data;
	unsigned short mask;

	mask = ((1 << (end_stage + 1)) - 1) - ((1 << start_stage) - 1);

	ad714x->read(ad714x, STG_COM_INT_EN_REG, &data, 1);
	data &= ~(1 << end_stage);
	ad714x->write(ad714x, STG_COM_INT_EN_REG, data);

	ad714x->read(ad714x, STG_HIGH_INT_EN_REG, &data, 1);
	data |= mask;
	ad714x->write(ad714x, STG_HIGH_INT_EN_REG, data);
}

static void touchpad_cal_sensor_val(struct ad714x_chip *ad714x, int idx)
{
	struct ad714x_touchpad_plat *hw = &ad714x->hw->touchpad[idx];
	int i;

	ad714x->read(ad714x, CDC_RESULT_S0 + hw->x_start_stage,
			&ad714x->adc_reg[hw->x_start_stage],
			hw->x_end_stage - hw->x_start_stage + 1);

	for (i = hw->x_start_stage; i <= hw->x_end_stage; i++) {
		ad714x->read(ad714x, STAGE0_AMBIENT + i * PER_STAGE_REG_NUM,
				&ad714x->amb_reg[i], 1);
		if (ad714x->adc_reg[i] > ad714x->amb_reg[i])
			ad714x->sensor_val[i] =
				ad714x->adc_reg[i] - ad714x->amb_reg[i];
		else
			ad714x->sensor_val[i] = 0;
	}
}

static void touchpad_use_com_int(struct ad714x_chip *ad714x, int idx)
{
	struct ad714x_touchpad_plat *hw = &ad714x->hw->touchpad[idx];

	ad714x_use_com_int(ad714x, hw->x_start_stage, hw->x_end_stage);
}

static void touchpad_use_thr_int(struct ad714x_chip *ad714x, int idx)
{
	struct ad714x_touchpad_plat *hw = &ad714x->hw->touchpad[idx];

	ad714x_use_thr_int(ad714x, hw->x_start_stage, hw->x_end_stage);
	ad714x_use_thr_int(ad714x, hw->y_start_stage, hw->y_end_stage);
}

static void ad714x_touchpad_state_machine(struct ad714x_chip *ad714x, int idx)
{
	struct ad714x_touchpad_plat *hw = &ad714x->hw->touchpad[idx];
	static enum ad714x_device_state state = IDLE;
	unsigned short h_state, c_state;
	unsigned short mask;

	mask = ((1 << (hw->x_end_stage + 1)) - 1) - ((1 << hw->x_start_stage) - 1);

	h_state = ad714x->h_state & mask;
	c_state = ad714x->c_state & mask;

	switch (state) {
	case IDLE:
		if (h_state) {
			state = JITTER;
			/* In End of Conversion interrupt mode, the AD714X
			 * continuously generates hardware interrupts.
			 */
			touchpad_use_com_int(ad714x, idx);
			am_util_stdio_printf("ad714x_use_com_int %d\r\n",idx);
		}
		break;

	case JITTER:
		if (c_state == mask) {
			touchpad_cal_sensor_val(ad714x, idx);
			state = ACTIVE;
		}
		break;

	case ACTIVE:
		if (c_state == mask) {
			if (h_state) {
				touchpad_cal_sensor_val(ad714x, idx);
				for (char i = hw->x_start_stage; i <= hw->x_end_stage; i++) {
					am_util_stdio_printf("sensor_val[%d] , %d\r\n",
						i+ad714x->idx*12,ad714x->sensor_val[i]);
				}
			} else {
				/* When the user lifts off the sensor, configure
				 * the AD714X back to threshold interrupt mode.
				 */
				touchpad_use_thr_int(ad714x, idx);
				state = IDLE;
				am_util_stdio_printf("touchpad released %d\r\n",ad714x->idx);
			}
		}
		break;

	default:
		break;
	}
}

static void ad714x_hw_detect(struct ad714x_chip *ad714x)
{
	unsigned short data = 0x0000;

	ad714x->read(ad714x, AD714X_PARTID_REG, &data, 1);
	if( (data & 0xFFF0) == AD7147_PARTID ){
		ad714x->product = 0x7147;
		ad714x->version = data & 0xF;
		am_util_stdio_printf("moudle %d found AD7147(A) captouch, rev:%d\r\n",ad714x->idx,
				ad714x->version);
		ad714x->is_ad7417[ad714x->idx] = 1;
	}else{
		am_util_stdio_printf("moudle %d fail to detect AD714X captouch\r\n",ad714x->idx);
		ad714x->is_ad7417[ad714x->idx] = 0;
	}
}

static void ad714x_hw_init(struct ad714x_chip *ad714x)
{
	int i, j;
	unsigned short reg_base;
	unsigned short data[SYS_CFGREG_NUM];

	/* configuration CDC and interrupts */

	for (i = 0; i < STAGE_NUM; i++) {
		reg_base = AD714X_STAGECFG_REG + i * STAGE_CFGREG_NUM;
		for (j = 0; j < STAGE_CFGREG_NUM; j++)
			ad714x->write(ad714x, reg_base + j,
					ad714x->hw->stage_cfg_reg[i][j]);
	}

	for (i = 0; i < SYS_CFGREG_NUM; i++)
		ad714x->write(ad714x, AD714X_SYSCFG_REG + i,
			ad714x->hw->sys_cfg_reg[i]);
	for (i = 0; i < SYS_CFGREG_NUM; i++)
		ad714x->read(ad714x, AD714X_SYSCFG_REG + i, &data[i], 1);

	ad714x->write(ad714x, AD714X_STG_CAL_EN_REG, 0xFFF);

	/* clear all interrupts */
	ad714x->read(ad714x, STG_LOW_INT_STA_REG, &ad714x->l_state, 3);
}

void ad7147_init(struct ad714x_chip *ad714x)
{
	ad714x_hw_detect(ad714x);
	if(ad714x->is_ad7417[ad714x->idx]){
		ad714x_hw_init(ad714x);
	}
}

void ad714x_probe(struct ad714x_chip *ad714x, int idx)
{
	if(ad714x->is_ad7417[idx]){
		ad714x->idx = idx;
		ad714x->read(ad714x, STG_LOW_INT_STA_REG, &ad714x->l_state, 3);
		ad714x_touchpad_state_machine(ad714x, 0);
	}else{
		am_util_stdio_printf("moudle %s %d error\r\n", __func__,idx);
	}
}

void ad714x_disable(struct ad714x_chip *ad714x)
{
	unsigned short data;

	am_util_stdio_printf("%s enter\n", __func__);

	data = ad714x->hw->sys_cfg_reg[AD714X_PWR_CTRL] | 0x3;
	ad714x->write(ad714x, AD714X_PWR_CTRL, data);
}

void ad714x_enable(struct ad714x_chip *ad714x)
{
	am_util_stdio_printf("%s enter\n", __func__);
	/* resume to non-shutdown mode */

	ad714x->write(ad714x, AD714X_PWR_CTRL,
			ad714x->hw->sys_cfg_reg[AD714X_PWR_CTRL]);

	/* make sure the interrupt output line is not low level after resume,
	 * otherwise we will get no chance to enter falling-edge irq again
	 */

	ad714x->read(ad714x, STG_LOW_INT_STA_REG, &ad714x->l_state, 3);
}
