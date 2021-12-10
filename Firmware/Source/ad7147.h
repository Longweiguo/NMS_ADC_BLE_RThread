#ifndef __AD714X_H__
#define __AD714X_H__

#define STAGE_NUM              12
#define STAGE_CFGREG_NUM       8
#define SYS_CFGREG_NUM         8

struct ad714x_touchpad_plat {
	int x_start_stage;
	int x_end_stage;
	int x_max_coord;

	int y_start_stage;
	int y_end_stage;
	int y_max_coord;
};

struct ad714x_platform_data {
	int touchpad_num;
	struct ad714x_touchpad_plat *touchpad;
	unsigned short stage_cfg_reg[STAGE_NUM][STAGE_CFGREG_NUM];
	unsigned short sys_cfg_reg[SYS_CFGREG_NUM];
	unsigned long irqflags;
};

struct ad714x_chip;

typedef unsigned int (*ad714x_read_t)(struct ad714x_chip *, unsigned short, unsigned short *, unsigned char);
typedef unsigned int (*ad714x_write_t)(struct ad714x_chip *, unsigned short, unsigned short);

struct ad714x_chip {
	unsigned char idx;             //iom_moudle;
	unsigned short l_state;
	unsigned short h_state;
	unsigned short c_state;
	unsigned short adc_reg[STAGE_NUM];
	unsigned short amb_reg[STAGE_NUM];
	unsigned short sensor_val[STAGE_NUM];
	
	struct ad714x_platform_data *hw;
	
	int irq;
	ad714x_read_t read;
	ad714x_write_t write;

	unsigned int product;
	unsigned int version;
	unsigned char  is_ad7417[2];
};
void ad7147_init(struct ad714x_chip *ad714x);
void ad714x_probe(struct ad714x_chip *ad714x,int idx);
void ad714x_disable(struct ad714x_chip *ad714x);
void ad714x_enable(struct ad714x_chip *ad714x);

#endif
