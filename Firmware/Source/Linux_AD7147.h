#ifndef __AD714X_H__
#define __AD714X_H__

#define STAGE_NUM              12
#define STAGE_CFGREG_NUM       8
#define SYS_CFGREG_NUM         8

/* board information which need be initialized in arch/mach... */
struct ad714x_slider_plat {
	int start_stage;
	int end_stage;
	int max_coord;
};

struct ad714x_wheel_plat {
	int start_stage;
	int end_stage;
	int max_coord;
};

struct ad714x_touchpad_plat {
	int x_start_stage;
	int x_end_stage;
	int x_max_coord;

	int y_start_stage;
	int y_end_stage;
	int y_max_coord;
};

struct ad714x_button_plat {
	int keycode;
	unsigned short l_mask;
	unsigned short h_mask;
};

struct ad714x_platform_data {
	int slider_num;
	int wheel_num;
	int touchpad_num;
	int button_num;
	struct ad714x_slider_plat *slider;
	struct ad714x_wheel_plat *wheel;
	struct ad714x_touchpad_plat *touchpad;
	struct ad714x_button_plat *button;
	unsigned short stage_cfg_reg[STAGE_NUM][STAGE_CFGREG_NUM];
	unsigned short sys_cfg_reg[SYS_CFGREG_NUM];
	unsigned long irqflags;
};

struct device;
struct ad714x_platform_data;
struct ad714x_driver_data;
struct ad714x_chip;

typedef int (*ad714x_read_t)(struct ad714x_chip *, unsigned short, unsigned short *, size_t);
typedef int (*ad714x_write_t)(struct ad714x_chip *, unsigned short, unsigned short);

struct ad714x_chip {
	unsigned short l_state;
	unsigned short h_state;
	unsigned short c_state;
	unsigned short adc_reg[STAGE_NUM];
	unsigned short amb_reg[STAGE_NUM];
	unsigned short sensor_val[STAGE_NUM];

	struct ad714x_platform_data *hw;
	struct ad714x_driver_data *sw;

	int irq;
	struct device *dev;
	ad714x_read_t read;
	ad714x_write_t write;

	unsigned int product;
	unsigned int version;
};

int ad714x_disable(struct ad714x_chip *ad714x);
int ad714x_enable(struct ad714x_chip *ad714x);
struct ad714x_chip *ad714x_probe(struct device *dev, unsigned short bus_type, int irq,
				 ad714x_read_t read, ad714x_write_t write);

#endif