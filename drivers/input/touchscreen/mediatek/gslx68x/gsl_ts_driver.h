#ifndef __GSL_TS_DRIVER_H__
#define __GSL_TS_DRIVER_H__

#define TPD_HAVE_BUTTON
#define GSL_ALG_ID
#define GSL_COMPATIBLE_CHIP
#define GSL_THREAD_EINT
#define TPD_PROC_DEBUG
//#define GSL_GESTURE
#define GSL_TIMER
//#define TPD_PROXIMITY

#define GSL9XX_VDDIO_1800 0

//#define TPD_POWER_VTP28_USE_VCAMA   //[add for mx2116 2015-11-03]

#define GSL_I2C_ADDR 0x40
#define GSL_PAGE_REG 0xf0
#define GSL_CLOCK_REG 0xe4
#define GSL_START_REG 0xe0
#define POWE_FAIL_REG 0xbc
#define TOUCH_INFO_REG 0x80
#define TPD_DEBUG_TIME 0x20130424

struct gsl_touch_info {
	int x[10];
	int y[10];
	int id[10];
	int finger_num;
};

/*button*/
#ifdef TPD_HAVE_BUTTON
#define TPD_KEY_COUNT 3
#define TPD_KEYS {KEY_MENU, KEY_HOMEPAGE, KEY_BACK}
#define TPD_KEYS_DIM {{80, 900, 20, 20}, {240, 900, 20, 20}, {400, 900, 20, 20}}
#endif

#ifdef GSL_ALG_ID
extern unsigned int gsl_mask_tiaoping(void);
extern unsigned int gsl_version_id(void);
extern void gsl_alg_id_main(struct gsl_touch_info *cinfo);
extern void gsl_DataInit(int *ret);
#endif

struct fw_data {
	unsigned char offset;
	unsigned int val;
};

#include "gslx680_c558_fw.h"

static unsigned char gsl_cfg_index = 0;

struct fw_config_type {
	const struct fw_data *fw;
	unsigned int fw_size;
	unsigned int *data_id;
	unsigned int data_size;
};

static struct fw_config_type gsl_cfg_table[] = {
	{
		GSLX680_C558_FW, (sizeof(GSLX680_C558_FW) / sizeof(struct fw_data)),
		gsl_config_data_id_c558, (sizeof(gsl_config_data_id_c558) / sizeof(unsigned int))
	}
};

#endif
