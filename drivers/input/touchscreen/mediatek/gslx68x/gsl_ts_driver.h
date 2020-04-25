#ifndef __GSL_TS_DRIVER_H__
#define __GSL_TS_DRIVER_H__

#define GSL_ALG_ID
//#define TPD_PROC_DEBUG
#define GSL_TIMER

#define GSL9XX_VDDIO_1800 0

#define GSL_I2C_ADDR 0x40
#define GSL_PAGE_REG 0xf0
#define GSL_CLOCK_REG 0xe4
#define GSL_START_REG 0xe0
#define POWER_FAIL_REG 0xbc
#define TOUCH_INFO_REG 0x80
#define TPD_DEBUG_TIME 0x20130424

#define GSL_CHIP_NAME "gslx68x"
#define GSL_DEV_NAME "gsl1680"
#define MAX_CONTACTS 10

struct gsl_touch_info {
	int x[MAX_CONTACTS];
	int y[MAX_CONTACTS];
	int id[MAX_CONTACTS];
	unsigned char finger_num;
};

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
#define GSL_FIRMWARE GSLX680_C558_FW
#define GSL_CONFIG_DATA_ID gsl_config_data_id_c558

#endif
