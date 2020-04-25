/******************************************************************************
  Copyright (C), 2010-2012, Silead, Inc.
 ******************************************************************************
Filename      : gsl1680-d0.c
Version       : R2.0
Aurthor       : mark_huang
Creattime     : 2012.6.20
Description   : Driver for Silead I2C touchscreen.
 ******************************************************************************/
#include <linux/interrupt.h>
#include <linux/of_irq.h>
#include <linux/i2c.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/wait.h>
#include <linux/delay.h>

#include "tpd.h"
#include "mtk_boot_common.h"
#include "upmu_common.h"
#include "gsl_ts_driver.h"

#ifdef TPD_PROC_DEBUG
#include <linux/proc_fs.h>
#include <asm/uaccess.h>
#include <linux/seq_file.h>
#endif

extern struct tpd_device *tpd;

static struct i2c_client *gsl_client = NULL;
static int touch_irq = 0; //add by crystal
static const struct i2c_device_id gsl_device_id[] = {{"gsl_tp", 0}, {}};
static volatile int gsl_halt_flag = 0;
static struct mutex gsl_i2c_lock;
static const struct of_device_id tpd_of_match[] = {
	{.compatible = "mediatek,cap_touch"},
	{},
};
static int gsl_ps_enable = 0;
static DECLARE_WAIT_QUEUE_HEAD(waiter);
static struct task_struct *thread = NULL;
static int tpd_flag = 0;

#ifdef GSL_TIMER
#define GSL_TIMER_CHECK_CIRCLE 200
static struct delayed_work gsl_timer_check_work;
static struct workqueue_struct *gsl_timer_workqueue = NULL;
static char int_1st[4];
static char int_2nd[4];
static int i2c_lock_flag = 0;
#endif

#ifdef TPD_PROC_DEBUG
#define GSL_CONFIG_PROC_FILE "gsl_config"
#define CONFIG_LEN 31
static char gsl_read[CONFIG_LEN];
static unsigned char gsl_data_proc[8] = {0};
static unsigned char gsl_proc_flag = 0;
#endif

#define GSL_LOGE(fmt, ...) pr_err("[tp-gsl][Error][%s]" fmt, __func__, ##__VA_ARGS__)
#define GSL_LOGD(fmt, ...) pr_debug("[tp-gsl][Debug][%s]" fmt, __func__, ##__VA_ARGS__)

static int gsl_read_interface(struct i2c_client *client, unsigned char reg, unsigned char *buf, unsigned int num)
{
	int received = 0;
	int i;
	unsigned char temp = reg;

	mutex_lock(&gsl_i2c_lock);
	if (temp < 0x80) {
		temp = (temp + 8) & 0x5c;
		i2c_master_send(client, &temp, 1);
		received += i2c_master_recv(client, &buf[0], 4);
		temp = reg;
		i2c_master_send(client, &temp, 1);
		received += i2c_master_recv(client, &buf[0], 4);
	}
	for (i = 0; i < num; i++) {
		temp = reg + i;
		i2c_master_send(client, &temp, 1);
		if ((i + 8) < num)
			received += i2c_master_recv(client, (buf + i), 8);
		else
			received += i2c_master_recv(client, (buf + i), (num - i));
		i += 8;
	}
	mutex_unlock(&gsl_i2c_lock);

	return received;
}

static int gsl_write_interface(struct i2c_client *client, const unsigned char reg, unsigned char *buf, unsigned int num)
{
	struct i2c_msg xfer_msg = {0}; //modify crystal
	int ret;
	unsigned char tmp_buf[num + 1];

	tmp_buf[0] = reg;
	memcpy(tmp_buf + 1, buf, num);

	xfer_msg.addr = client->addr;
	xfer_msg.len = num + 1;
	xfer_msg.flags = 0; //client->flags & I2C_M_TEN;
	xfer_msg.buf = tmp_buf;
	mutex_lock(&gsl_i2c_lock);
	ret = i2c_transfer(client->adapter, &xfer_msg, 1);
	mutex_unlock(&gsl_i2c_lock);

	return ret;
}

static int gsl_test_i2c(struct i2c_client *client)
{
	int i, received;
	unsigned char buf[4] = {0};

	for (i = 0; i < 5; i++) {
		received = gsl_read_interface(client, 0xfc, buf, 4);
		if (received > 0) {
			GSL_LOGD("[tp-gsl] i2c read 0xfc = 0x%02x%02x%02x%02x\n",
			         buf[3], buf[2], buf[1], buf[0]);
			break;
		}
	}

	return (received > 0 ? 0 : -1);
}

#if GSL9XX_VDDIO_1800
static void gsl_io_control(struct i2c_client *client)
{
	unsigned char buf[4] = {0};
	int i;

	for (i = 0; i < 5; i++) {
		buf[0] = 0;
		buf[1] = 0;
		buf[2] = 0xfe;
		buf[3] = 0x1;
		gsl_write_interface(client, 0xf0, buf, 4);

		buf[0] = 0x5;
		buf[1] = 0;
		buf[2] = 0;
		buf[3] = 0x80;
		gsl_write_interface(client, 0x78, buf, 4);
		msleep(5);
	}

	msleep(50);
}
#endif

static void gsl_start_core(struct i2c_client *client)
{
	unsigned char buf[4] = {0};

	//add by sayid litianfeng 151125
	buf[3] = 0x01;
	buf[2] = 0xfe;
	buf[1] = 0x0e;
	buf[0] = 0x02;
	gsl_write_interface(client, 0xf0, buf, 4);

	buf[3] = 0x00;
	buf[2] = 0x0a;
	buf[1] = 0x00;
	buf[0] = 0x70;
	gsl_write_interface(client, 0x4, buf, 4);

	//add by sayid litianfeng 151125
	buf[0] = 0x01;
	gsl_write_interface(client, 0x88, buf, 4);

	buf[0] = 0x00;
	gsl_write_interface(client, 0xe0, buf, 4);

#ifdef GSL_ALG_ID
	gsl_DataInit(GSL_CONFIG_DATA_ID);
#endif
}

static void gsl_reset_core(struct i2c_client *client)
{
	unsigned char buf[4] = {0x00};

	buf[0] = 0x88;
	gsl_write_interface(client, 0xe0, buf, 4);
	msleep(5);

	buf[0] = 0x04;
	gsl_write_interface(client, 0xe4, buf, 4);
	msleep(5);

	buf[0] = 0x00;
	gsl_write_interface(client, 0xbc, buf, 4);
	msleep(5);

#if GSL9XX_VDDIO_1800
	gsl_io_control(client);
#endif
}

static void gsl_clear_reg(struct i2c_client *client)
{
	unsigned char buf[4] = {0};
	
	//clear reg
	buf[0] = 0x88;
	gsl_write_interface(client, 0xe0, buf, 4);
	msleep(20);

	buf[0] = 0x03;
	gsl_write_interface(client, 0x80, buf, 4);
	msleep(5);

	buf[0] = 0x04;
	gsl_write_interface(client, 0xe4, buf, 4);
	msleep(5);

	buf[0] = 0x00;
	gsl_write_interface(client, 0xe0, buf, 4);
	msleep(20);
	//clear reg
}

static void gsl_load_fw(struct i2c_client *client, const struct fw_data *GSL_DOWNLOAD_DATA, int data_len)
{
	unsigned char buf[4] = {0};
	unsigned char addr = 0;
	unsigned int source_line = 0;
	unsigned int source_len = data_len;//ARRAY_SIZE(GSL_DOWNLOAD_DATA);

	GSL_LOGD("=============gsl_load_fw start==============\n");

	for (source_line = 0; source_line < source_len; source_line++) {
		/* init page trans, set the page val */
		addr = (unsigned char)GSL_DOWNLOAD_DATA[source_line].offset;
		memcpy(buf, &GSL_DOWNLOAD_DATA[source_line].val, 4);
		gsl_write_interface(client, addr, buf, 4);
	}

	GSL_LOGD("=============gsl_load_fw end==============\n");
}

static void gsl_sw_init(struct i2c_client *client)
{
	int temp;
	char write_buf[4] ;
	char read_buf[4] = {0};
	static volatile int gsl_sw_flag = 0;
	if (1 == gsl_sw_flag)
		return;
	gsl_sw_flag = 1;

	tpd_gpio_output(GTP_RST_PORT, 0);
	msleep(20);
	tpd_gpio_output(GTP_RST_PORT, 1);
	msleep(20);

	temp = gsl_test_i2c(client);
	if (temp < 0) {
		gsl_sw_flag = 0;
		return;
	}

	gsl_clear_reg(client);
	gsl_reset_core(client);

#if GSL9XX_VDDIO_1800
	gsl_io_control(client);
#endif

	gsl_load_fw(client, GSL_FIRMWARE, ARRAY_SIZE(GSL_FIRMWARE));

#if GSL9XX_VDDIO_1800
	gsl_io_control(client);
#endif

	gsl_start_core(client);
	gsl_sw_flag = 0;

	write_buf[3] = 0;
	write_buf[2] = 0;
	write_buf[1] = 0;
	write_buf[0] = 0xb6;
	gsl_write_interface(client, 0xf0, write_buf, 4);
	gsl_read_interface(client, 0x7c, read_buf, 4);
	GSL_LOGD("[gsl1680][%s] addr = (b6,7c); read_buf = %02x%02x%02x%02x\n",
	         __func__, read_buf[3], read_buf[2], read_buf[1], read_buf[0]);
}

static void check_mem_data(struct i2c_client *client)
{
	char read_buf[4] = {0};
	gsl_read_interface(client, 0xb0, read_buf, 4);

	GSL_LOGD("[gsl1680][%s] addr = 0xb0; read_buf = %02x%02x%02x%02x\n",
	         __func__, read_buf[3], read_buf[2], read_buf[1], read_buf[0]);

	if (read_buf[3] != 0x5a || read_buf[2] != 0x5a || read_buf[1] != 0x5a || read_buf[0] != 0x5a) {
		msleep(10);
		gsl_sw_init(client);
	}
}

#ifdef TPD_PROC_DEBUG
#define GSL_APPLICATION
#ifdef GSL_APPLICATION
static int gsl_read_MorePage(struct i2c_client *client, unsigned int addr, unsigned char *buf, unsigned int num)
{
	int i;
	unsigned char tmp_buf[4] = {0};
	unsigned char tmp_addr;
	for (i = 0; i < num / 8; i++) {
		tmp_buf[0] = (char)((addr + i * 8) / 0x80);
		tmp_buf[1] = (char)(((addr + i * 8) / 0x80) >> 8);
		tmp_buf[2] = (char)(((addr + i * 8) / 0x80) >> 16);
		tmp_buf[3] = (char)(((addr + i * 8) / 0x80) >> 24);
		gsl_write_interface(client, 0xf0, tmp_buf, 4);
		tmp_addr = (char)((addr + i * 8) % 0x80);
		gsl_read_interface(client, tmp_addr, (buf + i * 8), 8);
	}
	if (i * 8 < num) {
		tmp_buf[0] = (char)((addr + i * 8) / 0x80);
		tmp_buf[1] = (char)(((addr + i * 8) / 0x80) >> 8);
		tmp_buf[2] = (char)(((addr + i * 8) / 0x80) >> 16);
		tmp_buf[3] = (char)(((addr + i * 8) / 0x80) >> 24);
		gsl_write_interface(client, 0xf0, tmp_buf, 4);
		tmp_addr = (char)((addr + i * 8) % 0x80);
		gsl_read_interface(client, tmp_addr, (buf + i * 8), 4);
	}
	return 0;//modify crystal
}
#endif

static int char_to_int(char ch)
{
	if (ch >= '0' && ch <= '9')
		return (ch - '0');
	else
		return (ch - 'a' + 10);
}

static int gsl_config_read_proc(struct seq_file *m, void *v)
{
	char temp_data[5] = {0};
	//int i;
	unsigned int tmp = 0;
	if ('v' == gsl_read[0] && 's' == gsl_read[1]) {
#ifdef GSL_ALG_ID
		tmp = gsl_version_id();
#else
		tmp = 0x20121215;
#endif
		seq_printf(m, "version:%x\n", tmp);
	} else if ('r' == gsl_read[0] && 'e' == gsl_read[1]) {
		if ('i' == gsl_read[3]) {
#ifdef GSL_ALG_ID
			tmp = (gsl_data_proc[5] << 8) | gsl_data_proc[4];
			seq_printf(m, "gsl_config_data_id[%d] = ", tmp);
			if (tmp >= 0 && tmp < ARRAY_SIZE(GSL_CONFIG_DATA_ID))
				seq_printf(m, "%d\n", GSL_CONFIG_DATA_ID[tmp]);
#endif
		} else {
			gsl_write_interface(gsl_client, 0xf0, &gsl_data_proc[4], 4);
			gsl_read_interface(gsl_client, gsl_data_proc[0], temp_data, 4);
			seq_printf(m, "offset : {0x%02x,0x", gsl_data_proc[0]);
			seq_printf(m, "%02x", temp_data[3]);
			seq_printf(m, "%02x", temp_data[2]);
			seq_printf(m, "%02x", temp_data[1]);
			seq_printf(m, "%02x};\n", temp_data[0]);
		}
	}
#ifdef GSL_APPLICATION
	else if ('a' == gsl_read[0] && 'p' == gsl_read[1]) {
		char *buf;
		int temp1;
		tmp = (unsigned int)(((gsl_data_proc[2] << 8) | gsl_data_proc[1]) & 0xffff);
		buf = kzalloc(tmp, GFP_KERNEL);
		if (buf == NULL)
			return -1;
		if (3 == gsl_data_proc[0]) {
			gsl_read_interface(gsl_client, gsl_data_proc[3], buf, tmp);
			if (tmp < m->size) {
				memcpy(m->buf, buf, tmp);
			}
		} else if (4 == gsl_data_proc[0]) {
			temp1 = ((gsl_data_proc[6] << 24) | (gsl_data_proc[5] << 16) |
			         (gsl_data_proc[4] << 8) | gsl_data_proc[3]);
			gsl_read_MorePage(gsl_client, temp1, buf, tmp);
			if (tmp < m->size) {
				memcpy(m->buf, buf, tmp);
			}
		}
		kfree(buf);
	}
#endif
	return 0;
}

static ssize_t  gsl_config_write_proc(struct file *file, const char __user  *buffer, size_t  count, loff_t *data)
{
	unsigned char buf[8] = {0};
	char temp_buf[CONFIG_LEN];
	char *path_buf;
	int tmp = 0;
	int tmp1 = 0;
	GSL_LOGD("[tp-gsl][%s] \n", __func__);
	if (count > 512) {
//		GSL_LOGD("size not match [%d:%ld]\n", CONFIG_LEN, count);
		return -EFAULT;
	}
	path_buf = kzalloc(count, GFP_KERNEL);
	if (!path_buf) {
		GSL_LOGE("alloc path_buf memory error \n");
		return -1;
	}
	if (copy_from_user(path_buf, buffer, count)) {
		GSL_LOGE("copy from user fail\n");
		goto exit_write_proc_out;
	}
	memcpy(temp_buf, path_buf, (count < CONFIG_LEN ? count : CONFIG_LEN));
	GSL_LOGD("[tp-gsl][%s][%s]\n", __func__, temp_buf);
#ifdef GSL_APPLICATION
	if ('a' != temp_buf[0] || 'p' != temp_buf[1]) {
#endif
		buf[3] = char_to_int(temp_buf[14]) << 4 | char_to_int(temp_buf[15]);
		buf[2] = char_to_int(temp_buf[16]) << 4 | char_to_int(temp_buf[17]);
		buf[1] = char_to_int(temp_buf[18]) << 4 | char_to_int(temp_buf[19]);
		buf[0] = char_to_int(temp_buf[20]) << 4 | char_to_int(temp_buf[21]);

		buf[7] = char_to_int(temp_buf[5]) << 4 | char_to_int(temp_buf[6]);
		buf[6] = char_to_int(temp_buf[7]) << 4 | char_to_int(temp_buf[8]);
		buf[5] = char_to_int(temp_buf[9]) << 4 | char_to_int(temp_buf[10]);
		buf[4] = char_to_int(temp_buf[11]) << 4 | char_to_int(temp_buf[12]);
#ifdef GSL_APPLICATION
	}
#endif
	if ('v' == temp_buf[0] && 's' == temp_buf[1]) { //version //vs
		memcpy(gsl_read, temp_buf, 4);
		GSL_LOGD("gsl version\n");
	} else if ('s' == temp_buf[0] && 't' == temp_buf[1]) { //start //st
		gsl_proc_flag = 1;
		gsl_reset_core(gsl_client);
	} else if ('e' == temp_buf[0] && 'n' == temp_buf[1]) { //end //en
		msleep(20);
		gsl_reset_core(gsl_client);
		gsl_start_core(gsl_client);
		gsl_proc_flag = 0;
	} else if ('r' == temp_buf[0] && 'e' == temp_buf[1]) { //read buf //
		memcpy(gsl_read, temp_buf, 4);
		memcpy(gsl_data_proc, buf, 8);
	} else if ('w' == temp_buf[0] && 'r' == temp_buf[1]) { //write buf
		gsl_write_interface(gsl_client, buf[4], buf, 4);
	}

#ifdef GSL_ALG_ID
	else if ('i' == temp_buf[0] && 'd' == temp_buf[1]) { //write id config //
		tmp1 = (buf[7] << 24) | (buf[6] << 16) | (buf[5] << 8) | buf[4];
		tmp = (buf[3] << 24) | (buf[2] << 16) | (buf[1] << 8) | buf[0];
		if (tmp1 >= 0 && tmp1 < ARRAY_SIZE(GSL_CONFIG_DATA_ID)) {
			GSL_CONFIG_DATA_ID[tmp1] = tmp;
		}
	}
#endif
#ifdef GSL_APPLICATION
	else if ('a' == temp_buf[0] && 'p' == temp_buf[1]) {
		if (1 == path_buf[3]) {
			tmp = ((path_buf[5] << 8) | path_buf[4]);
			gsl_write_interface(gsl_client, path_buf[6], &path_buf[10], tmp);
		} else if (2 == path_buf[3]) {
			tmp = ((path_buf[5] << 8) | path_buf[4]);
			tmp1 = ((path_buf[9] << 24) | (path_buf[8] << 16) | (path_buf[7] << 8)
			        | path_buf[6]);
			buf[0] = (char)((tmp1 / 0x80) & 0xff);
			buf[1] = (char)(((tmp1 / 0x80) >> 8) & 0xff);
			buf[2] = (char)(((tmp1 / 0x80) >> 16) & 0xff);
			buf[3] = (char)(((tmp1 / 0x80) >> 24) & 0xff);
			buf[4] = (char)(tmp1 % 0x80);
			gsl_write_interface(gsl_client, 0xf0, buf, 4);
			gsl_write_interface(gsl_client, buf[4], &path_buf[10], tmp);
		} else if (3 == path_buf[3] || 4 == path_buf[3]) {
			memcpy(gsl_read, temp_buf, 4);
			memcpy(gsl_data_proc, &path_buf[3], 7);
		}
	}
#endif
exit_write_proc_out:
	kfree(path_buf);
	return count;
}
#endif

#ifdef GSL_TIMER
static void gsl_timer_check_func(struct work_struct *work)
{
	//static int i2c_lock_flag = 0;
	char read_buf[4]  = {0};
	char init_chip_flag = 0;
	int i, flag;
	GSL_LOGD("----------------gsl_monitor_worker-----------------\n");
	if (i2c_lock_flag != 0)
		return;
	else
		i2c_lock_flag = 1;
	gsl_read_interface(gsl_client, 0xb4, read_buf, 4);
	memcpy(int_2nd, int_1st, 4);
	memcpy(int_1st, read_buf, 4);

	if (int_1st[3] == int_2nd[3] && int_1st[2] == int_2nd[2] &&
	    int_1st[1] == int_2nd[1] && int_1st[0] == int_2nd[0]) {
		GSL_LOGD("======int_1st: %x %x %x %x , int_2nd: %x %x %x %x ======\n",
		         int_1st[3], int_1st[2], int_1st[1], int_1st[0],
		         int_2nd[3], int_2nd[2], int_2nd[1], int_2nd[0]);
		memset(read_buf, 0xff, sizeof(read_buf));

		msleep(20);
		gsl_read_interface(gsl_client, 0xb4, read_buf, 4);
		if (int_1st[0] == read_buf[0] && int_1st[1] == read_buf[1] &&
		    int_1st[2] == read_buf[2] && int_1st[3] == read_buf[3]) {
			init_chip_flag = 1;
			goto queue_monitor_work;
		}
	}
	/*check 0xb0 register,check firmware if ok*/
	for (i = 0; i < 5; i++) {
		gsl_read_interface(gsl_client, 0xb0, read_buf, 4);
		if (read_buf[3] != 0x5a || read_buf[2] != 0x5a ||
		    read_buf[1] != 0x5a || read_buf[0] != 0x5a) {
			GSL_LOGD("gsl_monitor_worker 0xb0 = {0x%02x%02x%02x%02x};\n",
			         read_buf[3], read_buf[2], read_buf[1], read_buf[0]);
			flag = 1;
		} else {
			flag = 0;
			break;
		}

	}
	if (flag == 1) {
		init_chip_flag = 1;
		goto queue_monitor_work;
	}

	/*check 0xbc register,check dac if normal*/
	for (i = 0; i < 5; i++) {
		gsl_read_interface(gsl_client, 0xbc, read_buf, 4);
		if (read_buf[3] != 0 || read_buf[2] != 0 ||
		    read_buf[1] != 0 || read_buf[0] != 0) {
			flag = 1;
		} else {
			flag = 0;
			break;
		}
	}
	if (flag == 1) {
		//gsl_reset_core(gsl_client);
		//gsl_start_core(gsl_client);
		init_chip_flag = 1;
		goto queue_monitor_work;
	}
queue_monitor_work:
	if (init_chip_flag) {
		gsl_sw_init(gsl_client);
		memset(int_1st, 0xff, sizeof(int_1st));
	}

	if (gsl_halt_flag == 0) {
		queue_delayed_work(gsl_timer_workqueue, &gsl_timer_check_work, 200);
	}
	i2c_lock_flag = 0;

}
#endif

static void gsl_report_point(struct gsl_touch_info *ti)
{
	int tmp = 0;
	static int gsl_up_flag = 0; //prevent more up event
	GSL_LOGD("gsl_report_point %d \n", ti->finger_num);

	if (unlikely(ti->finger_num == 0)) {
		if (gsl_up_flag == 0)
			return;
		gsl_up_flag = 0;
		input_report_key(tpd->dev, BTN_TOUCH, 0);
		input_mt_sync(tpd->dev);
		if (FACTORY_BOOT == get_boot_mode() ||
		    RECOVERY_BOOT == get_boot_mode()) {
			tpd_button(ti->x[tmp], ti->y[tmp], 0);
		}
	} else {
		gsl_up_flag = 1;
		for (tmp = 0; ti->finger_num > tmp; tmp++) {
			GSL_LOGD("[lifan_gsl1680](x[%d],y[%d]) = (%d,%d);\n",
			         ti->id[tmp], ti->id[tmp], ti->x[tmp], ti->y[tmp]);
			input_report_key(tpd->dev, BTN_TOUCH, 1);
			input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, 1);

			if (FACTORY_BOOT == get_boot_mode() || RECOVERY_BOOT == get_boot_mode()) {
				tpd_button(ti->x[tmp], ti->y[tmp], 1);
			}
			input_report_abs(tpd->dev, ABS_MT_TRACKING_ID, ti->id[tmp] - 1);
			input_report_abs(tpd->dev, ABS_MT_POSITION_X, ti->x[tmp]);
			input_report_abs(tpd->dev, ABS_MT_POSITION_Y, ti->y[tmp]);

			input_mt_sync(tpd->dev);
		}
	}
	input_sync(tpd->dev);
}

static void gsl_report_work(void)
{
	unsigned char buf[4] = {0};
	//unsigned char i = 0;
	//unsigned short ret = 0;
	unsigned short tmp = 0;
	int tmp1 = 0; //modify
	struct gsl_touch_info cinfo = {{0}};
	unsigned char tmp_buf[44] = {0};

	GSL_LOGD("enter gsl_report_work\n");

#ifdef GSL_TIMER
	if (i2c_lock_flag != 0)
		goto i2c_lock_schedule;
	else
		i2c_lock_flag = 1;
#endif

#ifdef TPD_PROC_DEBUG
	if (gsl_proc_flag == 1)
		return;
#endif

	gsl_read_interface(gsl_client, 0x80, tmp_buf, 8);
	if (tmp_buf[0] >= 2 && tmp_buf[0] <= 10)
		gsl_read_interface(gsl_client, 0x88, &tmp_buf[8], (tmp_buf[0] * 4 - 4));
	cinfo.finger_num = tmp_buf[0] & 0x0f;

	GSL_LOGD("tp-gsl  finger_num = %d\n", cinfo.finger_num);
	for (tmp = 0; tmp < (cinfo.finger_num > 10 ? 10 : cinfo.finger_num); tmp++) {
		cinfo.id[tmp] = tmp_buf[tmp * 4 + 7] >> 4;
		cinfo.y[tmp] = (tmp_buf[tmp * 4 + 4] | ((tmp_buf[tmp * 4 + 5]) << 8));
		cinfo.x[tmp] = (tmp_buf[tmp * 4 + 6] | ((tmp_buf[tmp * 4 + 7] & 0x0f) << 8));
		GSL_LOGD("tp-gsl  x = %d y = %d \n", cinfo.x[tmp], cinfo.y[tmp]);
	}

#ifdef GSL_ALG_ID
	//int tmp1 = 0;
	cinfo.finger_num = (tmp_buf[3] << 24) | (tmp_buf[2] << 16) | (tmp_buf[1] << 8) | (tmp_buf[0]);
	gsl_alg_id_main(&cinfo);

	tmp1 = gsl_mask_tiaoping();
	GSL_LOGD("[tp-gsl] tmp1=%x\n", tmp1);
	if (tmp1 > 0 && tmp1 < 0xffffffff) {
		buf[0] = 0xa;
		buf[1] = 0;
		buf[2] = 0;
		buf[3] = 0;
		gsl_write_interface(gsl_client, 0xf0, buf, 4);
		buf[0] = (unsigned char)(tmp1 & 0xff);
		buf[1] = (unsigned char)((tmp1 >> 8) & 0xff);
		buf[2] = (unsigned char)((tmp1 >> 16) & 0xff);
		buf[3] = (unsigned char)((tmp1 >> 24) & 0xff);
		GSL_LOGD("tmp1=%08x,buf[0]=%02x,buf[1]=%02x,buf[2]=%02x,buf[3]=%02x\n",
		         tmp1, buf[0], buf[1], buf[2], buf[3]);
		gsl_write_interface(gsl_client, 0x8, buf, 4);
	}
#endif

	gsl_report_point(&cinfo);

#ifdef GSL_TIMER
i2c_lock_schedule:
	i2c_lock_flag = 0;

#endif
}

static int touch_event_handler(void *unused)
{

	struct sched_param param = { .sched_priority = 4 };
	sched_setscheduler(current, SCHED_RR, &param);
	GSL_LOGD("[tpd-gsl][%s] 6666666666666666666\n", __func__);
	do {
		GSL_LOGD("[tpd-gsl][%s] 7777777777777777777777777\n", __func__);
		enable_irq(touch_irq);
		set_current_state(TASK_INTERRUPTIBLE);
		wait_event_interruptible(waiter, tpd_flag != 0);
		//disable_irq(touch_irq);
		tpd_flag = 0;
		TPD_DEBUG_SET_TIME;
		set_current_state(TASK_RUNNING);
		gsl_report_work();
	} while (!kthread_should_stop());
	return 0;
}

static irqreturn_t tpd_eint_interrupt_handler(unsigned irq, struct irq_desc *desc)
{
	//TPD_DEBUG_PRINT_INT;

	tpd_flag = 1;
	/* enter EINT handler disable INT, make sure INT is disable when handle touch event including top/bottom half */
	/* use _nosync to avoid deadlock */
	GSL_LOGD("[tpd-gsl][%s][%d] cccccccccccccccccccccccccccccc\n", __func__, __LINE__);
	disable_irq_nosync(touch_irq);
	wake_up_interruptible(&waiter);
	return IRQ_HANDLED;
}

static void gsl_hw_init(void)
{
	int ret;

	ret = regulator_set_voltage(tpd->reg, 2800000, 2800000);  /* set 2.8v */
	if (ret)
		GSL_LOGE("regulator_set_voltage() failed!\n");

	ret = regulator_enable(tpd->reg); /* enable regulator */
	if (ret)
		GSL_LOGE("regulator_enable() failed!\n");

	tpd_gpio_output(GTP_RST_PORT, 0);
	tpd_gpio_output(GTP_RST_PORT, 1);
	msleep(100);
	tpd_gpio_as_int(GTP_INT_PORT);
	msleep(100);
}

//modify by crystal
#ifdef TPD_PROC_DEBUG
static int gsl_server_list_open(struct inode *inode, struct file *file)
{
	return single_open(file, gsl_config_read_proc, NULL);
}

static const struct file_operations gsl_seq_fops = {
	.open = gsl_server_list_open,
	.read = seq_read,
	.release = single_release,
	.write = gsl_config_write_proc,
	.owner = THIS_MODULE,
};
#endif

static int gsl_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret;
	struct device_node *node = NULL;
	//unsigned char tp_data[4];

	GSL_LOGD();

	input_set_abs_params(tpd->dev, ABS_MT_TRACKING_ID, 0, (MAX_CONTACTS + 1), 0, 0); //add by sky [2015.11.30]

	mutex_init(&gsl_i2c_lock);
	gsl_client = client;
	GSL_LOGD("gsl_client->addr = 0x%02x\n", gsl_client->addr);
	if (gsl_client->addr != GSL_I2C_ADDR) {
		GSL_LOGD("setting gsl_client->addr to 0x%02x\n", GSL_I2C_ADDR);
		gsl_client->addr = GSL_I2C_ADDR;
	}
	gsl_hw_init();
	//disable_irq(touch_irq);

	ret = gsl_test_i2c(gsl_client);
	if (ret < 0)
		return ret;

	gsl_sw_init(gsl_client);
	msleep(20);
	check_mem_data(gsl_client);


	GSL_LOGD("[tpd-gsl][%s] 111111111111111\n", __func__);
	thread = kthread_run(touch_event_handler, 0, TPD_DEVICE);
	if (IS_ERR(thread)) {
		GSL_LOGD("[tpd-gsl][%s] 222222222222222\n", __func__);
		//ret = PTR_ERR(thread);
		//TPD_DMESG(TPD_DEVICE " failed to create kernel thread: %d\n", PTR_ERR(thread));
	}
	GSL_LOGD("[tpd-gsl][%s] 3333333333333333\n", __func__);

	node = of_find_matching_node(NULL, touch_of_match);
	GSL_LOGD("[tpd-gsl][%s] 4444444444444444\n", __func__);
	//node = of_find_compatible_node(NULL, NULL, "mediatek,TOUCH-eint");
	//node = of_find_compatible_node(NULL, NULL, "mediatek,cap_touch");
	if (node) {
		GSL_LOGD("[tpd-gsl][%s] 44444444444444445555555555555555\n", __func__);
		touch_irq = irq_of_parse_and_map(node, 0);
		ret = request_irq(touch_irq, (irq_handler_t) tpd_eint_interrupt_handler,
		                  IRQF_TRIGGER_RISING, "touch-eint", NULL);
		if (ret > 0) {
			ret = -1;
			GSL_LOGD("tpd request_irq IRQ LINE NOT AVAILABLE!.");

			/* mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM); */
//				enable_irq(touch_irq);
		}
	}

#ifdef GSL_TIMER
	INIT_DELAYED_WORK(&gsl_timer_check_work, gsl_timer_check_func);
	gsl_timer_workqueue = create_workqueue("gsl_timer_check");

	queue_delayed_work(gsl_timer_workqueue, &gsl_timer_check_work, GSL_TIMER_CHECK_CIRCLE);

#endif
	GSL_LOGD("[tpd-gsl][%s] 555555555555555555\n", __func__);

#ifdef TPD_PROC_DEBUG
	proc_create(GSL_CONFIG_PROC_FILE, 0666, NULL, &gsl_seq_fops);
	gsl_proc_flag = 0;
#endif

#ifdef GSL_ALG_ID
	gsl_DataInit(GSL_CONFIG_DATA_ID);
#endif

	input_set_abs_params(tpd->dev, ABS_MT_TRACKING_ID, 0, 5, 0, 0);

	GSL_LOGD("[tpd-gsl][%s][%d] bbbbbbbbbbbbbbbbbbbbbbbbbbbb\n", __func__, __LINE__);
	disable_irq(touch_irq);
	GSL_LOGD("[tpd-gsl][%s][%d] dddddddddddddddddddddddddddd\n", __func__, __LINE__);
	enable_irq(touch_irq);

	tpd_load_status = 1;

	return ret;
}

/*****************************************************************************
Prototype    : gsl_remove
Description  : remove gsl1680 driver
Input        : struct i2c_client *client
Output       : int
Return Value : static
 *****************************************************************************/
static int  gsl_remove(struct i2c_client *client)
{
	GSL_LOGD("[gsl1680] TPD removed\n");
	return 0;
}

/*****************************************************************************
Prototype    : gsl_detect
Description  : gsl1680 driver local setup without board file
Input        : struct i2c_client *client
int kind
struct i2c_board_info *info
Output       : int
Return Value : static
 *****************************************************************************/
static int gsl_detect (struct i2c_client *client, struct i2c_board_info *info)
{
	//int error;

	GSL_LOGD("%s, %d\n", __FUNCTION__, __LINE__);
	strcpy(info->type, TPD_DEVICE);
	return 0;
}

static struct i2c_driver gsl_i2c_driver = {
	.driver = {
		.name = TPD_DEVICE,
		.owner = THIS_MODULE,
		.of_match_table = tpd_of_match,
	},
	.probe = gsl_probe,
	.remove = gsl_remove,
	.id_table = gsl_device_id,
	.detect = gsl_detect,
};

/*****************************************************************************
Prototype    : gsl_local_init
Description  : setup gsl1680 driver
Input        : None
Output       : None
Return Value : static
 *****************************************************************************/
static int gsl_local_init(void)
{
	int ret;
	GSL_LOGD();
	GSL_LOGD("[wsl] gsl_local_init start \n");

	GSL_LOGD("[wsl] regulator_get start \n");
	tpd->reg = regulator_get(tpd->tpd_dev, "vtouch");
	if (IS_ERR(tpd->reg))
		GSL_LOGE("tpd->reg error\n");
	GSL_LOGD("[wsl] regulator_get end \n");

	input_set_abs_params(tpd->dev, ABS_MT_TRACKING_ID, 0, 10, 0, 0);
	if (tpd_dts_data.use_tpd_button) {
		/*initialize tpd button data */
		tpd_button_setting(tpd_dts_data.tpd_key_num,
		                   tpd_dts_data.tpd_key_local,
		                   tpd_dts_data.tpd_key_dim_local);
	}

	ret = i2c_add_driver(&gsl_i2c_driver);

	if (ret < 0) {
		GSL_LOGD("unable to i2c_add_driver\n");
		return -ENODEV;
	}

	if (tpd_load_status == 0) {
		GSL_LOGD("tpd_load_status == 0, gsl_probe failed\n");
		i2c_del_driver(&gsl_i2c_driver);
		return -ENODEV;
	}

	/* define in tpd_debug.h */
	tpd_type_cap = 1;
	GSL_LOGD("end %s, %d\n", __FUNCTION__, __LINE__);
	return 0;
}

static void gsl_suspend(struct device *h)
{
	int tmp;
	GSL_LOGD();
	GSL_LOGD("gsl_suspend11111111111111111111\n");
	GSL_LOGD("gsl_suspend:gsl_ps_enable = %d\n", gsl_ps_enable);

#ifdef GSL_TIMER
	cancel_delayed_work_sync(&gsl_timer_check_work);
	i2c_lock_flag = 0;
#endif

	gsl_halt_flag = 1;
	//version info
	GSL_LOGD("[tp-gsl]the last time of debug:%x\n", TPD_DEBUG_TIME);
#ifdef GSL_ALG_ID
	tmp = gsl_version_id();
	GSL_LOGD("[tp-gsl]the version of alg_id:%x\n", tmp);
#endif

	//version info
#ifdef TPD_PROC_DEBUG
	if (gsl_proc_flag == 1)
		return;
#endif

	GSL_LOGD("[tpd-gsl][%s][%d] cccccccccccccccccccccccccccccc\n", __func__, __LINE__);
	disable_irq(touch_irq);
//	gsl_reset_core(gsl_client);
//	msleep(20);
	tpd_gpio_output(GTP_RST_PORT, 0);
}

static void gsl_resume(struct device *h)
{
	GSL_LOGD();

#ifdef TPD_PROC_DEBUG
	if (gsl_proc_flag == 1)
		return;
#endif

	tpd_gpio_output(GTP_RST_PORT, 1);
	msleep(20);
	gsl_reset_core(gsl_client);

	gsl_start_core(gsl_client);
	check_mem_data(gsl_client);
	GSL_LOGD("[tpd-gsl][%s][%d] aaaaaaaaaaaaaaaaaaaaaaaaaa\n", __func__, __LINE__);
	enable_irq(touch_irq);

#ifdef GSL_TIMER
	queue_delayed_work(gsl_timer_workqueue, &gsl_timer_check_work, GSL_TIMER_CHECK_CIRCLE);
#endif

	gsl_halt_flag = 0;
}

static struct tpd_driver_t gsl_driver = {
	.tpd_device_name = GSL_DEV_NAME,
	.tpd_local_init = gsl_local_init,
	.suspend = gsl_suspend,
	.resume = gsl_resume,
	.tpd_have_button = 0
};

static int __init gsl_driver_init(void)
{
	int ret;

	GSL_LOGD();

	tpd_get_dts_info();
	ret = tpd_driver_add(&gsl_driver);
	if (ret < 0)
		GSL_LOGE("tpd_driver_add failed! Code: %d\n", ret);

	return ret;
}

static void __exit gsl_driver_exit(void)
{
	int ret;

	GSL_LOGD();

	ret = tpd_driver_remove(&gsl_driver);
	if (ret < 0)
		GSL_LOGE("tpd_driver_remove failed! Code: %d\n", ret);
}

module_init(gsl_driver_init);
module_exit(gsl_driver_exit);
