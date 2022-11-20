/*
 *
 * FocalTech ftxxxx TouchScreen driver.
 *
 * Copyright (c) 2012-2019, Focaltech Ltd. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

/*****************************************************************************
*
* File Name: focaltech_ex_mode.c
*
* Author: Focaltech Driver Team
*
* Created: 2016-08-31
*
* Abstract:
*
* Reference:
*
*****************************************************************************/

/*****************************************************************************
* 1.Included header files
*****************************************************************************/
#include "focaltech_core.h"

/*****************************************************************************
* 2.Private constant and macro definitions using #define
*****************************************************************************/

/*****************************************************************************
* 3.Private enumerations, structures and unions using typedef
*****************************************************************************/
//prize-huangjiwu-for  glover mode --begin
#if !defined(CONFIG_PRIZE_SMART_COVER_COMMON_NODE)
enum _ex_mode {
    MODE_GLOVE = 0,
    MODE_COVER,
    MODE_CHARGER,
};

/*****************************************************************************
* 4.Static variables
*****************************************************************************/

/*****************************************************************************
* 5.Global variable or extern global variabls/functions
*****************************************************************************/

/*****************************************************************************
* 6.Static function prototypes
*******************************************************************************/
static int fts_ex_mode_switch(enum _ex_mode mode, u8 value)
{
    int ret = 0;
    u8 m_val = 0;

    if (value)
        m_val = 0x01;
    else
        m_val = 0x00;

    switch (mode) {
    case MODE_GLOVE:
        ret = fts_write_reg(FTS_REG_GLOVE_MODE_EN, m_val);
        if (ret < 0) {
            FTS_ERROR("MODE_GLOVE switch to %d fail", m_val);
        }
        break;
    case MODE_COVER:
        ret = fts_write_reg(FTS_REG_COVER_MODE_EN, m_val);
        if (ret < 0) {
            FTS_ERROR("MODE_COVER switch to %d fail", m_val);
        }
        break;
    case MODE_CHARGER:
        ret = fts_write_reg(FTS_REG_CHARGER_MODE_EN, m_val);
        if (ret < 0) {
            FTS_ERROR("MODE_CHARGER switch to %d fail", m_val);
        }
        break;
    default:
        FTS_ERROR("mode(%d) unsupport", mode);
        ret = -EINVAL;
        break;
    }

    return ret;
}

static ssize_t fts_glove_mode_show(
    struct device *dev, struct device_attribute *attr, char *buf)
{
    int count = 0;
    u8 val = 0;
    struct fts_ts_data *ts_data = fts_data;
    struct input_dev *input_dev = ts_data->input_dev;

    mutex_lock(&input_dev->mutex);
    fts_read_reg(FTS_REG_GLOVE_MODE_EN, &val);
    count = snprintf(buf + count, PAGE_SIZE, "Glove Mode:%s\n",
                     ts_data->glove_mode ? "On" : "Off");
    count += snprintf(buf + count, PAGE_SIZE, "Glove Reg(0xC0):%d\n", val);
    mutex_unlock(&input_dev->mutex);

    return count;
}

static ssize_t fts_glove_mode_store(
    struct device *dev,
    struct device_attribute *attr, const char *buf, size_t count)
{
    int ret = 0;
    struct fts_ts_data *ts_data = fts_data;

    if (FTS_SYSFS_ECHO_ON(buf)) {
        if (!ts_data->glove_mode) {
            FTS_DEBUG("enter glove mode");
            ret = fts_ex_mode_switch(MODE_GLOVE, ENABLE);
            if (ret >= 0) {
                ts_data->glove_mode = ENABLE;
            }
        }
    } else if (FTS_SYSFS_ECHO_OFF(buf)) {
        if (ts_data->glove_mode) {
            FTS_DEBUG("exit glove mode");
            ret = fts_ex_mode_switch(MODE_GLOVE, DISABLE);
            if (ret >= 0) {
                ts_data->glove_mode = DISABLE;
            }
        }
    }

    FTS_DEBUG("glove mode:%d", ts_data->glove_mode);
    return count;
}


static ssize_t fts_cover_mode_show(
    struct device *dev, struct device_attribute *attr, char *buf)
{
    int count = 0;
    u8 val = 0;
    struct fts_ts_data *ts_data = fts_data;
    struct input_dev *input_dev = ts_data->input_dev;

    mutex_lock(&input_dev->mutex);
    fts_read_reg(FTS_REG_COVER_MODE_EN, &val);
    count = snprintf(buf + count, PAGE_SIZE, "Cover Mode:%s\n",
                     ts_data->cover_mode ? "On" : "Off");
    count += snprintf(buf + count, PAGE_SIZE, "Cover Reg(0xC1):%d\n", val);
    mutex_unlock(&input_dev->mutex);

    return count;
}

static ssize_t fts_cover_mode_store(
    struct device *dev,
    struct device_attribute *attr, const char *buf, size_t count)
{
    int ret = 0;
    struct fts_ts_data *ts_data = fts_data;

    if (FTS_SYSFS_ECHO_ON(buf)) {
        if (!ts_data->cover_mode) {
            FTS_DEBUG("enter cover mode");
            ret = fts_ex_mode_switch(MODE_COVER, ENABLE);
            if (ret >= 0) {
                ts_data->cover_mode = ENABLE;
            }
        }
    } else if (FTS_SYSFS_ECHO_OFF(buf)) {
        if (ts_data->cover_mode) {
            FTS_DEBUG("exit cover mode");
            ret = fts_ex_mode_switch(MODE_COVER, DISABLE);
            if (ret >= 0) {
                ts_data->cover_mode = DISABLE;
            }
        }
    }

    FTS_DEBUG("cover mode:%d", ts_data->cover_mode);
    return count;
}

static ssize_t fts_charger_mode_show(
    struct device *dev, struct device_attribute *attr, char *buf)
{
    int count = 0;
    u8 val = 0;
    struct fts_ts_data *ts_data = fts_data;
    struct input_dev *input_dev = ts_data->input_dev;

    mutex_lock(&input_dev->mutex);
    fts_read_reg(FTS_REG_CHARGER_MODE_EN, &val);
    count = snprintf(buf + count, PAGE_SIZE, "Charger Mode:%s\n",
                     ts_data->charger_mode ? "On" : "Off");
    count += snprintf(buf + count, PAGE_SIZE, "Charger Reg(0x8B):%d\n", val);
    mutex_unlock(&input_dev->mutex);

    return count;
}

static ssize_t fts_charger_mode_store(
    struct device *dev,
    struct device_attribute *attr, const char *buf, size_t count)
{
    int ret = 0;
    struct fts_ts_data *ts_data = fts_data;

    if (FTS_SYSFS_ECHO_ON(buf)) {
        if (!ts_data->charger_mode) {
            FTS_DEBUG("enter charger mode");
            ret = fts_ex_mode_switch(MODE_CHARGER, ENABLE);
            if (ret >= 0) {
                ts_data->charger_mode = ENABLE;
            }
        }
    } else if (FTS_SYSFS_ECHO_OFF(buf)) {
        if (ts_data->charger_mode) {
            FTS_DEBUG("exit charger mode");
            ret = fts_ex_mode_switch(MODE_CHARGER, DISABLE);
            if (ret >= 0) {
                ts_data->charger_mode = DISABLE;
            }
        }
    }

    FTS_DEBUG("charger mode:%d", ts_data->glove_mode);
    return count;
}


/* read and write charger mode
 * read example: cat fts_glove_mode        ---read  glove mode
 * write example:echo 1 > fts_glove_mode   ---write glove mode to 01
 */
static DEVICE_ATTR(fts_glove_mode, S_IRUGO | S_IWUSR,
                   fts_glove_mode_show, fts_glove_mode_store);

static DEVICE_ATTR(fts_cover_mode, S_IRUGO | S_IWUSR,
                   fts_cover_mode_show, fts_cover_mode_store);

static DEVICE_ATTR(fts_charger_mode, S_IRUGO | S_IWUSR,
                   fts_charger_mode_show, fts_charger_mode_store);

static struct attribute *fts_touch_mode_attrs[] = {
    &dev_attr_fts_glove_mode.attr,
    &dev_attr_fts_cover_mode.attr,
    &dev_attr_fts_charger_mode.attr,
    NULL,
};

static struct attribute_group fts_touch_mode_group = {
    .attrs = fts_touch_mode_attrs,
};

int fts_ex_mode_recovery(struct fts_ts_data *ts_data)
{
    if (ts_data->glove_mode) {
        fts_ex_mode_switch(MODE_GLOVE, ENABLE);
    }

    if (ts_data->cover_mode) {
        fts_ex_mode_switch(MODE_COVER, ENABLE);
    }

    if (ts_data->charger_mode) {
        fts_ex_mode_switch(MODE_CHARGER, ENABLE);
    }

    return 0;
}

int fts_ex_mode_init(struct fts_ts_data *ts_data)
{
    int ret = 0;

    ts_data->glove_mode = DISABLE;
    ts_data->cover_mode = DISABLE;
    ts_data->charger_mode = DISABLE;

    ret = sysfs_create_group(&ts_data->dev->kobj, &fts_touch_mode_group);
    if (ret < 0) {
        FTS_ERROR("create sysfs(ex_mode) fail");
        sysfs_remove_group(&ts_data->dev->kobj, &fts_touch_mode_group);
        return ret;
    } else {
        FTS_DEBUG("create sysfs(ex_mode) succeedfully");
    }

    return 0;
}

int fts_ex_mode_exit(struct fts_ts_data *ts_data)
{
    sysfs_remove_group(&ts_data->dev->kobj, &fts_touch_mode_group);
    return 0;
}
#else
//prize-huangjiwu-for  glover mode --end

//prize-huangjiwu-for  glover mode --begin
struct fts_mode_flag {
    int  fts_glove_mode_flag;
    int  fts_cover_mode_flag;
    int  fts_charger_mode_flag;
};

struct fts_mode_flag g_fts_mode_flag;
/*****************************************************************************
* 4.Static variables
*****************************************************************************/

/*****************************************************************************
* 5.Global variable or extern global variabls/functions
*****************************************************************************/
int fts_enter_glove_mode(struct i2c_client *client, int mode );
int fts_enter_cover_mode(struct i2c_client *client, int mode );
int fts_enter_charger_mode(struct i2c_client *client, int mode );

/*****************************************************************************
* 6.Static function prototypes
*******************************************************************************/
#if FTS_GLOVE_EN
/*prize-for creat common node  /sys/prize/smartcover/state -yaozhipeng-20190718-start*/
#if defined(CONFIG_PRIZE_SMART_COVER_COMMON_NODE)
static ssize_t fts_touch_glove_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    int count;
    u8 val;
    struct input_dev *input_dev = fts_data->input_dev;
  //struct i2c_client *client =  fts_data->client;

    mutex_lock(&input_dev->mutex);
    fts_read_reg(FTS_REG_GLOVE_MODE_EN, &val);
    count = sprintf(buf, "Glove Mode: %s\n", g_fts_mode_flag.fts_glove_mode_flag ? "On" : "Off");
    count += sprintf(buf + count, "Glove Reg(0xC0) = %d\n", val);
    mutex_unlock(&input_dev->mutex);

    return count;
}

static ssize_t fts_touch_glove_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    int ret;
    struct fts_ts_data *ts_data = fts_data;
    struct i2c_client *client;
	int s = (buf[0] - '0');

    client = ts_data->client;
    if (s){
        if (!g_fts_mode_flag.fts_glove_mode_flag) {
            printk("[Mode]enter glove mode");
            ret = fts_enter_glove_mode(client, true);
            if (ret >= 0) {
                g_fts_mode_flag.fts_glove_mode_flag = true;
            }
        }
	}else{
			if (g_fts_mode_flag.fts_glove_mode_flag) {
				printk("[Mode]exit glove mode");
				ret = fts_enter_glove_mode(client, false);
				if (ret >= 0) {
					g_fts_mode_flag.fts_glove_mode_flag = false;
            }
        }
    }
    printk("[Mode]glove mode status:  %d", g_fts_mode_flag.fts_glove_mode_flag);
    return count;
}

/************************************************************************
* Name: fts_enter_glove_mode
* Brief:  change glove mode
* Input:  glove mode
* Output: no
* Return: success >=0, otherwise failed
***********************************************************************/
int fts_enter_glove_mode( struct i2c_client *client, int mode)
{
    int ret = 0;
    static u8 buf_addr[2] = { 0 };
    static u8 buf_value[2] = { 0 };
    buf_addr[0] = FTS_REG_GLOVE_MODE_EN; //glove control

    if (mode)
        buf_value[0] = 0x01;
    else
        buf_value[0] = 0x00;

    ret = fts_write_reg( buf_addr[0], buf_value[0]);
    if (ret < 0) {
        FTS_ERROR("[Mode]fts_enter_glove_mode write value fail");
    }

    return ret ;

}
static DEVICE_ATTR(state,0664, fts_touch_glove_show, fts_touch_glove_store);
#else
/*prize-for creat common node  /sys/prize/smartcover/state -yaozhipeng-20190718-end*/
static ssize_t fts_touch_glove_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    int count;
    u8 val;
    struct input_dev *input_dev = fts_data->input_dev;
    struct i2c_client *client = container_of(dev, struct i2c_client, dev);

    mutex_lock(&input_dev->mutex);
    fts_read_reg(client, FTS_REG_GLOVE_MODE_EN, &val);
    count = snprintf(buf, PAGE_SIZE, "Glove Mode: %s\n", g_fts_mode_flag.fts_glove_mode_flag ? "On" : "Off");
    count += snprintf(buf + count, PAGE_SIZE, "Glove Reg(0xC0) = %d\n", val);
    mutex_unlock(&input_dev->mutex);

    return count;
}

static ssize_t fts_touch_glove_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    int ret;
    struct fts_ts_data *ts_data = fts_data;
    struct i2c_client *client;


    client = ts_data->client;
    if (FTS_SYSFS_ECHO_ON(buf)) {
        if (!g_fts_mode_flag.fts_glove_mode_flag) {
            FTS_INFO("[Mode]enter glove mode");
            ret = fts_enter_glove_mode(client, true);
            if (ret >= 0) {
                g_fts_mode_flag.fts_glove_mode_flag = true;
            }
        }
    } else if (FTS_SYSFS_ECHO_OFF(buf)) {
        if (g_fts_mode_flag.fts_glove_mode_flag) {
            FTS_INFO("[Mode]exit glove mode");
            ret = fts_enter_glove_mode(client, false);
            if (ret >= 0) {
                g_fts_mode_flag.fts_glove_mode_flag = false;
            }
        }
    }
    FTS_INFO("[Mode]glove mode status:  %d", g_fts_mode_flag.fts_glove_mode_flag);
    return count;
}

/************************************************************************
* Name: fts_enter_glove_mode
* Brief:  change glove mode
* Input:  glove mode
* Output: no
* Return: success >=0, otherwise failed
***********************************************************************/
int fts_enter_glove_mode( struct i2c_client *client, int mode)
{
    int ret = 0;
    static u8 buf_addr[2] = { 0 };
    static u8 buf_value[2] = { 0 };
    buf_addr[0] = FTS_REG_GLOVE_MODE_EN; /* glove control */

    if (mode)
        buf_value[0] = 0x01;
    else
        buf_value[0] = 0x00;

    ret = fts_write_reg( buf_addr[0], buf_value[0]);
    if (ret < 0) {
        FTS_ERROR("[Mode]fts_enter_glove_mode write value fail");
    }

    return ret ;

}

/* read and write glove mode
*   read example: cat  fts_touch_glove_mode---read  glove mode
*   write example:echo 01 > fts_touch_glove_mode ---write glove mode to 01
*
*/
static DEVICE_ATTR (fts_glove_mode,  S_IRUGO | S_IWUSR, fts_touch_glove_show, fts_touch_glove_store);
#endif
#endif

#if FTS_COVER_EN
static ssize_t fts_touch_cover_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    int count;
    u8 val;
    struct input_dev *input_dev = fts_data->input_dev;
    struct i2c_client *client = container_of(dev, struct i2c_client, dev);

    mutex_lock(&input_dev->mutex);
    fts_read_reg(client, FTS_REG_COVER_MODE_EN, &val);
    count = snprintf(buf, PAGE_SIZE, "Cover Mode: %s\n", g_fts_mode_flag.fts_cover_mode_flag ? "On" : "Off");
    count += snprintf(buf + count, PAGE_SIZE, "Cover Reg(0xC1) = %d\n", val);
    mutex_unlock(&input_dev->mutex);

    return count;
}

static ssize_t fts_touch_cover_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    int ret;
    struct fts_ts_data *ts_data = fts_data;
    struct i2c_client *client;

    client = ts_data->client;
    if (FTS_SYSFS_ECHO_ON(buf)) {
        if (!g_fts_mode_flag.fts_cover_mode_flag) {
            FTS_INFO("[Mode]enter cover mode");
            ret = fts_enter_cover_mode(client, true);
            if (ret >= 0) {
                g_fts_mode_flag.fts_cover_mode_flag = true;
            }
        }
    } else if (FTS_SYSFS_ECHO_OFF(buf)) {
        if (g_fts_mode_flag.fts_cover_mode_flag) {
            FTS_INFO("[Mode]exit cover mode");
            ret = fts_enter_cover_mode(client, false);
            if (ret >= 0) {
                g_fts_mode_flag.fts_cover_mode_flag = false;
            }
        }
    }
    FTS_INFO("[Mode]cover mode status:  %d", g_fts_mode_flag.fts_cover_mode_flag);
    return count;
}

/************************************************************************
* Name: fts_enter_cover_mode
* Brief:  change cover mode
* Input:  cover mode
* Output: no
* Return: success >=0, otherwise failed
***********************************************************************/
int  fts_enter_cover_mode( struct i2c_client *client, int mode)
{
    int ret = 0;
    static u8 buf_addr[2] = { 0 };
    static u8 buf_value[2] = { 0 };
    buf_addr[0] = FTS_REG_COVER_MODE_EN; /* cover control */

    if (mode)
        buf_value[0] = 0x01;
    else
        buf_value[0] = 0x00;

    ret = fts_write_reg( client, buf_addr[0], buf_value[0]);
    if (ret < 0) {
        FTS_ERROR("[Mode] fts_enter_cover_mode write value fail \n");
    }

    return ret ;

}

/* read and write cover mode
*   read example: cat  fts_touch_cover_mode---read  cover mode
*   write example:echo 01 > fts_touch_cover_mode ---write cover mode to 01
*
*/
static DEVICE_ATTR (fts_cover_mode,  S_IRUGO | S_IWUSR, fts_touch_cover_show, fts_touch_cover_store);

#endif

#if FTS_CHARGER_EN
static ssize_t fts_touch_charger_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    int count;
    u8 val;
    struct input_dev *input_dev = fts_data->input_dev;
    struct i2c_client *client = container_of(dev, struct i2c_client, dev);

    mutex_lock(&input_dev->mutex);
    fts_read_reg(client, FTS_REG_CHARGER_MODE_EN, &val);
    count = snprintf(buf, PAGE_SIZE, "Charge Mode: %s\n", g_fts_mode_flag.fts_charger_mode_flag ? "On" : "Off");
    count += snprintf(buf + count, PAGE_SIZE, "Charge Reg(0x8B) = %d\n", val);
    mutex_unlock(&input_dev->mutex);

    return count;
}

static ssize_t fts_touch_charger_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    int ret;
    struct fts_ts_data *ts_data = fts_data;
    struct i2c_client *client;

    client = ts_data->client;

    if (FTS_SYSFS_ECHO_ON(buf)) {
        if (!g_fts_mode_flag.fts_charger_mode_flag) {
            FTS_INFO("[Mode]enter charger mode");
            ret = fts_enter_charger_mode(client, true);
            if (ret >= 0) {
                g_fts_mode_flag.fts_charger_mode_flag = true;
            }
        }
    } else if (FTS_SYSFS_ECHO_OFF(buf)) {
        if (g_fts_mode_flag.fts_charger_mode_flag) {
            FTS_INFO("[Mode]exit charger mode");
            ret = fts_enter_charger_mode(client, false);
            if (ret >= 0) {
                g_fts_mode_flag.fts_charger_mode_flag = false;
            }
        }
    }
    FTS_INFO("[Mode]charger mode status: %d", g_fts_mode_flag.fts_charger_mode_flag);
    return count;
}

/************************************************************************
* Name: fts_enter_charger_mode
* Brief:  change charger mode
* Input:  charger mode
* Output: no
* Return: success >=0, otherwise failed
***********************************************************************/
int  fts_enter_charger_mode(struct i2c_client *client, int mode)
{
    int ret = 0;
    static u8 buf_addr[2] = { 0 };
    static u8 buf_value[2] = { 0 };
    buf_addr[0] = FTS_REG_CHARGER_MODE_EN; /* charger control */

    if (mode)
        buf_value[0] = 0x01;
    else
        buf_value[0] = 0x00;

    ret = fts_write_reg( client, buf_addr[0], buf_value[0]);
    if (ret < 0) {
        FTS_DEBUG("[Mode]fts_enter_charger_mode write value fail");
    }

    return ret ;

}

/* read and write charger mode
*   read example: cat  fts_touch_charger_mode---read  charger mode
*   write example:echo 01 > fts_touch_charger_mode ---write charger mode to 01
*
*/
static DEVICE_ATTR (fts_charger_mode,  S_IRUGO | S_IWUSR, fts_touch_charger_show, fts_touch_charger_store);

#endif

static struct attribute *fts_touch_mode_attrs[] = {
#if FTS_GLOVE_EN
#if !defined(CONFIG_PRIZE_SMART_COVER_COMMON_NODE)
    &dev_attr_fts_glove_mode.attr,
#endif
#endif

#if FTS_COVER_EN
    &dev_attr_fts_cover_mode.attr,
#endif

#if FTS_CHARGER_EN
    &dev_attr_fts_charger_mode.attr,
#endif

    NULL,
};

static struct attribute_group fts_touch_mode_group = {
    .attrs = fts_touch_mode_attrs,
};

int fts_ex_mode_init(struct fts_ts_data *ts_data)
{
/*prize-for creat common node  /sys/prize/smartcover/state -yaozhipeng-20190718-start*/
	#if defined(CONFIG_PRIZE_SMART_COVER_COMMON_NODE)  
		static struct kobject *sysfs_rootdir = NULL; 
		struct kobject *prize_glove = NULL;
		int err = 0;
		
		g_fts_mode_flag.fts_glove_mode_flag = false;
		g_fts_mode_flag.fts_cover_mode_flag = false;
		g_fts_mode_flag.fts_charger_mode_flag = false;
		

		if (!sysfs_rootdir) {
			// this kobject is shared between modules, do not free it when error occur
			sysfs_rootdir = kobject_create_and_add("prize", kernel_kobj);
		}

		if (!prize_glove){
			prize_glove = kobject_create_and_add("smartcover", sysfs_rootdir);
		}
		err = sysfs_create_link(prize_glove,&ts_data->client->dev.kobj,"common_node");
		if (err){
			printk("prize fts sysfs_create_link fail\n");
		}
		if(sysfs_create_file(&ts_data->client->dev.kobj, &dev_attr_state.attr))
		{
			return -1;
		}
			return 0;
	#else
/*prize-for creat common node  /sys/prize/smartcover/state -yaozhipeng-20190718-end*/
    int err = 0;

    g_fts_mode_flag.fts_glove_mode_flag = false;
    g_fts_mode_flag.fts_cover_mode_flag = false;
    g_fts_mode_flag.fts_charger_mode_flag = false;

    err = sysfs_create_group(&ts_data->client->dev.kobj, &fts_touch_mode_group);
    if (0 != err) {
        FTS_ERROR("[Mode]create sysfs failed.");
        sysfs_remove_group(&ts_data->client->dev.kobj, &fts_touch_mode_group);
        return -EIO;
    } else {
        FTS_DEBUG("[Mode]create sysfs succeeded");
    }

    return err;
	#endif
}

int fts_ex_mode_exit(struct fts_ts_data *ts_data)
{
    sysfs_remove_group(&ts_data->client->dev.kobj, &fts_touch_mode_group);
    return 0;
}

int fts_ex_mode_recovery(struct fts_ts_data *ts_data)
{
	
    int ret = 0;
#if FTS_GLOVE_EN
    if (g_fts_mode_flag.fts_glove_mode_flag)
        ret = fts_enter_glove_mode(ts_data->client, true);
#endif

#if FTS_COVER_EN
    if (g_fts_mode_flag.fts_cover_mode_flag)
        ret = fts_enter_cover_mode(ts_data->client, true);
#endif

#if FTS_CHARGER_EN
    if (g_fts_mode_flag.fts_charger_mode_flag)
        ret = fts_enter_charger_mode(ts_data->client, true);
#endif

    return ret;
}
#endif
//prize-huangjiwu-for  glover mode --end
