#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <linux/ioctl.h>
#include <unistd.h>
#include <math.h>
#include <sys/wait.h>


#define AWINIC_SMARTPA_CALI_RE   "/sys/class/smartpa/re25_calib"
#define AWINIC_SMARTPA_CALI_F0    "/sys/class/smartpa/f0_calib"


#define READ_BUF_MAX	(512)
#define AW_STR		"aw"
#define SET_RE_BUF_MAX (128)


enum {
	CALI_STR_NONE = 0,
	CALI_STR_CALI_RE_F0,
	CALI_STR_CALI_RE,
	CALI_STR_CALI_F0,
	CALI_STR_SET_RE,
	CALI_STR_SHOW_RE,		/*show cali_re*/
	CALI_STR_SHOW_R0,		/*show real r0*/
	CALI_STR_SHOW_CALI_F0,		/*GET DEV CALI_F0*/
	CALI_STR_SHOW_F0,		/*SHOW REAL F0*/
	CALI_STR_SHOW_TE,
	CALI_STR_DEV_SEL,		/*switch device*/
	CALI_STR_VER,
	CALI_STR_SHOW_RE_RANGE,
	CALI_STR_MAX,
};


static const char *cali_str[CALI_STR_MAX] = {
	"none", "start_cali", "cali_re", "cali_f0", "store_re",
	"show_re", "show_r0", "show_cali_f0", "show_f0","show_te", "dev_sel", "get_ver", "get_re_range"
};


enum {
	AW_DEV_0 = 0,
	AW_DEV_1 = 1,
	AW_DEV_2 = 2,
	AW_DEV_3 = 3,
	AW_DEV_4 = 4,
	AW_DEV_5 = 5,
	AW_DEV_6 = 6,
	AW_DEV_7 = 7,
	AW_DEV_8 = 8,
	AW_DEV_9 = 9,
	AW_DEV_10 = 10,
	AW_DEV_11 = 11,
	AW_DEV_12 = 12,
	AW_DEV_13 = 13,
	AW_DEV_14 = 14,
	AW_DEV_15 = 15,
	AW_DEV_CH_MAX,
};

struct aw_show_data {
	int count;
	int data[AW_DEV_CH_MAX];
};

struct aw_set_re {
	int count;
	int re[AW_DEV_CH_MAX];
};


int aw883xx_svc_open_dev(int *fd, uint8_t cmd)
{
	switch (cmd) {
	case CALI_STR_SHOW_RE:
		*fd = open(AWINIC_SMARTPA_CALI_RE, O_RDWR);
		if (*fd < 0) {
			printf("%s:open %s failed\n",
				__func__, AWINIC_SMARTPA_CALI_RE);
			return -1;
		}
		break;
	case CALI_STR_SHOW_CALI_F0:
		*fd = open(AWINIC_SMARTPA_CALI_F0, O_RDONLY);
		if (*fd < 0) {
			printf("%s:open %s failed\n",
				__func__, AWINIC_SMARTPA_CALI_F0);
			return -1;
		}
		break;
	case CALI_STR_SET_RE:
		*fd = open(AWINIC_SMARTPA_CALI_RE, O_RDWR);
		if (*fd < 0) {
			printf("%s:open %s failed\n",
				__func__, AWINIC_SMARTPA_CALI_RE);
			return -1;
		}
		break;
	default:
		printf("%s: unsupported cmd %s\n", __func__, cali_str[cmd]);
		return -1;
	}

	return 0;
}

void aw883xx_svc_colse_dev(int fd)
{
	close(fd);
}

static int aw883xx_svc_write_data(const char *data_buf, int data_size)
{
	int ret = 0;
	int fd = 0;

	ret = aw883xx_svc_open_dev(&fd, CALI_STR_SET_RE);
	if (ret < 0)
		return ret;

	ret = write(fd, data_buf, data_size);
	if (ret <= 0) {
		printf("%s:write data to dev node failed\n", __func__);
		ret = -1;
	}

	aw883xx_svc_colse_dev(fd);
	return ret;
};

static int aw883xx_svc_read_data(uint8_t cmd, char *data_buf, int data_size)
{
	int ret = 0;
	int fd = 0;

	ret = aw883xx_svc_open_dev(&fd, cmd);
	if (ret < 0)
		return ret;

	ret = read(fd, data_buf, data_size);
	if (ret < 0) {
		printf("%s:read data from dev node failed\n", __func__);
		ret = -1;
	}

	aw883xx_svc_colse_dev(fd);
	return ret;
};

static int aw883xx_svc_read_common_show_data(uint8_t cmd,
						struct aw_show_data *show_data)
{
	int ret;
	char data_buf[READ_BUF_MAX] = {AW_STR};
	char str_data[32] = { 0 };
	int i;
	int len = 0;

	memset(show_data, 0, sizeof(struct aw_show_data));

	switch (cmd) {
	case CALI_STR_SHOW_RE:
		ret = aw883xx_svc_read_data(CALI_STR_SHOW_RE, data_buf + strlen(AW_STR), sizeof(data_buf) - strlen(AW_STR));
		if (ret < 0) {
			printf("%s:read cmd %s failed\n", __func__, cali_str[cmd]);
			return ret;
		}

		for (i = 0; i < AW_DEV_CH_MAX; i++) {
			memset(str_data, 0, sizeof(str_data));
			snprintf(str_data, sizeof(str_data), "%sdev[%d]:%s ", "%*[^d]", i, "%d");

			ret = sscanf(data_buf + len, str_data, &show_data->data[i]);
			if (ret <= 0) {
				if (i == 0) {
					printf("unsupported str: %s", data_buf);
					return -1;
				}
				show_data->count = i;
				break;
			}
			len += snprintf(str_data, sizeof(str_data), "dev[%d]:%d ", i, show_data->data[i]);
		}
		break;

	case CALI_STR_SHOW_CALI_F0:
		ret = aw883xx_svc_read_data(CALI_STR_SHOW_CALI_F0, data_buf + strlen(AW_STR), sizeof(data_buf) - strlen(AW_STR));
		if (ret < 0) {
			printf("%s:read cmd %s failed\n", __func__, cali_str[cmd]);
			return ret;
		}

		for (i = 0; i < AW_DEV_CH_MAX; i++) {
			memset(str_data, 0, sizeof(str_data));
			snprintf(str_data, sizeof(str_data), "%sdev[%d]:%s ", "%*[^d]", i, "%d");

			ret = sscanf(data_buf + len, str_data, &show_data->data[i]);
			if (ret <= 0) {
				if (i == 0) {
					printf("unsupported str: %s", data_buf);
					return -1;
				}
				show_data->count = i;
				break;
			}
			len += snprintf(str_data, sizeof(str_data), "dev[%d]:%d ", i, show_data->data[i]);
		}
		break;
	default:
		printf("%s: unsupported cmd %s\n", __func__, cali_str[cmd]);
		return -1;
	}

	return 0;
}

int aw883xx_write_cali_re_to_dirver(struct aw_show_data set_re)
{
	int ret;
	char set_re_buf[SET_RE_BUF_MAX] = { 0 };
	int len = 0;
	int i;

	for (i = 0; i < set_re.count; i++) {
		len += snprintf(set_re_buf + len, SET_RE_BUF_MAX - 1 - len,
			"dev[%d]:%d ", i, set_re.data[i]);
	}

	ret = aw883xx_svc_write_data(set_re_buf, strlen(set_re_buf) + 1);
	if (ret < 0) {
		printf("%s:write re failed\n", __func__);
		return ret;
	}

	for (i = 0; i < set_re.count; i++)
		printf("dev[%d]:set cali re %d\n", i, set_re.data[i]);

	return 0;
}


int aw883xx_cali_re(struct aw_show_data *re_data)
{
	int ret;
	int i;

	ret = aw883xx_svc_read_common_show_data(CALI_STR_SHOW_RE, re_data);
	if (ret < 0)
		return ret;

	for (i = 0; i < re_data->count; i++)
		printf("dev[%d]cali_RE = %d\n", i, re_data->data[i]);

	return 0;
}

int aw883xx_cali_f0(struct aw_show_data *f0_data)
{
	int ret;
	int i;

	ret = aw883xx_svc_read_common_show_data(CALI_STR_SHOW_CALI_F0, f0_data);
	if (ret < 0) {
		printf("cali f0 failed\n");
		return ret;
	}
	for (i = 0; i < f0_data->count; i++)
		printf("dev[%d]cali_f0 = %d\n", i, f0_data->data[i]);

	return 0;
}

int main(void)
{
	int ret = 0, i;
	struct aw_show_data re_data;
	struct aw_show_data f0_data;

	printf("---------------------------------------\n");
	printf("aw883xx_cali_class_example\n");
	printf("---------------------------------------\n");

	ret = aw883xx_cali_re(&re_data);
	if (ret < 0) {
		printf("cali_re failed");
		return -1;
	}

	aw883xx_write_cali_re_to_dirver(re_data);

	ret = aw883xx_cali_f0(&f0_data);
	if (ret < 0) {
		printf("cali_f0 failed");
		return -1;
	}

	for (i = 0; i < re_data.count; i++)
		printf("dev[%d]cali_re = %d\n", i, re_data.data[i]);

	for (i = 0; i < re_data.count; i++)
		printf("dev[%d]set_re = %d\n", i, re_data.data[i]);

	for (i = 0; i < f0_data.count; i++)
		printf("dev[%d]cali_f0 = %d\n", i, f0_data.data[i]);

	return 0;
}

