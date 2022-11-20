

#ifndef _HARDWARE_INFO_H_
#define _HARDWARE_INFO_H_


struct hardware_info{
	unsigned char chip[32];
	unsigned char vendor[32];
	unsigned char id[32];
	unsigned char more[64];
#if defined(CONFIG_PRIZE_HARDWARE_INFO_BAT)
	unsigned char batt_versions[32];
	unsigned char Q_MAX_POS_50[32];
	unsigned char Q_MAX_POS_25[32];
	unsigned char Q_MAX_POS_10[32];
	unsigned char Q_MAX_POS_0[32];
#endif	
};

#endif /* _HARDWARE_INFO_H_ */
