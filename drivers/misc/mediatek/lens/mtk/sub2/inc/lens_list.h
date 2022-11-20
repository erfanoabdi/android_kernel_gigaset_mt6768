/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 */



#ifndef _LENS_LIST_H

#define _LENS_LIST_H

//prize add by tangcong 20220416 start 
#define AW86014AF_SetI2Cclient AW86014AF_SetI2Cclient_Sub2
#define AW86014AF_Ioctl AW86014AF_Ioctl_Sub2
#define AW86014AF_Release AW86014AF_Release_Sub2
#define AW86014AF_PowerDown AW86014AF_PowerDown_Sub2
#define AW86014AF_GetFileName AW86014AF_GetFileName_Sub2
extern int AW86014AF_SetI2Cclient(struct i2c_client *pstAF_I2Cclient,
				 spinlock_t *pAF_SpinLock, int *pAF_Opened);
extern long AW86014AF_Ioctl(struct file *a_pstFile, unsigned int a_u4Command,
			   unsigned long a_u4Param);
extern int AW86014AF_Release(struct inode *a_pstInode, struct file *a_pstFile);
extern int AW86014AF_PowerDown(struct i2c_client *pstAF_I2Cclient,
				int *pAF_Opened);
extern int AW86014AF_GetFileName(unsigned char *pFileName);
#define DW9718SAF_SetI2Cclient DW9718SAF_SetI2Cclient_Sub2
#define DW9718SAF_Ioctl DW9718SAF_Ioctl_Sub2
#define DW9718SAF_Release DW9718SAF_Release_Sub2
extern int DW9718SAF_SetI2Cclient(struct i2c_client *pstAF_I2Cclient, spinlock_t *pAF_SpinLock, int *pAF_Opened);
extern long DW9718SAF_Ioctl(struct file *a_pstFile, unsigned int a_u4Command, unsigned long a_u4Param);
extern int DW9718SAF_Release(struct inode *a_pstInode, struct file *a_pstFile);
extern int DW9718SAF_GetFileName(unsigned char *pFileName);
//prize add by tangcong 20220416 end 

#endif
