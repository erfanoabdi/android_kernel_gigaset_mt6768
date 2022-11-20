#include <linux/types.h>
#ifndef _AW32207_SW_H_
#define _AW32207_SW_H_

#define AW32207_CHARGE

#define AW32207_CON0			0x00
#define AW32207_CON1			0x01
#define AW32207_CON2			0x02
#define AW32207_CON3			0x03
#define AW32207_CON4			0x04
#define AW32207_CON5			0x05
#define AW32207_CON6			0x06
#define AW32207_CON7			0x07
#define AW32207_CON8			0x08
#define AW32207_CON9			0x09
#define AW32207_CONA			0x0A
#define AW32207_REG_NUM			11
/* CON0 */
#define CON0_TMR_RST_MASK	0x01
#define CON0_TMR_RST_SHIFT	7
#define CON0_OTG_MASK			0x01
#define CON0_OTG_SHIFT			7
#define CON0_EN_STAT_MASK		0x01
#define CON0_EN_STAT_SHIFT		6
#define CON0_STAT_MASK			0x03
#define CON0_STAT_SHIFT			4
#define CON0_BOOST_MASK			0x01
#define CON0_BOOST_SHIFT		3
#define CON0_CHG_FAULT_MASK		0x07
#define CON0_CHG_FAULT_SHIFT		0

/* CON1 */
#define CON1_LIN_LIMIT_MASK	0x03   // aw32207 have not
#define CON1_LIN_LIMIT_SHIFT	6  // aw32207 have not
#define CON1_LOW_V_MASK		0x03
#define CON1_LOW_V_SHIFT	4
#define CON1_TE_MASK			0x01
#define CON1_TE_SHIFT			3
#define CON1_CEN_MASK			0x01
#define CON1_CEN_SHIFT			2
#define CON1_HZ_MODE_MASK		0x01
#define CON1_HZ_MODE_SHIFT		1
#define CON1_OPA_MODE_MASK		0x01
#define CON1_OPA_MODE_SHIFT		0

#define CON2_VOREG_MASK			0x3F
#define CON2_VOREG_SHIFT		2
#define CON2_OTG_PL_MASK		0x01
#define CON2_OTG_PL_SHIFT		1
#define CON2_OTG_EN_MASK		0x01
#define CON2_OTG_EN_SHIFT		0

#define CON3_VENDER_MASK		0x07
#define CON3_VENDER_SHIFT		5
#define CON3_PN_MASK			0x03
#define CON3_PN_SHIFT			3
#define CON3_REVISION_MASK		0x07
#define CON3_REVISION_SHIFT		0

#define CON4_RESET_MASK			0x01
#define CON4_RESET_SHIFT		7
#define CON4_I_CHR_MASK			0xf
#define CON4_I_CHR_SHIFT		3
#define CON4_I_TERM_MASK		0x07
#define CON4_I_TERM_SHIFT		0
/*ETA6937*/
#define CON4_I_CHR_OFFSET_MASK	0x01
#define CON4_I_CHR_OFFSET_SHIFT	3

/* CON5 */
#define CON5_DIS_VREG_MASK	0x01	// aw32207 have not 
#define CON5_DIS_VREG_SHIFT	6	// aw32207 have not 
#define CON5_IO_LEVEL_MASK	0x01	// aw32207 have not 
#define CON5_IO_LEVEL_SHIFT	5		// aw32207 have not 
#define CON5_DPM_STATUS_MASK		0x01
#define CON5_DPM_STATUS_SHIFT		4
#define CON5_CD_STATUS_MASK		0x01
#define CON5_CD_STATUS_SHIFT		3
#define CON5_VSP_MASK			0x07
#define CON5_VSP_SHIFT			0
/*ETA6937*/
#define CON5_I_CHR_BIT3_MASK	0x01
#define CON5_I_CHR_BIT3_SHIFT	6
#define CON5_I_CHR_BIT4_MASK	0x01
#define CON5_I_CHR_BIT4_SHIFT	7
#define CON5_I_CHRH_MASK	0x03
#define CON5_I_CHRH_SHIFT	6

/* CON6 */
#define CON6_ISAFE_MASK			0x0F
#define CON6_ISAFE_SHIFT		4
#define CON6_VSAFE_MASK			0x0F
#define CON6_VSAFE_SHIFT		0

/* CON7 */
#define CON7_TE_P_MASK			0x01
#define CON7_TE_P_SHIFT			7
#define CON7_TE_NUM_MASK		0x03
#define CON7_TE_NUM_SHIFT		5
#define CON7_TE_DEG_TM_MASK		0x03
#define CON7_TE_DEG_TM_SHIFT		3
#define CON7_VRECHG_MASK		0x03
#define CON7_VRECHG_SHIFT		0
/*ETA6937*/
#define CON7_IINLIMIT_SELECTION_MASK		0x01
#define CON7_IINLIMIT_SELECTION_SHIFT	3
#define CON7_IINLIMIT2_MASK		0x07
#define CON7_IINLIMIT2_SHIFT	0

#define CON8_VENDOR_MASK		0xFF
#define CON8_TE_P_SHIFT			0

#define CON9_BST_FAULT_MASK		0x07
#define CON9_BST_FAULT_SHIFT		0

#define CONA_PWM_FRQ_MASK		0x01
#define CONA_PWM_FRQ_SHIFT		7
#define CONA_SLOW_SW_MASK		0x03
#define CONA_SLOW_SW_SHIFT		5
#define CONA_FIX_DEADT_MASK		0x01
#define CONA_FIX_DEADT_SHIFT		4
#define CONA_FPWM_MASK			0x01
#define CONA_FPWM_SHIFT			3
#define CONA_BSTOUT_CFG_MASK		0x03
#define CONA_BSTOUT_CFG_SHIFT		0
#endif
