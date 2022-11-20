#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/slab.h>
#include "semi_touch_upgrade.h"
#include "fw_code_bin.h"

#if SEMI_TOUCH_BOOTUP_UPDATE_EN
#include "fw_update_packet.h"
#endif

#define BURN_W                              0x0
#define BURN_R                              0x3
#define CORE_R                              0x4
#define CORE_W                              0x5

enum MCAP_RAM_CODE
{
    RAM_CODE_BURN_MCAP_SHARE  = 0, 
    RAM_CODE_SHORT_DATA_SHARE = 1,
    SEMI_TOUCH_RAM_CODE_COUNT,
};

struct bin_code_wraper
{
    const unsigned char* bin_code_addr;
    unsigned short bin_code_len;
};

struct bin_code_wraper bin_code_map[SEMI_TOUCH_RAM_CODE_COUNT] = 
{
    {fw_burn_mcapshare, sizeof(fw_burn_mcapshare)},
    {fw_short_mcapshare, sizeof(fw_short_mcapshare)},
};

int semi_touch_write_core_data_and_check(unsigned int addr, const unsigned char* buffer, unsigned short len)
{
    int ret = 0, once = 0, index = 0, retry = 0;
    unsigned char core_cmp_buffer[MAX_CORE_WRITE_LEN];
    const int max_try = 3;

    while(len > 0)
    {
        retry = 0;
        do
        {
            ret = SEMI_DRV_ERR_OK;
            once = min(len, (unsigned short)MAX_CORE_WRITE_LEN);
            ret = semi_touch_write_bytes(addr, buffer, once);
            ret = semi_touch_read_bytes(addr, core_cmp_buffer, once);
            for(index = 0; index < once; index++)
            {
                if(core_cmp_buffer[index] != buffer[index])
                {
                    //kernel_log_d("err pos = %d\r\n", index);
                    ret = -SEMI_DRV_ERR_CHECKSUM;
                    break;
                }
            }
            if(SEMI_DRV_ERR_OK == ret){
                break;
            }         
        }while(++retry < max_try);

        check_return_if_fail(ret, NULL);
        
        addr += once;
        buffer += once;
        len-= once;
    }

    return ret;
}
int semi_touch_run_ram_code(unsigned char code)
{
    int retry;
    int ret = 0, reg_value = 0;
    
    if(code >= SEMI_TOUCH_RAM_CODE_COUNT) return -EINVAL;

    for(retry = 0; retry < 5; retry++){
        //reset mcu
        semi_touch_reset(no_report_after_reset);
        mdelay(5);
        
        //hold mcu
        reg_value = TP_HOLD_MCU_VAL;
        ret = semi_touch_write_bytes(TP_HOLD_MCU_ADDR, (unsigned char*)&reg_value, 4);
        if(ret < 0){
            continue;
        }
        //open auto feed
        reg_value = TP_AUTO_FEED_VAL;
        ret = semi_touch_write_bytes(TP_AUTO_FEED_ADDR, (unsigned char*)&reg_value, 4);
        if(ret < 0){
            continue;
        }

        //run ramcode
        ret = semi_touch_write_core_data_and_check(0x20000000, bin_code_map[code].bin_code_addr, bin_code_map[code].bin_code_len);
        if(ret < 0){
            continue;
        }

        break;
    }
    check_return_if_fail(ret, NULL);

    //remap
    reg_value = TP_REMAP_MCU_VAL;
    ret = semi_touch_write_bytes(TP_REMAP_MCU_ADDR, (unsigned char*)&reg_value, 4);
    check_return_if_fail(ret, NULL);

    //release mcu
    reg_value = TP_RELEASE_MCU_VAL;
    ret = semi_touch_write_bytes(TP_RELEASE_MCU_ADDR, (unsigned char*)&reg_value, 4);
    check_return_if_fail(ret, NULL);

    mdelay(30);

    return 0;
}

/*
    This function will put IC into NVM mode, call it carefully and must reset 
    the chip before entering normal mode.
*/
int semi_touch_enter_burn_mode(void)
{
    int ret = 0;
    struct m_ctp_cmd_std_t cmd_send_tp;
    struct m_ctp_rsp_std_t ack_from_tp;

    cmd_send_tp.id = CMD_IDENTITY;
    ret = cmd_send_to_tp(&cmd_send_tp, &ack_from_tp, 200);
    //if empty chip

    if((ack_from_tp.d0 == 0xE9A2) && (ack_from_tp.d1 == 0x165d))
    {
        set_status_upgrade_run(st_dev.stc.ctp_run_status);
        return SEMI_DRV_ERR_OK;
    }

    ret = semi_touch_run_ram_code(RAM_CODE_BURN_MCAP_SHARE);
    check_return_if_fail(ret, NULL);

    cmd_send_tp.id = CMD_IDENTITY;
    ret = cmd_send_to_tp(&cmd_send_tp, &ack_from_tp, 200);
    check_return_if_fail(ret, NULL);

    if((ack_from_tp.d0 == 0xE9A2) && (ack_from_tp.d1 == 0x165d))
    {
        set_status_upgrade_run(st_dev.stc.ctp_run_status);
        return SEMI_DRV_ERR_OK;
    }

    set_status_pointing(st_dev.stc.ctp_run_status);
    return -SEMI_DRV_ERR_HAL_IO;
}
int semi_touch_core_write(struct apk_complex_data* apk_comlex_addr)
{
    int ret = -EINVAL;
    struct m_ctp_cmd_std_t* ptr_cmd = (struct m_ctp_cmd_std_t*)apk_comlex_addr->stm_cmd_buffer;
    unsigned short left = ptr_cmd->d1;
    unsigned int addr = (ptr_cmd->d5 << 16) + ptr_cmd->d0;

    if(ptr_cmd->d2 != caculate_checksum_u816((unsigned char*)apk_comlex_addr->stm_txrx_buffer, left))
    {
        return -SEMI_DRV_ERR_CHECKSUM;
    }

    ret = semi_touch_write_bytes(addr, apk_comlex_addr->stm_txrx_buffer, left);

    return ret;
}
int semi_touch_core_read(struct apk_complex_data* apk_comlex_addr)
{
    int ret = -EINVAL;
    struct m_ctp_cmd_std_t* ptr_cmd = (struct m_ctp_cmd_std_t*)apk_comlex_addr->stm_cmd_buffer;
    struct m_ctp_rsp_std_t* driv_rsp = (struct m_ctp_rsp_std_t*)apk_comlex_addr->stm_rsp_buffer;
    unsigned short left = ptr_cmd->d1;
    unsigned int addr = (ptr_cmd->d5 << 16) + ptr_cmd->d0;

    ret = semi_touch_read_bytes(addr, apk_comlex_addr->stm_txrx_buffer, left);
    driv_rsp->d0 = caculate_checksum_u816(apk_comlex_addr->stm_txrx_buffer, left);

    return ret;
}
int semi_touch_burn_write(struct apk_complex_data* apk_comlex_addr)
{
    int ret = -EINVAL;
    struct m_ctp_cmd_std_t* ptr_cmd = (struct m_ctp_cmd_std_t*)apk_comlex_addr->stm_cmd_buffer;
    struct m_ctp_rsp_std_t* driv_rsp = (struct m_ctp_rsp_std_t*)apk_comlex_addr->stm_rsp_buffer;
    unsigned short left = ptr_cmd->d1;

    ret = semi_touch_enter_burn_mode();
    check_return_if_fail(ret, NULL);

    ret = semi_touch_write_bytes(TP_WR_BUFF_ADDR, apk_comlex_addr->stm_txrx_buffer, left);
    check_return_if_fail(ret, NULL);

    return cmd_send_to_tp(ptr_cmd, driv_rsp, 7500);
}
int semi_touch_burn_read(struct apk_complex_data* apk_comlex_addr)
{
    int ret = -EINVAL;
    struct m_ctp_cmd_std_t *ptr_cmd = (struct m_ctp_cmd_std_t*)apk_comlex_addr->stm_cmd_buffer;
    struct m_ctp_rsp_std_t* driv_rsp = (struct m_ctp_rsp_std_t*)apk_comlex_addr->stm_rsp_buffer;
    unsigned short left = ptr_cmd->d1;

    ret = semi_touch_enter_burn_mode();
    check_return_if_fail(ret, NULL);

    ret = cmd_send_to_tp(ptr_cmd, driv_rsp, 200);
    check_return_if_fail(ret, NULL);

    ret = semi_touch_read_bytes(TP_RD_BUFF_ADDR, apk_comlex_addr->stm_txrx_buffer, left);

    return ret;
}
int semi_touch_memory_write(struct apk_complex_data* apk_comlex_addr)
{
    struct m_ctp_cmd_std_t *ptr_cmd = (struct m_ctp_cmd_std_t*)apk_comlex_addr->stm_cmd_buffer;

    if(CORE_W == ptr_cmd->d3)
    {
        return semi_touch_core_write(apk_comlex_addr);
    }
    else
    {
        return semi_touch_burn_write(apk_comlex_addr);
    }
}
int semi_touch_memory_read(struct apk_complex_data* apk_comlex_addr)
{
    struct m_ctp_cmd_std_t *ptr_cmd = (struct m_ctp_cmd_std_t*)apk_comlex_addr->stm_cmd_buffer;

    if(CORE_R == ptr_cmd->d3)
    {
        return semi_touch_core_read(apk_comlex_addr);
    }
    else
    {
        return semi_touch_burn_read(apk_comlex_addr);
    }
}

int semi_touch_bulk_read(unsigned char *pdes, unsigned int adr, unsigned int len)
{
    int ret = -EINVAL;
    unsigned int left = len;
    unsigned int local_check, retry;
    struct m_ctp_cmd_std_t cmd_send_tp;
    struct m_ctp_rsp_std_t ack_from_tp;

    cmd_send_tp.id = CMD_MEM_RD;

    while (left) {
        len = (left > 1024) ? 1024 : left;

        cmd_send_tp.d0 = adr & 0xffff;
        cmd_send_tp.d1 = len;
        cmd_send_tp.d2 = 0;
        cmd_send_tp.d3 = BURN_R;
        cmd_send_tp.d5 = (adr >> 16) & 0xffff;

        retry = 0;
        while (retry++ < 3) {
            ack_from_tp.id = CMD_NA;
            ret = cmd_send_to_tp(&cmd_send_tp, &ack_from_tp, 200);
            if(SEMI_DRV_ERR_OK != ret) continue;

            semi_touch_read_bytes(TP_RD_BUFF_ADDR, pdes, len);
            local_check = caculate_checksum_ex(pdes, len);
            if ((ack_from_tp.d0 != (unsigned short)local_check) || (ack_from_tp.d1 != (unsigned short)(local_check >> 16))) continue;

            break;
        }

        adr  += len;
        left -= len;
        pdes += len;
        check_break_if_fail(ret, NULL);
    }

    return ret;
}

int semi_touch_bulk_write(unsigned char *psrc, unsigned int adr, unsigned int len)
{
    int ret = -EINVAL;
    unsigned int left = len;
    unsigned int retry, combChk;
    struct m_ctp_cmd_std_t cmd_send_tp;
    struct m_ctp_rsp_std_t ack_from_tp;

    cmd_send_tp.id = CMD_MEM_WR;

    while (left) {
        len     = (left > 1024) ? 1024 : left;
        combChk = caculate_checksum_ex(psrc, len);

        cmd_send_tp.d0 = adr & 0xffff;    /* addrss space[0,64K)  */    
        cmd_send_tp.d1 = len;
        cmd_send_tp.d3 = BURN_W;
        cmd_send_tp.d2 = (unsigned short) combChk;
        cmd_send_tp.d4 = (unsigned short) (combChk >> 16);
        cmd_send_tp.d5 = (adr >> 16) & 0xffff;

        retry = 0;
        while (++retry <= 3) {
            ret = semi_touch_write_bytes(TP_WR_BUFF_ADDR, psrc, len);
              if(SEMI_DRV_ERR_OK != ret) continue;

            ack_from_tp.id = CMD_NA;
            ret = cmd_send_to_tp(&cmd_send_tp, &ack_from_tp, 7500);
            if(SEMI_DRV_ERR_OK != ret) continue;
            //check_break_if_fail(ret, NULL);

            break;
        }
        
        left -= len;
        adr  += len;
        psrc += len;
        check_break_if_fail(ret, NULL);
    }

    return ret;
}

int semi_get_backup_pid(unsigned int *id)
{
    int ret;

    ret = semi_touch_enter_burn_mode();
    check_return_if_fail(ret, NULL);

    return semi_touch_bulk_read((unsigned char *)id, VID_PID_BACKUP_ADDR, 4);
}

int semi_touch_burn_erase(void)
{
    int ret = SEMI_DRV_ERR_OK;

#if TYPE_OF_IC(SEMI_TOUCH_IC) == TYPE_OF_IC(SEMI_TOUCH_5816)
    struct m_ctp_cmd_std_t cmd_send_tp;
    struct m_ctp_rsp_std_t ack_from_tp;

    cmd_send_tp.id = CMD_FLASH_ERASE;
    cmd_send_tp.d0 = 0x01;

    ret = cmd_send_to_tp(&cmd_send_tp, &ack_from_tp, 30000);
    check_return_if_fail(ret, NULL);
#endif

    return ret;
}

unsigned int config_to_vid_pid(unsigned char *ptcfg, unsigned int len)
{
    unsigned int upd_vid_pid;
#if TYPE_OF_IC(SEMI_TOUCH_IC) == TYPE_OF_IC(SEMI_TOUCH_5816)
    ushort cfgAddr = (ushort)((ptcfg[0x39] << 8) + ptcfg[0x38]);
    if(cfgAddr + 4 < len)
    {
        ptcfg = ptcfg + cfgAddr;
    }
#endif

    upd_vid_pid = ptcfg[4];
    upd_vid_pid = (upd_vid_pid << 8) | ptcfg[3];
    upd_vid_pid = (upd_vid_pid << 8) | ptcfg[2];
    upd_vid_pid = (upd_vid_pid << 8) | ptcfg[1];

    return upd_vid_pid;
}

static int semi_touch_check_cfg_update(unsigned char *parray, unsigned int cfg_single_len, unsigned int cfg_num, unsigned int force_update)
{
    int index, ret = SEMI_DRV_ERR_OK;
    int idx_active;
    unsigned int upd_vid_pid;
    unsigned char *ptcfg = parray;

    if(0 == cfg_num) {
        return SEMI_DRV_ERR_OK;
    }
    if (st_dev.vid_pid == 0) {    /* no available version information */
        return SEMI_DRV_ERR_OK;
    }

    idx_active = -1;
    for (index = 0; index < cfg_num; index++) {
        upd_vid_pid = config_to_vid_pid(ptcfg, cfg_single_len);
       
        if((st_dev.vid_pid & 0xffffff00) == (upd_vid_pid & 0xffffff00)) {
            kernel_log_d("tp vid_pid = 0x%08x, udp vid_pid = 0x%08x\r\n", st_dev.vid_pid, upd_vid_pid);
            if ((st_dev.vid_pid < upd_vid_pid) || force_update) {
                idx_active = index;
                break;
            }
        }

        ptcfg = ptcfg + cfg_single_len;
    }

    if(idx_active >= 0) {
        ret = semi_touch_enter_burn_mode();
        check_return_if_fail(ret, NULL);

        ret = semi_touch_burn_erase();
        check_return_if_fail(ret, NULL);

        ret = semi_touch_bulk_write(ptcfg, CFG_ROM_ADDRESS, cfg_single_len);
        check_return_if_fail(ret, NULL);

        st_dev.vid_pid = upd_vid_pid;
    }else { //is latest version

    }

    return ret;
}

int semi_touch_check_boot_update(unsigned char *pdata, unsigned int len, unsigned int* vlist, unsigned int n_match, unsigned int force_update)
{
    int ret = SEMI_DRV_ERR_OK;
    int k, idx_active;
    unsigned short upd_boot_ver = 0;

    idx_active = -1;
    upd_boot_ver = (pdata[0x3f] << 8) + pdata[0x3e];
    for (k = 0; k < n_match; k++) {
        if ((vlist[k] & 0xffffff00) == (st_dev.vid_pid & 0xffffff00)) {
            if((st_dev.fw_ver < upd_boot_ver) || force_update){
                kernel_log_d("tp vid_pid = 0x%08x, udp vid_pid = 0x%08x\r\n", st_dev.vid_pid, vlist[k]);
                idx_active = k;
                break;
            }
        }
    }

    kernel_log_d("tp boot ver = 0x%x, udp boot ver = 0x%x\r\n", st_dev.fw_ver, upd_boot_ver);
    /*
        If we need to compare versions, only update the newer version
    */
    if(idx_active >= 0) {
        ret = semi_touch_enter_burn_mode();
        check_return_if_fail(ret, NULL);

        ret = semi_touch_bulk_write((unsigned char *)pdata, 0x00000000, len);
        check_return_if_fail(ret, NULL);

        st_dev.fw_ver = upd_boot_ver;
    }
    
    return ret;
}

/*
    Return Value:
        0x01: reject caller's request
        -1  : some error
        0x00: successfull
*/
int semi_touch_update_updfile(const unsigned char *pdata, unsigned int len, unsigned int force_update)
{
    int ret = SEMI_DRV_ERR_OK;
    unsigned int cfg_num, cfg_single_len;
    unsigned int offset, cfg_offset;
    unsigned int *vlist;
    struct chsc_updfile_header *upd_header;

    kernel_log_d("check if firmware need update, product pid_vid = 0x%08x, force = %d\r\n", st_dev.vid_pid, force_update);

    if (len < sizeof(struct chsc_updfile_header)) { 
        return -SEMI_DRV_INVALID_PARAM;
    }

    upd_header = (struct chsc_updfile_header *) pdata;

    if (upd_header->sig != 0x43534843) {
        return -SEMI_DRV_ERR_NOT_MATCH;
    }

    if(0 == upd_header->n_cfg){ 
        return -SEMI_DRV_INVALID_PARAM;
    }

    cfg_num = upd_header->n_cfg;
    offset  = (upd_header->n_cfg * 4) + sizeof(struct chsc_updfile_header);
    cfg_offset = offset;
    cfg_single_len = upd_header->len_cfg / cfg_num;

    if ((offset + upd_header->len_cfg + upd_header->len_boot) != len) {
        return -SEMI_DRV_INVALID_PARAM;
    }

    offset = offset + upd_header->len_cfg;
    vlist  = (unsigned int *) (pdata + sizeof(struct chsc_updfile_header));

    if(SEMI_DRV_ERR_OK == ret) { //check if config need update
        ret = semi_touch_check_cfg_update((unsigned char *) (pdata + cfg_offset), cfg_single_len, cfg_num, force_update);
    }

    if(SEMI_DRV_ERR_OK == ret){ //check if boot need update
        ret = semi_touch_check_boot_update((unsigned char*)(pdata + offset), upd_header->len_boot, vlist, upd_header->n_match, force_update);
    }

    return ret;
}

int semi_touch_check_and_update(const unsigned char* udp, unsigned int len)
{
    int ret = -EINVAL; 
    unsigned char bootCheckOk = 0;
    unsigned int backup_vid_pid;

    st_dev.fw_ver = st_dev.vid_pid = 0;
    semi_touch_start_up_check(&bootCheckOk);

    if(!bootCheckOk)
    {
        ret = semi_get_backup_pid(&backup_vid_pid);
        check_return_if_fail(ret, NULL);
        st_dev.vid_pid = backup_vid_pid;
    }

    if((0 != st_dev.vid_pid) && (0xffffffff != st_dev.vid_pid))
    {
        ret = semi_touch_update_updfile(udp, len, bootCheckOk ? 0 : 1);
        check_return_if_fail(ret, NULL);
    }
    else //we don't know what kind if product it is
    {
        ret = -SEMI_DRV_ERR_NOT_MATCH;
        kernel_log_d("opps! what happeded...\r\n");
    }

    return ret;
}
/*
    Get and check boot up information of touch IC
*/
#if SEMI_TOUCH_BOOTUP_UPDATE_EN
int semi_touch_bootup_update_check(void)
{
    return semi_touch_check_and_update(upd_data, sizeof(upd_data));
}
#endif

#if SEMI_TOUCH_ONLINE_UPDATE_EN
int semi_touch_online_update_check(char* file_path)
{
    mm_segment_t old_fs;
    unsigned int file_size, file_sig;
    const unsigned char* file_buffer = NULL;
    static struct file *file = NULL;
    loff_t fpos = 0;

    file = filp_open(file_path, O_RDWR, 0);
    if(IS_ERR(file)) 
    {
        kernel_log_d("open file %s error\n", file_path);
        return -EINVAL;
    }

    old_fs = get_fs();
    set_fs(KERNEL_DS);

    fpos = 0;
    vfs_llseek(file, 0, SEEK_SET);
    vfs_read(file, (char*)&file_sig, sizeof(file_sig), &fpos);
    if(0x43534843 != file_sig) 
    {
        filp_close(file, NULL);
        kernel_log_d("file validation check fail...\n");
        return -EINVAL;
    }

    file_sig = 0;
    file_size = vfs_llseek(file, 0, SEEK_END);
    file_buffer = kmalloc(file_size, GFP_KERNEL);
    
    fpos = 0;
    vfs_llseek(file, 0, SEEK_SET);
    if(file_size == vfs_read(file, (char*)file_buffer, file_size, &fpos))
    {
        semi_touch_check_and_update(file_buffer, file_size);
        semi_touch_mode_init(&st_dev);

        //file->f_op->llseek(file, 0, SEEK_SET);
        //file->f_op->write(file, (char*)&file_sig, sizeof(file_sig), &file->f_pos);
    }

    kfree(file_buffer);
    set_fs(old_fs);
    filp_close(file, NULL);

    return SEMI_DRV_ERR_OK;
}
#endif //SEMI_TOUCH_ONLINE_UPDATE_EN
