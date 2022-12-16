/*
* @file SOF_block.cpp
*
* @brief MTD device handling of STM32 internal flash memory.
*
*
* History:
*/

#include <stdio.h>
#include <assert.h>
#include "flash_device.h"
#include <string.h>
#include <stdlib.h>

#include "FreeRTOS.h"
#include "task.h"


#define LOCAL_DEBUG 0 // turn on local debug

#define DCRLF	"\r\n"

#if LOCAL_DEBUG
#define DPRINTF printf
#define EPRINTF printf
#define DUMP_BLOCK dump_block
#define DASSERT assert
#else
#define DPRINTF(...)
#define EPRINTF(...)
#define DUMP_BLOCK(...)
#define DASSERT(...)
#endif

static uint16_t checksum(const uint8_t* data, int count);

#define SYNC_MARK_BYTE_IN_LEN	0x07	// signature mark for upper byte in the storage_len

#define RESERVED_BLOCK_INFO_SIZE sizeof(BlockInfo_t)

typedef struct {
    uint16_t for_storage;
    uint16_t for_info;
} BlockChecksum_t;

typedef struct {
    BlockChecksum_t		csum;
    uint32_t			storage_len;
} BlockInfo_t;

typedef struct {
    uint32_t	begin_offset;
    uint32_t	len;
    uint16_t	storage_csum;
} StorageInfo_t;



/*-----------------------------------------------------------------------------------------------------*/
/* Flash block device interface */
/*-----------------------------------------------------------------------------------------------------*/

static bool SOF_BlockHandle_is_writable(const SOF_BlockHandle* handle)
{
	return handle->write_mode_;
}

static uint32_t SOF_BlockHandle_total_physical_block_size(const SOF_BlockHandle* handle)
{
    return SOF_dev_info(handle->hdev_)->sec_size;
}

static bool get_block_info(const SOF_BlockHandle_t handle, size_t seq, BlockInfo_t *info, uint32_t *loc_offset)
{
    uint32_t check_pos = ((seq+1) * sizeof(BlockInfo_t));
    uint32_t info_pos = 0;

    DASSERT(check_pos < SOF_BlockHandle_total_physical_block_size(handle));
    if (check_pos >= SOF_BlockHandle_total_physical_block_size(handle))
        return false;

    *loc_offset = info_pos = SOF_BlockHandle_total_physical_block_size(handle) - check_pos;

    // checksum in the first word
    *((uint32_t*)&info->csum) = SOF_dev_read_word(handle->hdev_, info_pos);
    // storage len in the next word
    info->storage_len = SOF_dev_read_word(handle->hdev_, info_pos + 4);

    return true;
}

static bool is_empty_block_info(BlockInfo_t *info)
{
    uint8_t *p = (uint8_t*)info;

    for (size_t i = 0; i < sizeof(BlockInfo_t); ++i)
        if (p[i] != SOF_ERASED_BYTE_VALUE)
            return false;

    return true;
}

static bool is_valid_block_info(BlockInfo_t *info)
{
    uint16_t csum = checksum((uint8_t*)&info->storage_len, 4);

    if (SYNC_MARK_BYTE_IN_LEN != (info->storage_len >> 24))
    {
        EPRINTF("no sync mark in storage_len=%#x"DCRLF,info->storage_len);
        return false;
    }

    if (csum != info->csum.for_info)
    {
        EPRINTF("CSUM mismatch %#x %#x"DCRLF,csum, info->csum.for_info);
        return false;
    }

    return true;
}

static bool get_empty_info_location(const SOF_BlockHandle_t handle, uint32_t *loc_offset)
{
    BlockInfo_t info = {.csum = {.for_storage = 0, .for_info = 0}, .storage_len = 0};
    uint32_t pos = 0;

    for (size_t seq = 0; get_block_info(handle, seq, &info, &pos); ++seq)
    {        //DPRINTF("[%u] len=%#x pos=%u"DCRLF,seq, info.storage_len, pos);
        if (is_empty_block_info(&info))
        {
            *loc_offset = pos;
            return true;
        }
    }
    return false;
}

static SOF_Error_t probe_active_storage_info(const SOF_BlockHandle_t handle, StorageInfo_t *storage_info)
{
    BlockInfo_t info = {.csum = {.for_storage = 0, .for_info = 0}, .storage_len = 0};
    BlockInfo_t last_info = {.csum = {.for_storage = 0, .for_info = 0}, .storage_len = 0};
    uint32_t pos = 0, storage_len_sum = 0;

    for (size_t seq = 0; get_block_info(handle, seq, &info, &pos); ++seq)
    {
        if (is_empty_block_info(&info))
        {
            if (seq == 0)
                return kSOF_ErrNoInfo;
            break;
        }
        if (!is_valid_block_info(&info))
        {
            if (storage_info->begin_offset + storage_info->len == pos)
            {
                DPRINTF("data is full: %u"DCRLF,storage_info->begin_offset + storage_info->len);
                break;
            }

            EPRINTF("invalid block at %u"DCRLF,pos);
            return kSOF_ErrBadBlock;
        }

        storage_len_sum += info.storage_len & 0x00FFFFFF;
        last_info = info;
    }

    uint32_t storage_len = last_info.storage_len & 0x00FFFFFF;

    storage_info->begin_offset = storage_len_sum - storage_len;
    storage_info->len = storage_len;
    storage_info->storage_csum = last_info.csum.for_storage;

    return kSOF_ErrNone;
}


#if LOCAL_DEBUG
static void dump_block(SOF_BlockHandle_t handle)
{
    DPRINTF("sector(%u)"DCRLF, SOF_dev_info(handle->hdev_)->sec_no);
    DPRINTF("	offset               =%u"DCRLF, handle->cur_pos_);
    DPRINTF("	writemode            =%d"DCRLF, handle->write_mode_);
    DPRINTF("	storage_max_offset   =%u"DCRLF, handle->storage_max_offset_);
    DPRINTF("	storage_begin_offset =%u"DCRLF, handle->storage_begin_offset_);
    DPRINTF("	storage_end_offset   =%u"DCRLF, handle->storage_end_offset_);
    DPRINTF("	free=%u total=%u"DCRLF,SOF_block_get_free_size(handle), handle->total_physical_block_size());
}
#endif

size_t SOF_block_get_free_size(SOF_BlockHandle_t handle)
{
    DASSERT(handle != NULL);
    if (handle->storage_end_offset_ <= handle->storage_max_offset_-RESERVED_BLOCK_INFO_SIZE)
        return (handle->storage_max_offset_- RESERVED_BLOCK_INFO_SIZE) - handle->storage_end_offset_;
    else
    {
        return 0;
    }
}

uint32_t SOF_block_storage_size(SOF_BlockHandle_t handle)
{
    DASSERT(handle != NULL);
    return handle->storage_end_offset_ - handle->storage_begin_offset_;
}

static uint16_t checksum(const uint8_t* data, int count)
{
    // Fletcher's checksum algorithm
    uint16_t sum1 = 0;
    uint16_t sum2 = 0;
    int index;

    for( index = 0; index < count; ++index )
    {
        sum1 = (sum1 + data[index]) % 255;
        sum2 = (sum2 + sum1) % 255;
    }

    return (sum2 << 8) | sum1;
}

static uint16_t compute_storage_checksum(SOF_BlockHandle_t handle)
{
    uint8_t *addr = SOF_dev_get_hw_addr(handle->hdev_);

    return checksum(addr+handle->storage_begin_offset_, SOF_block_storage_size(handle));
}

static bool write_storage_info(SOF_BlockHandle_t handle)
{
    BlockInfo_t cs;

    cs.storage_len = (SYNC_MARK_BYTE_IN_LEN << 24) | SOF_block_storage_size(handle);
    cs.csum.for_info = checksum((uint8_t*)&cs.storage_len, 4);
    cs.csum.for_storage = compute_storage_checksum(handle);

    DPRINTF("write %#x at %#x"DCRLF,*((uint32_t*)&cs.csum), handle->storage_max_offset_);
    if (SOF_dev_write_word(handle->hdev_, handle->storage_max_offset_, *((uint32_t*)&cs.csum)) < 0)
        return false;

    if (SOF_dev_write_word(handle->hdev_, handle->storage_max_offset_+4, *((uint32_t*)&cs.storage_len)) < 0)
        return false;

    return true;
}

static bool create_empty_storage(SOF_DevHandle_t hdev, uint8_t sector_index)
{
    SOF_BlockHandle handle_data = {
    		.cur_pos_ = 0,
    		.hdev_ = 0,
			.storage_begin_offset_ = 0,
			.storage_end_offset_ = 0,
			.storage_max_offset_ = 0,
			.write_mode_ = 0};

    handle_data.hdev_ = hdev;

    uint32_t info_begin_offset = 0;

    if (!get_empty_info_location(&handle_data, &info_begin_offset))
    {
        EPRINTF("no info"DCRLF);
        SOF_block_close(&handle_data);
        return false;
    }

    handle_data.storage_max_offset_ = info_begin_offset;
    handle_data.storage_begin_offset_ = 0;
    handle_data.storage_end_offset_ = handle_data.storage_begin_offset_;
    handle_data.cur_pos_ = handle_data.storage_begin_offset_;

    DPRINTF("storage created: begin=%d end=%d free=%d"DCRLF,
            handle_data.storage_begin_offset_, handle_data.storage_end_offset_, SOF_block_get_free_size(&handle_data));

    return write_storage_info(&handle_data);
}


bool SOF_block_format(uint8_t sector_index)
{
    if (!SOF_dev_is_valid_sector(sector_index))
    {
        DPRINTF("invalid sector_index=%d"DCRLF, sector_index);
        return false;
    }

    SOF_DevHandle_t hdev = SOF_dev_open(sector_index);

    if (hdev == SOF_INVALID_HANDLE)
    {
        DPRINTF("SOF_dev_open(%d) failed"DCRLF, sector_index);
        return false;
    }

    DPRINTF("Flash erase %d"DCRLF, sector_index);
    SOF_dev_erase(hdev);
    bool result = create_empty_storage(hdev, sector_index);
    SOF_dev_close(hdev);

    return result;
}

SOF_BlockHandle_t SOF_block_open_storage(uint8_t sector_index, SOF_Error_t *err)
{
    if (!SOF_dev_is_valid_sector(sector_index))
    {
        DPRINTF("invalid sector_index=%d"DCRLF, sector_index);
        return false;
    }

    SOF_DevHandle_t hdev = SOF_dev_open(sector_index);

    if (hdev == SOF_INVALID_HANDLE)
    {
        DPRINTF("SOF_dev_open(%d) failed"DCRLF, sector_index);
        return false;
    }

    SOF_BlockHandle_t handle = malloc(sizeof(SOF_BlockHandle));

    handle->hdev_ = hdev;

    StorageInfo_t storage_info = { .begin_offset = 0, .len = 0, .storage_csum = 0};

    if ((*err=probe_active_storage_info(handle, &storage_info)) != kSOF_ErrNone)
    {
        free(handle);
        return NULL;
    }

    uint32_t info_begin_offset = 0;

    if (!get_empty_info_location(handle, &info_begin_offset))
    {
        *err = kSOF_ErrBadBlock;
        free(handle);
        return NULL;
    }

    // set max offset that storage grows.
    handle->storage_max_offset_ = info_begin_offset;

    handle->storage_begin_offset_ = storage_info.begin_offset;
    handle->storage_end_offset_ = storage_info.begin_offset + storage_info.len;

    handle->cur_pos_ = handle->storage_begin_offset_;

    DPRINTF("open for read: begin=%d end=%d len=%d free=%d"DCRLF,
            handle->storage_begin_offset_, handle->storage_end_offset_, storage_info.len,
            SOF_block_get_free_size(handle));
    if (compute_storage_checksum(handle) != storage_info.storage_csum)
    {
        EPRINTF("checksum error %#x != %#x"DCRLF, compute_storage_checksum(handle), storage_info.storage_csum);
        *err = kSOF_ErrDataCurrupted;
        free(handle);
        return NULL;
    }

    DUMP_BLOCK(handle);
    *err = kSOF_ErrNone;

    return handle;
}

SOF_BlockHandle_t SOF_block_create_storage(uint8_t sector_index, SOF_Error_t *err)
{
    if (!SOF_dev_is_valid_sector(sector_index))
    {
        DPRINTF("invalid sector_index=%d"DCRLF, sector_index);
        return false;
    }

    SOF_DevHandle_t hdev = SOF_dev_open(sector_index);

    if (hdev == SOF_INVALID_HANDLE)
    {
        DPRINTF("SOF_dev_open(%d) failed"DCRLF, sector_index);
        return false;
    }
    SOF_BlockHandle_t handle = malloc(sizeof(SOF_BlockHandle));

    handle->hdev_ = hdev;

    StorageInfo_t storage_info;
    if ((*err=probe_active_storage_info(handle, &storage_info)) != kSOF_ErrNone)
    {
    	free(handle);
        return NULL;
    }

    uint32_t info_begin_offset;

    if (!get_empty_info_location(handle, &info_begin_offset))
    {
        *err = kSOF_ErrBadBlock;
        free(handle);
        return NULL;
    }
    // set max offset that storage grows.
    handle->storage_max_offset_ = info_begin_offset;

    // writing position is just after previous storage
    handle->storage_begin_offset_ = storage_info.begin_offset + storage_info.len;
    handle->storage_end_offset_ = handle->storage_begin_offset_;

    handle->cur_pos_ = handle->storage_begin_offset_;
    handle->write_mode_ = true;
    DPRINTF("open for write: begin=%d end=%d free=%d"DCRLF,
            handle->storage_begin_offset_, handle->storage_end_offset_, SOF_block_get_free_size(handle));

    DUMP_BLOCK(handle);
    *err = kSOF_ErrNone;

    return handle;
}

bool SOF_block_close(SOF_BlockHandle_t handle)
{
    bool r = true;

    DASSERT(handle != NULL);
    if (handle->write_mode_)
        r = (bool)write_storage_info(handle);
    SOF_dev_close(handle->hdev_);
    free(handle);

    return r;
}

uint8_t *SOF_block_base_addr(SOF_BlockHandle_t handle)
{
    DASSERT(handle != NULL);
    return SOF_dev_get_hw_addr(handle->hdev_) + handle->cur_pos_;
}

bool SOF_block_putc(SOF_BlockHandle_t handle, uint8_t c)
{
    DASSERT(handle != NULL);
    DASSERT(handle->is_writable());

    if (SOF_block_get_free_size(handle) == 0)
    {
        DPRINTF("no free space"DCRLF);
        DUMP_BLOCK(handle);
        return false;
    }

    bool b = SOF_dev_write_byte(handle->hdev_, handle->cur_pos_, c) != -1;
    if (b)
    {
        handle->cur_pos_++;
        handle->storage_end_offset_++;
    }
    return b;
}

size_t SOF_block_write(SOF_BlockHandle_t handle, const uint8_t *p, size_t p_size)
{
    size_t i;

    for (i = 0; i < p_size; ++i)
        if (SOF_block_putc(handle, *p++) != true)
            return i;

    return i;
}

bool SOF_block_getc(SOF_BlockHandle_t handle, uint8_t *c)
{
    DASSERT(handle != NULL);
    DASSERT(handle->is_writable());

    if (handle->cur_pos_ >= handle->storage_end_offset_)
    {
        DPRINTF("end of data\n"DCRLF);
        DUMP_BLOCK(handle);

        return false;
    }

    *c = SOF_dev_read_byte(handle->hdev_, handle->cur_pos_++);

    return true;
}

size_t SOF_block_read(SOF_BlockHandle_t handle, uint8_t *p, size_t p_size)
{
    size_t i;
    if (handle->storage_end_offset_-handle->cur_pos_ < p_size)
    	return 0;
    for (i = 0; i < p_size; ++i)
        if (!SOF_block_getc(handle, p++))
            break;

    return i;
}

SOF_Error_t SOF_block_get_statics(uint8_t sector_index, SOF_Statics_t *stat)
{
    if (!SOF_dev_is_valid_sector(sector_index))
    {
        DPRINTF("invalid sector_index=%d"DCRLF, sector_index);
        return kSOF_ErrParam;
    }

    SOF_Error_t err;
    SOF_BlockHandle_t hblk = SOF_block_open_storage(sector_index, &err);

    if (hblk == NULL)
    {
        DPRINTF("SOF_block_open_storage(%d) failed"DCRLF, sector_index);
        return err;
    }

    stat->data_addr = SOF_block_base_addr(hblk);
    stat->data_size = SOF_block_storage_size(hblk);
    stat->free_size = SOF_block_get_free_size(hblk);

    SOF_block_close(hblk);

    return kSOF_ErrNone;
}

const SOF_SectorSpec_t *SOF_block_get_info(uint8_t sector_index)
{
    if (!SOF_dev_is_valid_sector(sector_index))
    {
        DPRINTF("invalid sector_index=%d"DCRLF, sector_index);
        return NULL;
    }

    return SOF_dev_info_by_index(sector_index);
}



/*-----------------------------------------------------------------------------------------------------*/
/* Flash device interface */
/*-----------------------------------------------------------------------------------------------------*/
#define STM32F405xx

#if defined(STM32F401xE) || defined(STM32F411xE) || defined(STM32F407xx) || defined(STM32F405xx)
static const SOF_SectorSpec_t _sec_spec[] = {
    {FLASH_Sector_0, 0x08000000, 16 * 1024},
    {FLASH_Sector_1, 0x08004000, 16 * 1024},
    {FLASH_Sector_2, 0x08008000, 16 * 1024},
    {FLASH_Sector_3, 0x0800C000, 16 * 1024},
    {FLASH_Sector_4, 0x08010000, 64 * 1024},
    {FLASH_Sector_5, 0x08020000, 128 * 1024},
    {FLASH_Sector_6, 0x08040000, 128 * 1024},
    {FLASH_Sector_7, 0x08060000, 128 * 1024},
    {FLASH_Sector_8, 0x08080000, 128 * 1024},
    {FLASH_Sector_9, 0x080A0000, 128 * 1024},
    {FLASH_Sector_10, 0x080C0000, 128 * 1024},
    {FLASH_Sector_11, 0x080E0000, 128 * 1024},
};
#else
#error "Not supported device"
#endif

#define N_SECTOR_SPEC			(sizeof(_sec_spec)/sizeof(_sec_spec[0]))

#define SECTOR_NO(sector)		_sec_spec[sector].sec_no
#define SECTOR_ADDR(sector)	    _sec_spec[sector].sec_addr
#define SECTOR_SIZE(sector)		_sec_spec[sector].sec_size


static inline size_t handle_to_sector_index(SOF_DevHandle_t hdev)
{
    DASSERT(hdev < N_SECTOR_SPEC);
    return hdev;
}

//const SOF_SectorSpec_t *SOF_dev_info(uint8_t sector_index)
//{
//    DASSERT(sector_index < N_SECTOR_SPEC);
//    return &_sec_spec[sector_index];
//}

int SOF_dev_is_valid_sector(uint8_t sector_index)
{
    return sector_index < N_SECTOR_SPEC;
}

const SOF_SectorSpec_t *SOF_dev_info_by_index(uint8_t sector_index)
{
    DASSERT(SOF_dev_is_valid_sector(sector_index));
    return &_sec_spec[sector_index];
}

const SOF_SectorSpec_t *SOF_dev_info(SOF_DevHandle_t hdev)
{
    uint8_t sector_index = handle_to_sector_index(hdev);

    return SOF_dev_info_by_index(sector_index);
}

SOF_DevHandle_t SOF_dev_open(uint8_t sector_index)
{
    DASSERT(sector_index < N_SECTOR_SPEC);
    return (SOF_DevHandle_t)sector_index;
}

void SOF_dev_close(SOF_DevHandle_t hdev)
{
}

uint8_t *SOF_dev_get_hw_addr(SOF_DevHandle_t hdev)
{
    uint8_t sector_index = handle_to_sector_index(hdev);

    return (uint8_t *)SECTOR_ADDR(sector_index);
}


void SOF_dev_erase(SOF_DevHandle_t hdev)
{
    uint8_t sector_index = handle_to_sector_index(hdev);
    uint32_t error = 0;

    FLASH_Unlock();
    FLASH_EraseSector(SECTOR_NO(sector_index), VoltageRange_3);
    FLASH_Lock();
    DPRINTF("FLASH_Erase_Sector ok"DCRLF);
}


uint32_t SOF_dev_read_word(SOF_DevHandle_t hdev, uint32_t offset_addr)
{
    uint8_t sector_index = handle_to_sector_index(hdev);
    uint32_t src = SECTOR_ADDR(sector_index) + offset_addr;

    DASSERT((offset_addr%sizeof(uint32_t)) == 0);

    return *(volatile uint32_t*)src;
}

int SOF_dev_write_word(SOF_DevHandle_t hdev, uint32_t offset_addr, uint32_t data)
{
    uint8_t sector_index = handle_to_sector_index(hdev);
    uint32_t dst = SECTOR_ADDR(sector_index) + offset_addr;

    DASSERT((offset_addr%sizeof(uint32_t)) == 0);
    taskDISABLE_INTERRUPTS();
    taskENTER_CRITICAL();
    FLASH_ClearFlag(FLASH_FLAG_PGPERR);
    FLASH_ClearFlag(FLASH_FLAG_PGSERR);
    FLASH_Unlock();
    if (FLASH_ProgramWord(dst, data) != FLASH_COMPLETE)
    {
        DPRINTF("FLASH_ProgramWord failed: %#x"DCRLF, dst);
        FLASH_Lock();
        return -1;
    }

    FLASH_Lock();
    taskEXIT_CRITICAL();
    taskENABLE_INTERRUPTS();

    if (data != SOF_dev_read_word(hdev, offset_addr)) {
        DPRINTF("addr=%x %#04x %#04x"DCRLF, dst, data, SOF_dev_read_word(hdev, offset_addr));
        return -1;
    }

    return 0;
}


uint8_t SOF_dev_read_byte(SOF_DevHandle_t hdev, uint32_t offset_addr)
{
    uint8_t sector_index = handle_to_sector_index(hdev);
    uint32_t src = SECTOR_ADDR(sector_index) + offset_addr;

    return *(volatile uint8_t*)src;
}


int SOF_dev_write_byte(SOF_DevHandle_t hdev, uint32_t offset_addr, uint8_t data)
{
    uint8_t sector_index = handle_to_sector_index(hdev);
    uint32_t dst = SECTOR_ADDR(sector_index) + offset_addr;

    taskDISABLE_INTERRUPTS();
    taskENTER_CRITICAL();
    FLASH_ClearFlag(FLASH_FLAG_PGPERR);
    FLASH_ClearFlag(FLASH_FLAG_PGSERR);
    FLASH_Unlock();
    if (FLASH_ProgramByte(dst, data) != FLASH_COMPLETE)
    {
        DPRINTF("FLASH_ProgramWord failed: %#x"DCRLF, dst);
        FLASH_Lock();
        return -1;
    }
    FLASH_Lock();
    taskEXIT_CRITICAL();
    taskENABLE_INTERRUPTS();


    if (data != SOF_dev_read_byte(hdev, offset_addr)) {
        DPRINTF("addr=%x %#02x %#02x"DCRLF, dst, data, SOF_dev_read_byte(hdev, offset_addr));
        return -1;
    }

    return 0;
}







