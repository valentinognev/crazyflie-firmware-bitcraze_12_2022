/*
* @file SOFBlock.cpp
* @author hillkim7@gmail.com
*
* @brief Simple storage implementation on internal MCU flash memory.
*
*/

#include <string.h>
#include <stdlib.h>
#include <flash_API.h>

bool SOFBlock_is_open(SOFBlock* sofblock)
{
	return sofblock->hblock_ != NULL;
}

bool SOFBlock_format(const uint8_t sector_index)
{
    return SOF_block_format(sector_index);
}

bool SOFBlock_get_stat(const uint8_t sector_index, SOF_Statics_t* statics )
{
    return SOF_block_get_statics(sector_index, statics) ==  kSOF_ErrNone;
}


SOF_Error_t SOFBlock_open_write(SOFBlock* sofblock, const uint8_t sector_index )
{
    if (SOFBlock_is_open(sofblock))
    {
        return kSOF_ErrBusyBlock;
    }

    SOF_Error_t err;

    sofblock->hblock_ = SOF_block_create_storage(sector_index, &err);
    sofblock->hblock_->write_mode_ = 1;

    return err;
}

void SOFBlock_close_write(SOFBlock* sofblock)
{
    if (sofblock->hblock_ != NULL) {
        SOF_block_close(sofblock->hblock_);
        sofblock->hblock_ = NULL;
    }
}
bool SOFBlock_write_byte_data(SOFBlock* sofblock, uint8_t c )
{
    if (!SOFBlock_is_open(sofblock))
    {
        return false;
    }

    return SOF_block_putc(sofblock->hblock_, c);
}

size_t SOFBlock_write_data(SOFBlock* sofblock, const uint8_t *p, size_t p_size )
{
    if (!SOFBlock_is_open(sofblock))
    {
        return false;
    }

    return SOF_block_write(sofblock->hblock_, p, p_size);
}

size_t SOFBlock_get_free_size(SOFBlock* sofblock)
{
    if (!SOFBlock_is_open(sofblock))
    {
        return 0;
    }

    return SOF_block_get_free_size(sofblock->hblock_);
}


SOF_Error_t SOFBlock_open_read(SOFBlock* sofblock, const uint8_t sector_index )
{
    if (SOFBlock_is_open(sofblock))
    {
        return kSOF_ErrBusyBlock;
    }

    SOF_Error_t err;

    sofblock->hblock_ = SOF_block_open_storage(sector_index, &err);
    sofblock->hblock_->write_mode_ = 0;

    return err;
}

uint8_t * SOFBlock_get_physical_data_addr(SOFBlock* sofblock)
{
    if (!SOFBlock_is_open(sofblock))
    {
        return NULL;
    }

    return SOF_block_base_addr(sofblock->hblock_);
}

size_t SOFBlock_get_data_size(SOFBlock* sofblock)
{
    if (!SOFBlock_is_open(sofblock))
    {
        return 0;
    }

    return SOF_block_storage_size(sofblock->hblock_);
}

bool SOFBlock_read_byte_data(SOFBlock* sofblock, uint8_t *c )
{
    if (!SOFBlock_is_open(sofblock))
    {
        return false;
    }

    return SOF_block_getc(sofblock->hblock_, c);
}

size_t SOFBlock_read_data(SOFBlock* sofblock, uint8_t *p, size_t p_size )
{
    if (!SOFBlock_is_open(sofblock))
    {
        return 0;
    }

    return SOF_block_read(sofblock->hblock_, p, p_size);
}

void SOFBlock_close_read(SOFBlock* sofblock)
{
    if (sofblock->hblock_ != NULL) {
        SOF_block_close(sofblock->hblock_);
        sofblock->hblock_ = NULL;
    }
}


