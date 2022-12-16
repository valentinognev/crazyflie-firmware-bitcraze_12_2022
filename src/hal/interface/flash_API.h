/**
 * @file SOFBlock.h
 *
 * @author hillkim7@gmail.com
 * @brief Simple storage implementation on internal MCU flash memory.
 *
 * The SOF in SOFBlock is abbreviation of "Storage On Flash".
 * The purpose of SOFBlock class is to provide a way to write data on flash memory
 * in the same way of file handling class in the file system.
 * It manages a chunk of data on the Flash memory efficiently by minimizing flash erase operation as little as possible.
 * Note: Currently it only supports STM32F4xx series platforms.
 *       - NUCLEO-F401RE, NUCLEO-F411RE, Seeed Arch Max
 * The STM32 F4xx series from ST have plenty of internal Flash memory inside MCU core.
 * For example STM32 401RE has 512Kbyts Flash.
 * Typical size of firmware file is less than 256KB, so remaining area is free to use.
 * The simplest way of flash utilization as data storage is to use a chunk of Flash area as an unit of storage.
 * A block of flash is called sector in STM32F4xx domain. It requires to erase a sector before update bits in flash.
 *
 * Conceptually it is quite simple.
 * Here is typical write operation:
 * 1) Erase sector #n
 * 2) Write data to sector #n
 * Read operation:
 * 1) Just read physical memory address of sector #n
 *    The base physical address of STM32 flash is 0x08000000.
 *
 * There may be inefficiency in this flash usage scenario when size of data is too small compared with sector size.
 * The size of sectors from #5 to #7 of STM32-F4xx Flash is 128KB. For example, if I only need to maintain 1KB data,
 * whenever I need to update data I need to erase whole 128KB of sector.
 * This produces two problems.
 * One is time consumption of the erase operation. The operation of ERASE128KB takes 1~4 seconds long.
 * The other is related to lifetime of Flash memory.
 * More you erase and write and lifetime of flash is shorter.
 *
 * To overcome such problems, here simple flash management algorithm is used for.
 * By tracking data offset and size it can hold multiple data in a sector.
 * Bear in mind that is impossible rewriting data on Flash.
 * Keeping tracking data along with data itself without frequent erase operation is crucial.
 * To do this, data itself is growing from low address.
 * On the other hand tracking data is growing down from high address.
 * Let's assume the size of data is 1KB and store it in sector #6 which address range is from 0x08040000 to 0x0805ffff.
 * +-------------+----------------------------------------------------------------------+-----+
 * <data>                                                                        <tracking data>
 * +-------------+----------------------------------------------------------------------+-----+
 * data grows ->                                                           <- tracking data grows
 * Writing data will be placed at the end of data always and reading data will pick the last data.
 * It is like simple file system that only keep a file only.
 *
 * Unlike file manipulation operation, there is caution you need to check if write operation fails
 * or need to check free size before you start to write data.
 * It is required to format flash sector when there is no more free space.
 */

#ifndef __SOFBLOCK_H
#define __SOFBLOCK_H

#define FLASH_FIRST_WRITE    (false)
#define FLASH_SECTOR_INDEX  (10)

#include "flash_device.h"

/** SOF(Storage On Flash) usage example
 *
 * Example:
 * @code
 * #include "SOFBlock.h"
 *
 * int main()
 * {
 * 	const uint8_t sector_index = 7;
 * 	SOFBlock::format(sector_index);	// Erase flash sector 7 and make structure for storage.
 *
 * 	SOFBlock writer;
 * 	SOFBlock reader;
 *
 * 	SOFBlock_open_write(writer, sector_index);
 * 	SOFBlock_write_data(writer, (uint8_t*)"First Data", 10);
 * 	SOFBlock_close_write(writer);
 *
 * 	SOFBlock_open_read(reader, sector_index);
 * 	printf("data %d bytes at %p :\r\n", SOFBlock_get_data_size(reader), SOFBlock_get_physical_base_addr(reader));
 * 	printf("%.*s\r\n", SOFBlock_get_data_size(reader), SOFBlock_get_physical_base_addr(reader));
 * 	// "First Data" printed
 * 	SOFBlock_read_close(reader);
 *
 *  SOF_Statics_t statics;
 *  if (!SOFBlock(sector_index, statics) || statics.free_size < 11) { // check available byte
 *     SOFBlock_format(writer, sector_index);
 *  }
 * 	SOFBlock_open_write(writer, sector_index);
 * 	// Overwrite previous data without erasing flash.
 * 	SOFBlock_write_data(writer, (uint8_t*)"Second Data", 11);
 * 	SOFBlock_close_write(writer);
 *
 * 	SOFBlock_open_read(reader, sector_index);
 * 	printf("data %d bytes at %p :\r\n", SOFBlock_get_data_size(reader), SOFBlock_get_physical_base_addr(reader));
 * 	printf("%.*s\r\n", SOFBlock_get_data_size(reader), SOFBlock_get_physical_base_addr(reader));
 * 	// "Second Data" printed
 * 	SOFBlock_read_close(reader);
 * }
 */

/**
 * Base class of SOF(Storage On Flash)
 */
typedef struct
{
    SOF_BlockHandle_t hblock_;
} SOFBlock;

/*** Returns whether instance of SOFBlock is currently associated to flash storage.  */
bool SOFBlock_is_open(SOFBlock* sofblock);
/*** Erase flash sector and put signature to setup file system struct */
bool SOFBlock_format(const uint8_t sector_index);
/*** Get statistics of storage */
bool SOFBlock_get_stat(const uint8_t sector_index, SOF_Statics_t *statics);

uint8_t * SOFBlock_get_physical_data_addr(SOFBlock* sofblock);
size_t SOFBlock_get_data_size(SOFBlock* sofblock);
/**
 * Interface for writing data to flash memory.
 */
/*** Open for writing mode */
SOF_Error_t SOFBlock_open_write(SOFBlock* sofblock, const uint8_t sector_index);
/*** close for writing mode */
void SOFBlock_close_write(SOFBlock* sofblock);
/*** Return max available for writing */
size_t SOFBlock_get_free_size(SOFBlock* sofblock);
/*** Write one byte of data.
 * Note: in case of storage full, it can't write data any more.
 * It is required to format sector and write it again.
 */
bool SOFBlock_write_byte_data(SOFBlock* sofblock, uint8_t c);
/*** Write n bytes of data */
size_t SOFBlock_write_data(SOFBlock* sofblock, const uint8_t *p, size_t p_size);

/**
* Interface for reading data from flash memory.
* It can read data directly by accessing physical flash address or
* calling function like traditional file API style.
*/
SOF_Error_t SOFBlock_open_read(SOFBlock* sofblock, const uint8_t sector_index);
void SOFBlock_close_read(SOFBlock* sofblock);

/*** Return flash physical address of data for direct access */
uint8_t *SOFBlock_get_physical_data_addr(SOFBlock* sofblock);

/*** Return data size */
size_t SOFBlock_get_data_size(SOFBlock* sofblock);

/*** Return one byte of data */
bool SOFBlock_read_byte_data(SOFBlock* sofblock, uint8_t *c);

/*** Return n bytes of data */
size_t SOFBlock_read_data(SOFBlock* sofblock, uint8_t *p, size_t p_size);

#endif

