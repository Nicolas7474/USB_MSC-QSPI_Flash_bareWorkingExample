/*-----------------------------------------------------------------------*/
/* Low level disk I/O module SKELETON for FatFs     (C)ChaN, 2025        */
/*-----------------------------------------------------------------------*/
/* If a working storage control module is available, it should be        */
/* attached to the FatFs via a glue function rather than modifying it.   */
/* This is an example of glue functions to attach various existing      */
/* storage control modules to the FatFs module with a defined API.       */
/*-----------------------------------------------------------------------*/

#include <string.h>
#include "ff.h"			/* Basic definitions of FatFs */
#include "diskio.h"		/* Declarations FatFs MAI */
#include "qspi.h"

/* Example: Declarations of the platform and disk functions in the project */
//#include "platform.h"
//#include "storage.h"

/* Example: Mapping of physical drive number for each drive */
#define DEV_FLASH	0	/* Map FTL to physical drive 0 */
#define DEV_MMC		1	/* Map MMC/SD card to physical drive 1 */
#define DEV_USB		2	/* Map USB MSD to physical drive 2 */

/* FatFs usually aligns its internal buffers, but if you define your own buffer in main.c to read a file into,
always use __attribute__((aligned(4))) */
uint8_t sector_buffer[4096] __attribute__((aligned(4)));

/*-----------------------------------------------------------------------*/
/* Get Drive Status                                                      */
/*-----------------------------------------------------------------------*/

DSTATUS disk_status (BYTE pdrv)	{

	/* Physical drive ID number = 0 */
	if (pdrv != 0) return STA_NOINIT;
	//	DSTATUS stat; int result; result = USB_disk_status();
	//	return stat; // TODO test Flash status
	return RES_OK;
}

/*-----------------------------------------------------------------------*/
/* Initialize a Drive                                                    */
/*-----------------------------------------------------------------------*/

DSTATUS disk_initialize (BYTE pdrv) {

	if (flag_usb_connected) return FR_LOCKED; // The "Low-Level Gate": check if the PC is currently the boss
	/* Physical drive ID number = 0 */
	// DSTATUS stat; int result;
	//result = RAM_disk_initialize();	return stat; // TODO
	return RES_OK;
}



/*-----------------------------------------------------------------------*/
/* Read Sector(s)                                                        */
/*-----------------------------------------------------------------------*/

DRESULT disk_read (BYTE pdrv, BYTE *buff, LBA_t sector, UINT count) {
	// Drive nb / Buffer to store read data / Start sector in LBA / Nb of sectors to read

	if (flag_usb_connected) return FR_LOCKED; // The "Low-Level Gate": check if the PC is currently the boss
	if (pdrv != 0) return RES_PARERR; 	// Safety check (support only one drive (0))

	// Calculate the physical Memory-Mapped address
	uint32_t flash_addr = 0x90000000 + (sector * FF_MAX_SS); 	// a LBA sector is FF_MAX_SS bytes (4096)

	memcpy(buff, (void*)flash_addr, count * FF_MAX_SS); 	// Copy the data, FatFs provides 'buff'

	return RES_OK; 	// Return Success
}



/*-----------------------------------------------------------------------*/
/* Write Sector(s)                                                       */
/*-----------------------------------------------------------------------*/

#if FF_FS_READONLY == 0

DRESULT disk_write(BYTE pdrv, const BYTE *buff, LBA_t sector, UINT count) {

	if (pdrv != 0) return RES_PARERR;
	if (flag_usb_connected) return FR_LOCKED; // The "Low-Level Gate": check if the PC is currently the boss

    const BYTE *data_ptr = buff;
    uint32_t current_lba = sector;
    uint32_t remaining_count = count;

    while (remaining_count > 0) {
        // Use FF_MAX_SS (4096) to match your disk_read math!
        uint32_t subsector_base_addr = current_lba * FF_MAX_SS;

        // 2. Physical Erase (Exit Memory-Mapped Mode)
        MT25Q_SubsectorErase_4KB(subsector_base_addr);

        // 3. Physical Write
        // Every time FatFs wants to write a "sector," it perfectly matches one physical 4KB subsector of the MT25Q128
        // Since count=1 is 4096 bytes, we can write the buffer directly
        MT25Q_SubsectorWrite_4KB(subsector_base_addr, (uint8_t*)data_ptr);

        // 4. Update
        remaining_count--;
        current_lba++;
        data_ptr += FF_MAX_SS;
    }

    // 5. Critical: Restore Memory-Mapped mode for disk_read and other accesses
    QSPI_Enable_MemoryMapped();

    return RES_OK;
}


#endif


/*-----------------------------------------------------------------------*/
/* Miscellaneous Functions                                               */
/*-----------------------------------------------------------------------*/

DRESULT disk_ioctl (BYTE pdrv, BYTE cmd, void *buff) {
	//  Drive nb, Control code, Buffer to send/receive control data
	// This function control device specific features and miscellaneous functions other than generic read/write
	// This is how FatFs learns about "32KB Cluster" logic and "512B Sector" alignment
	//DRESULT res; int result;

	if (pdrv != 0) return RES_PARERR;
	if (flag_usb_connected) return FR_LOCKED; // The "Low-Level Gate": check if the PC is currently the boss

	switch (cmd) {
	case CTRL_SYNC: 	// Makes sure that the device has finished pending write process.
		flag_fatfs_busy = 0; 	// We are done
		return RES_OK; // Nothing to flush in memory-mapped mode
	case GET_SECTOR_COUNT:
		*(LBA_t*)buff = 4096; // 16MB / 4096 bytes
		flag_fatfs_busy = 0; 	// We are done
		return RES_OK;
	case GET_SECTOR_SIZE:
		*(WORD*)buff = 4096;
		flag_fatfs_busy = 0; 	// We are done
		return RES_OK;
	case GET_BLOCK_SIZE:
		*(DWORD*)buff = 8; // Erase unit is 4KB (8 sectors)
		flag_fatfs_busy = 0; 	// We are done
		return RES_OK;
	}
	return RES_PARERR;
}



DWORD get_fattime (void) {
    /* Returns a dummy time: 1st Jan 2026 */
    return ((DWORD)(2026 - 1980) << 25) /* Year 2026 */
         | ((DWORD)1 << 21)             /* Month 1 */
         | ((DWORD)1 << 16)             /* Mday 1 */
         | ((DWORD)0 << 11)             /* Hour 0 */
         | ((DWORD)0 << 5)              /* Min 0 */
         | ((DWORD)0 >> 1);             /* Sec 0 */
}
