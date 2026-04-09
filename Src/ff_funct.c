/*-------------------------------------------------------------------*/
/* Examples of High level functions for FatFs     					 */
/* Prata Nicolas, 2026       		 								 */
/*-------------------------------------------------------------------*/

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "stm32f469xx.h"
#include "main.h"
#include "ff.h" 		// FatFs core
#include "diskio.h" 	// FatFs driver wrappers
#include "uart3.h"

char read_buffer[128]; // Store the text from the file
char line[2]; /* Line buffer */


// Mount the drive - This doesn't "touch" the flash much; it just tells FatFs to initialize the fs structure and prepare for communication.
/*res = f_mount(&fs, "0:", 1); //  "": Defaut Drive (number 0) ; 1: Forced mount (checks for FAT structure immediately)
if (res != FR_OK) {
	// If res is FR_NO_FILESYSTEM
}*/



/*----------------------------------------------------*/
/*****		 List the contents of a directory	 ******/
/*----------------------------------------------------*/
FRESULT list_dir (const char *path)
{
	/* Perform High-Level Logic for USB-FATfs switch */
	if (flag_usb_connected) return FR_LOCKED; // Check if the PC is currently the boss
	flag_fatfs_busy = 1;   // Inform maintenance_switch() that we are busy

	FRESULT res;
	DIR dir;
	FILINFO fno; // Pointer to the file information structure to store the information about read item
	int nfile, ndir;
	char buf[272]; // LFN: in ExFAT maximum file name length is 255 (+ hard-coded words = 272)

	res = f_opendir(&dir, path);  // Open the directory
	if (res == FR_OK) {

		nfile = ndir = 0; 	// initialize variables
		snprintf(buf, 5, "\r\n\r\n");
		UART_Transmit_DMA(&huart3, (uint8_t*)buf, 4);
		for (;;) {
			res = f_readdir(&dir, &fno);  // Read a directory item - loop increment: FatFs moves the internal pointer in &dir to the next entry
			if (fno.fname[0] == 0) break;  // Error or end of dir (index [0] checks the existence of the 1st file's character)
			if (fno.fattrib & AM_DIR) {     // It is a directory
				snprintf(buf, sizeof(buf), "\n\r   <DIR>   %s\n\r", fno.fname);
				UART_Transmit_DMA(&huart3, (uint8_t*)buf, strlen(buf));
				ndir++;
			} else { 	// It is a file
				snprintf(buf, sizeof(buf), "%u %s\n\r", (uint16_t)fno.fsize, fno.fname);
				UART_Transmit_DMA(&huart3, (uint8_t*)buf, strlen(buf));
				nfile++;
			}
		}
		f_closedir(&dir);
		snprintf(buf, sizeof(buf), "%d dirs, %d files.\n\r", ndir, nfile);
		UART_Transmit_DMA(&huart3, (uint8_t*)buf, strlen(buf));

	} else {
		snprintf(buf, sizeof(buf), "\n\rFailed to open \"%s\". (%u)\n\r", path, res);
		UART_Transmit_DMA(&huart3, (uint8_t*)buf, strlen(buf));
	}
	flag_fatfs_busy = 0; // Only now are we truly "Safe"

	return res;
}


/*----------------------------------------------------*/
/* Open a text file, read every line and display it   */
/*----------------------------------------------------*/
FRESULT disp_lines (const char *yourFile)
{
	/* Perform High-Level Logic for USB-FATfs switch */
	if (flag_usb_connected) return FR_LOCKED; // Check if the PC is currently the boss
	flag_fatfs_busy = 1;   // Inform maintenance_switch() that we are busy

	FIL fil;
	FRESULT res;
	res = f_open(&fil, yourFile, FA_READ); // Open a text file
	if (res) return (int)res;

	while (f_gets(line, sizeof line, &fil)) {		//  Read every line and display it
		UART_Transmit_DMA(&huart3, (uint8_t*)line, strlen(line));
	}
	f_close (&fil); //  Close the file
	flag_fatfs_busy = 0; // Only now are we truly "Safe"

	return FR_OK;
}

/*-----------------------------------------------------*/
/* Open a file, write a string at the end and close it */
/*-----------------------------------------------------*/
FRESULT append_to_file(const char* path, const char* data) {

	/* Perform High-Level Logic for USB-FATfs switch */
	if (flag_usb_connected) return FR_LOCKED; // Check if the PC is currently the boss
	flag_fatfs_busy = 1;   // Inform maintenance_switch() that we are busy

    FIL fil;
    FRESULT res;
    UINT bw; // Pointer to the variable that returns the number of bytes written

    // Open the file with Write access
    // FA_OPEN_APPEND is available in newer FatFs versions
    res = f_open(&fil, path, FA_WRITE | FA_OPEN_APPEND);
    if (res != FR_OK) return res;

    // Write the string - with FA_OPEN_APPEND, no f_lseek() needed!
    res = f_write(&fil, data, strlen(data), &bw);

    f_close(&fil);  // Close the file to flush the data to QSPI Flash
    flag_fatfs_busy = 0; // Only now are we truly "Safe"

    return res;
}



