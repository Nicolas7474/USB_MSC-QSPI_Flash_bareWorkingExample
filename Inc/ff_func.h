/*-----------------------------------------------------------------------*/
/* High level functions for FatFs     Prata Nicolas, 2026       		 */
/*-----------------------------------------------------------------------*/

#include <stdio.h>
#include <stdint.h>

/* Prototypes definitions */

FRESULT list_dir (const char *path);
FRESULT disp_lines (const char *yourFile);
FRESULT append_to_file(const char* path, const char* text);



