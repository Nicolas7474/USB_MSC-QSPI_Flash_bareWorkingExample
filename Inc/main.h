
#include <stdint.h>
#include "ff.h" // FatFs core

uint32_t USB_CDC_receive_data(uint16_t length);

extern FATFS fs;   // Only declare as extern the Filesystem object (The "Drive" instance)

