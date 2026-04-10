#include "stm32f469xx.h"


/************ DEFINE *****************************/

#define FLASH_TOTAL_SIZE      (16 * 1024 * 1024) //	For a 128Mbit (16MB) Flash with 4096-byte sectors
#define MSC_BLOCK_SIZE        4096
#define MSC_TOTAL_BLOCKS      (FLASH_TOTAL_SIZE / MSC_BLOCK_SIZE) // 4096
#define PAGE_SIZE			  256

/* MT25QL128 Command Definitions */
#define READ_ID 					0x9E
#define DEVICE_ID 					0x18BA20
#define READ 						0x03
#define FAST_READ 					0x0B
#define WRITE_ENABLE 				0x6B
#define READ_STATUS_REGISTER 		0x05
#define READ_FLAG_STATUS_REGISTER 	0x70
#define CLEAR_FLAG_STATUS_REGISTER	0x50
#define QUAD_INPUT_FAST_PROGRAM		0x32
#define SUBSECTOR_ERASE_32KB		0x52
#define SUBSECTOR_ERASE_4KB			0x20
#define SECTOR_ERASE				0xD8
#define BULK_ERASE 					0xC7

/**********************************************/


void QSPI_Hardware_Init(void);

void QSPI_Flash_Reset(void);
void QSPI_Enable_MemoryMapped(void);
void QSPI_WriteEnable(void);
void QSPI_EnableQuadMode(void);
uint8_t QSPI_GetStatus(uint8_t instruction);
void QSPI_WaitUntilReady(void);
void QSPI_Prepare_Indirect(void);
uint32_t MT25Q_readID(void);

void MT25Q_SubsectorErase_4KB(uint32_t address);
void MT25Q_SubsectorErase_32KB(uint32_t address);
void MT25Q_SubsectorWrite_4KB(uint32_t address, uint8_t *data);
void MT25Q_BulkErase(void);
void MT25Q_SendCommand(uint8_t instruction);
uint8_t MT25Q_Check_Hardware_Success(void);
