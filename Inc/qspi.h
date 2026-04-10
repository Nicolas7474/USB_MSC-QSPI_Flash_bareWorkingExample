#include "stm32f469xx.h"


/*	For a 128Mbit (16MB) Flash with 4096-byte sectors	*/
#define FLASH_TOTAL_SIZE      (16 * 1024 * 1024) // 16MB
#define MSC_BLOCK_SIZE        4096
#define MSC_TOTAL_BLOCKS      (FLASH_TOTAL_SIZE / MSC_BLOCK_SIZE) // 4096


void QSPI_Hardware_Init(void);

void QSPI_Flash_Reset(void);
void QSPI_Enable_MemoryMapped(void);
void QSPI_WriteEnable(void);
void QSPI_EnableQuadMode(void);
uint8_t QSPI_GetStatus(uint8_t instruction);
void QSPI_WaitUntilReady(void);
void QSPI_Prepare_Indirect(void);
uint32_t MT25Q_readID(void);

void MT25Q_SubsectorRead(uint32_t address, uint8_t *rData);
void MT25Q_SubsectorErase_4KB(uint32_t address);
void MT25Q_SubsectorErase_32KB(uint32_t address);
void MT25Q_SubsectorWrite_4KB(uint32_t address, uint8_t *data);
void MT25Q_BulkErase(void);
void MT25Q_SendCommand(uint8_t instruction);
uint8_t MT25Q_Check_Hardware_Success(void);
