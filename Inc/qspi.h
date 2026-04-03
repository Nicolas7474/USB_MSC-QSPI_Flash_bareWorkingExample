#include "stm32f469xx.h"

void QSPI_Hardware_Init(void);
void QSPI_DMA_Global_Init(void);
void QSPI_Flash_Reset(void);
void QSPI_Enable_MemoryMapped(void);
void QSPI_WriteEnable(void);
void QSPI_EnableQuadMode(void);
uint8_t QSPI_GetStatus(void);
void QSPI_WaitUntilReady(void);
uint32_t readID(void);
void MT25Q_SubsectorRead_DMA(uint32_t address, uint32_t *rData);
void MT25Q_SubsectorErase(uint32_t address);
void MT25Q_SubsectorWrite(uint32_t address, uint32_t *data);
