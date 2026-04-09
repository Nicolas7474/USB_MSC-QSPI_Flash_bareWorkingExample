#include <BareM_Def.h>
#include "stm32f469xx.h"


// UART 3 mode
typedef enum {
	UART_Mode_IT,
	UART_Mode_DMA,
} Uart_Mode_t;


// UART 3 status
typedef enum {
    UART3_IDLE = 1,     // Added comma here
    UART3_BUSY = 2,
    UART3_ERROR = 4,
    UART3_RX_CMPLT = 8,
    UART3_TX_CMPLT = 16
} Uart3_Status_t;

typedef struct {
    USART_TypeDef 		*Instance;
    DMA_Stream_TypeDef  *DMA_Stream;
    uint32_t 			DMA_Channel;
    DMA_TypeDef 		*DMA;
} UART_DMA_Handle;


// Not passing the I2C_Handle_t Struct as parameter in the I2C functions, not to expose directly it to the Main (and other) files
typedef uint8_t uartPort_t; // typdef to pass public pointer variables as parameter to the I2C functions
extern uartPort_t huart1; // public pointers with the same name as the original HAL functions
extern uartPort_t huart2;
extern uartPort_t huart3;

extern BareM_StatusTypeDef BareM_Status_Uart3; // used in main()
extern Uart3_Status_t Uart3_Status; // used in main()

extern uint8_t bufferRx[256]; // used with main()

BareM_StatusTypeDef Uart3_Init(uartPort_t *huart, int baudrate);

void UART3_SendChar (uint8_t c);

void UART3_SendString (char *string);

uint8_t UART3_GetChar (void);

BareM_StatusTypeDef UART_Send_IT(uint8_t *buf, uint16_t len); // uint8_t *buf, uint16_t len


void UART3_DMA_SendString(char* str);
void UART3_ProcessIncoming(void);
void Parse_String(uint8_t* data, uint16_t length);

volatile extern int tx_done;

/// *** DMA *********

#define BUF_SIZE 256
// To be called in the ISR to process incoming strings
void Process_UART_Data(uint8_t echo_back);
void Execute_Command(char* cmd);
BareM_StatusTypeDef UART_Receive_DMA(uartPort_t *huart, uint8_t *pData, uint16_t Size);
BareM_StatusTypeDef UART_Transmit_DMA(uartPort_t *huart, const uint8_t *pData, uint16_t Size);

int Irq_dma (UART_DMA_Handle *huartTx);

