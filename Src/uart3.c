// ******* UART3 - Bare-Metal Driver - Interrupts and DMA based **** //

#include <stdint.h>
#include <string.h>
#include <math.h>
#include "uart3.h"
#include "stm32f469xx.h"
#include "timers.h"

// #define USE_COBS

volatile int flag2 = 0;
int count3 = 0;
volatile uint16_t tx_len;
volatile uint16_t tx_index;
uint8_t tx_buf[256];

uint8_t bufferRx[256] = {0};
uint8_t encodeBuffer[2] = {0};

#if defined(USE_COBS)
#include "cobs.h"
cobs_encode_result encode_result;
#endif

Uart3_Status_t Uart3_Status = UART3_IDLE;
BareM_StatusTypeDef BareM_Status_Uart3; // variable for return value of functions

Uart_Mode_t Uart_Mode;

uartPort_t huart1;  // not passing the UART_DMA_Handle Struct as parameter in the uart functions, not to expose directly it to the Main (and other) files
uartPort_t huart2; // public pointers with the same name as the original HAL functions
uartPort_t huart3; // public pointers with the same name as the original HAL functions


/************** DMA mode ***************/

uint8_t rx_buffer[BUF_SIZE];
volatile uint16_t rx_read_ptr = 0;

/******************************************/

// Initialize a handle here (extern - redeclaration) containing the parameters of DMA
UART_DMA_Handle huart3_rx = {USART3, DMA1_Stream1, 4, DMA1}; //  Are your streams & channels valid for the peripheral chosen ? Check the DMA request mapping table
UART_DMA_Handle huart3_tx = {USART3, DMA1_Stream3, 4, DMA1}; // DMA1 generally handles slower peripherals on the APB1 bus (e.g., I2C, USART2/3, TIM2-7)


BareM_StatusTypeDef Uart3_Init(uartPort_t *huart, int baudrate) {

	Uart_Mode_t mode = UART_Mode_DMA;
	Uart_Mode = mode; // variable in ISR

	// pointer declarations to:  typedef struct { USART_TypeDef *Instance; DMA_Stream_TypeDef *DMA_Stream; uint32_t DMA_Channel; } UART_DMA_Handle;
	UART_DMA_Handle *huartRx;
	UART_DMA_Handle *huartTx;

	if(huart == &huart3) {
		huartRx = &huart3_rx; // points to UART_DMA_Handle huart3_tx
		huartTx = &huart3_tx; // points to UART_DMA_Handle huart3_rx
	}
	// to fill if() with other UART ports
	else { return BareM_Status_Uart3 = Bare_TIMEOUT; }

	int USART_IRQn;
	if(huartRx->Instance == USART1 && huartTx->Instance == USART1) { USART_IRQn = 37; }
	else if(huartRx->Instance == USART2 && huartTx->Instance == USART2) { USART_IRQn = 38; }
	else if(huartRx->Instance == USART3 && huartTx->Instance == USART3) { USART_IRQn = 39; }
	else { return BareM_Status_Uart3 = Bare_TIMEOUT; }

	//Uart_Mode = mode; // variable in ISR

	RCC->APB1ENR |= (1<<18) | (1<<1);  // 1. Enable the USART3 CLOCK & Enable the GPIO-B clock
	GPIOB->MODER |= (2<<20) | (2<<22); // 2. Configure the UART PINs for Alternate Functions // Bits (21:20) & (23:22)= 1:0 --> Alternate Function for Pin PB10 & Pin 11
	GPIOB->OSPEEDR |= GPIO_OSPEEDR_OSPEED10 | GPIO_OSPEEDR_OSPEED11; // (3<<20) | (3<<22); Bits (21:20)= 1:1 and Bits (23:22)= 1:1 --> High Speed for PIN PB10 and PB11
	GPIOB->AFR[1] |= (7<<8) | (7<<12);  // Bytes  (15:14:13:12) & (11:10:9:8) = 0:1:1:1  --> AF7 Alternate function for USART3 at Pin PB10 & PB11

	// 3. Enable the USART by writing the UE bit in USART_CR1 register to 1.
	// huartRx and huartTx share the same instance ( = USART3) so no need to write the bits for huartTx
	huartRx->Instance->CR1 |= USART_CR1_UE; // (1<<13); UE = 1... Enable USART

	// 4. Program the M bit in USART_CR1 to define the word length.
	huartRx->Instance->CR1 &= ~(1<<12);  //USART_CR1_M;  M =0; 8 bit word length

	// 5. Select the desired baud rate using the USART_BRR register assuming PCLK1 (= APB1 Peripheral Clock) @ 45MHz
	float raw = 45000000 / (float)(8 * 2 * baudrate);  	// USARTDIV = fPCLK / (8x (2 - OVER8) x BaudRate = 45'000'000 / (8 x 2 x 115200) = 24,414
	int mantisse = (int)raw; 		// DIV_Mantissa = mantissa (0d24,414) = 0d24 ; DIV_Fraction: 16*0,414 = 6,62 -> nearest integer = 0d7
	int fraction =  rintf((float)(raw - mantisse) * 16); // rintf rounds the float with <math.h>
	huartRx->Instance->BRR = (fraction<<0) | (mantisse<<4);

	// ** Configure DMA1 Stream 1, Channel 4 (RX) **
	// Data is loaded from the USART_DR register to a SRAM area configured using the DMA peripheral (refer to the DMA specs) whenever a data byte is received
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN; // Enable DMA1 Clock
	huartRx->Instance->CR3 = USART_CR3_DMAT | USART_CR3_DMAR; // DMA mode can be enabled for transmission by setting DMAT bit in the USART_CR3 register

	// When the bit DMA_SxCR_EN is read as 0, the sw is allowed to program the configuration and FIFO bits registers
	huartRx->DMA_Stream->CR &= ~DMA_SxCR_EN; 		// (it is forbidden to write those registers when the EN bit is read as 1)
	while(huartRx->DMA_Stream->CR & DMA_SxCR_EN);		// Stream enable/flag stream ready when read low

	// Channel 4 selected, PL priority Medium, Memory address pointer incremented after each data transfer, Circular mode,
	// EN: Stream enable/flag stream ready when read low. This bit is set and cleared by sw.  0: Stream disabled - 1: Stream enabled
	// (this bit is cleared by hw: – on a DMA end of transfer (stream ready to be configured), by default DMA_SxCR_DIR = 0 (00: Peripheral-to-memory)
	huartRx->DMA_Stream->CR = (huartRx->DMA_Channel << DMA_SxCR_CHSEL_Pos) | DMA_SxCR_PL_0 | DMA_SxCR_MINC | DMA_SxCR_CIRC; // | DMA_SxCR_EN; // DMA_SxCR_EN -> Start DMA RX

	/* When the number of data transfers programmed in the DMA Controller is reached, the DMA controller generates an interrupt on the DMA channel interrupt vector.
		  The DMAR bit should be cleared by software in the USART_CR3 register during the interrupt subroutine.*/

	// ** Configure DMA1 Stream 3, Channel 4 (TX) **
	//DMA1_Stream3->M0AR = (uint32_t)tx_buffer;  // The data will be loaded into the USART_DR register from this memory area after each TXE event
	// DIR[1:0]: Data transfer direction (01: Memory-to-peripheral), TCIE: Transfer complete interrupt enable
	huartTx->DMA_Stream->CR = (huartTx->DMA_Channel << DMA_SxCR_CHSEL_Pos) | DMA_SxCR_PL_0 | DMA_SxCR_MINC | DMA_SxCR_DIR_0 | DMA_SxCR_TCIE ; // | DMA_SxCR_TEIE

	// NVIC Setup for DMA Interrupt
	NVIC_EnableIRQ(Irq_dma(huartTx));

	// Enable Uart interrupts and the Transmitter/Receiver by Setting the TE and RE bits in USART_CR1 Register
	huartRx->Instance->CR1 |= USART_CR1_RE | USART_CR1_TE | USART_CR1_RXNEIE | USART_CR1_IDLEIE; //Enable the Receiver and Transmitter, Rx interrupts, Idle line
	huartRx->Instance->CR1 &= ~USART_CR1_TXEIE; // disable Tx interrupts

	NVIC_SetPriority(USART_IRQn, 4);
	NVIC_EnableIRQ(USART_IRQn);

	return BareM_Status_Uart3 = Bare_OK;
}


BareM_StatusTypeDef UART_Transmit_DMA(uartPort_t *huart, const uint8_t *pData, uint16_t Size) 	// function mimics HAL_UART_Transmit_DMA()
{
	while(!(Uart3_Status & UART3_IDLE)); // as long as status is not Idle (1)
	UART_DMA_Handle *huartTx; // points to UART_DMA_Handle huart3_rx
	if(huart == &huart3) { huartTx = &huart3_tx; }
	// fill if() with other UART ports
	else { return BareM_Status_Uart3 = Bare_ERROR; }

	// Wait if previous TX is still busy - Safely Disable Stream after 5ms
	uint32_t timeout_counter = GetSysTick();
	while(huartTx->DMA_Stream->CR & DMA_SxCR_EN) {
		if (GetSysTick() - timeout_counter > 5)  return BareM_Status_Uart3 = Bare_TIMEOUT;
	}

	Uart3_Status = UART3_BUSY;
	// PAR: Base address of the peripheral data register from/to which the data is read/written. These bits are write-protected
	huartTx->DMA_Stream->PAR  = (uint32_t)&huartTx->Instance->DR; // DR = USART Data register from USART_TypeDef huartTx
	huartTx->DMA_Stream->M0AR = (uint32_t)pData;
	huartTx->DMA_Stream->NDTR = Size;

	huartTx->DMA->LIFCR |= DMA_LIFCR_CTCIF3 | DMA_LIFCR_CHTIF3 | DMA_LIFCR_CFEIF3 | DMA_LIFCR_CDMEIF3 | DMA_LIFCR_CTEIF3; // Clear interrupt flags / huartTx->DMA = DMA1
	huartTx->DMA_Stream->CR |= DMA_SxCR_EN; //  and start

	while(!(Uart3_Status & UART3_IDLE)); // as long as status is not Idle (1)
	return BareM_Status_Uart3 = Bare_OK;
}

BareM_StatusTypeDef UART_Receive_DMA(uartPort_t *huart, uint8_t *pData, uint16_t Size) {

	UART_DMA_Handle *huartRx; // points to UART_DMA_Handle huart3_rx
	if(huart == &huart3) { huartRx = &huart3_rx; }
	// fill if() with other UART ports
	else { return BareM_Status_Uart3 = Bare_ERROR; }

	// Wait if previous TX is still busy - Safely Disable Stream after 5ms
	uint32_t timeout_counter = GetSysTick();
	huartRx->DMA_Stream->CR &= ~DMA_SxCR_EN;
	while(huartRx->DMA_Stream->CR & DMA_SxCR_EN) { if (GetSysTick() - timeout_counter > 5) return BareM_Status_Uart3 = Bare_TIMEOUT; }

	// Clear Flags for Stream 2 (LIFCR)
	huartRx->DMA->LIFCR |= DMA_LIFCR_CTCIF2 | DMA_LIFCR_CHTIF2 | DMA_LIFCR_CTEIF2;

	huartRx->DMA_Stream->PAR  = (uint32_t)&(huartRx->Instance->DR);
	huartRx->DMA_Stream->M0AR = (uint32_t)pData;
	huartRx->DMA_Stream->NDTR = Size;

	// Config: P-to-M, Circular, Mem Increment, HTIE, TCIE
	huartRx->DMA_Stream->CR &= ~(DMA_SxCR_CHSEL | DMA_SxCR_DIR);
	huartRx->DMA_Stream->CR |= (huartRx->DMA_Channel << DMA_SxCR_CHSEL_Pos) | DMA_SxCR_MINC | DMA_SxCR_CIRC | DMA_SxCR_TCIE | DMA_SxCR_HTIE;
	huartRx->DMA_Stream->CR |= DMA_SxCR_EN;

    return BareM_Status_Uart3 = Bare_OK;
}

/*	Main Processing Logic
This logic calculates how many bytes were written by the DMA since the last check. It handles the "wrap-around" of the circular buffer automatically.
Circular Overflow: By using DMA_SxCR_CIRC, the DMA will never stop. If you don't process data fast enough, it will simply overwrite the oldest data.
The logic len = (BUF_SIZE - rx_read_ptr) + curr_pos correctly calculates the distance even when the hardware pointer has looped back to the start of the array.
*/
void Process_UART_Data(uint8_t echo_back) {

    // NDTR counts DOWN from BUF_SIZE to 0
    uint16_t curr_pos = BUF_SIZE - (uint16_t)DMA1_Stream1->NDTR;
    uint16_t len = 0;

    if (curr_pos != rx_read_ptr) {   GPIOG->ODR^=GPIO_ODR_OD6;
        if (curr_pos > rx_read_ptr) {
            len = curr_pos - rx_read_ptr;
        } else {
            len = (BUF_SIZE - rx_read_ptr) + curr_pos;
        }

        if (echo_back) {
            // Because it's a ring buffer, if data wraps, we might need two sends
            // or we copy it to a linear buffer first. For simplicity:
            static uint8_t temp_buf[BUF_SIZE];

            for(uint16_t i=0; i<len; i++) {
                temp_buf[i] = bufferRx[(rx_read_ptr + i) % BUF_SIZE];
            }
            // echo back the string received
            UART_Transmit_DMA(&huart3, (uint8_t*)temp_buf, len);

            temp_buf[len] = '\0'; // Crucial for strstr()
			// Execute the logic
			Execute_Command((char *)temp_buf);
        }

        rx_read_ptr = curr_pos; // Update read pointer to current DMA position
    }
}

void Execute_Command(char* cmd) {
    // Check for "LED_ON"
    if (strstr(cmd, " LED_ON")) {
    	GPIOG->ODR^=GPIO_ODR_OD6;
    	UART_Transmit_DMA(&huart3, (uint8_t*)" OK: LED is now ON\r\n", 19);
    }
    // Check for "LED_OFF"
    else if (strstr(cmd, "LED_OFF")) {
    	GPIOG->ODR^=GPIO_ODR_OD6;
    	UART_Transmit_DMA(&huart3, (uint8_t*)" OK: LED is now OFF\r\n", 20);
    }
    // Fallback for unknown commands
    else {
     //   UART_SendDMA(&huart3, " ERR: Unknown Command\r\n", 22);
    }
}

/*In transmission mode, once the DMA has written all the data to be transmitted (the TCIF flag
is set in the DMA_ISR register), the TC flag can be monitored to make sure that the USART
communication is complete. This is required to avoid corrupting the last transmission before
disabling the USART or entering the Stop mode. The software must wait until TC=1. The TC
flag remains cleared during all data transfers and it is set by hardware at the last frame end
of transmission.*/

void DMA1_Stream1_IRQHandler(void) {
    if (DMA1->LISR & DMA_LISR_TCIF1) {
    	DMA1->LIFCR = DMA_LIFCR_CTCIF1; // Clear Transfer Complete flag
    	Uart3_Status = UART3_RX_CMPLT | UART3_IDLE; // add 16 + 1 to cleared status (=)
    }
}

void DMA1_Stream3_IRQHandler(void) {
	if (DMA1->LISR & DMA_LISR_TCIF3) {
		DMA1->LIFCR = DMA_LIFCR_CTCIF3; // Clear TX Transfer Complete
		Uart3_Status = UART3_TX_CMPLT | UART3_IDLE; // add 16 + 1 to cleared status (=)
	}
}


void USART3_IRQHandler() {

	if(Uart_Mode == UART_Mode_IT)
	{
		// receive UART bytes
		if (USART3->SR & USART_SR_RXNE) { // 'Receive register not empty' interrupt; RXNE is cleared by a read to the USART_DR register
			Uart3_Status = UART3_BUSY;
			bufferRx[count3] = USART3->DR;    // Copy new data into the buffer
			count3++;
		}
		// detect idle line, indicates the last character is received
		else if(USART3->SR & USART_SR_IDLE) {
			volatile uint32_t dummy = USART3->SR;
			dummy = USART3->DR; (void)dummy; 	// Clear IDLE flag (Read SR then DR)
			bufferRx[count3] = USART3->DR;
			count3 = 0;
			Uart3_Status = UART3_RX_CMPLT | UART3_IDLE; // add 8 + 1 to cleared status (=)
		}
		else if((USART3->CR1 & USART_CR1_TXEIE) && (USART3->CR1 & USART_SR_TXE)) {
			if (tx_index < tx_len) {
				Uart3_Status = UART3_BUSY;
				USART3->DR = tx_buf[tx_index++]; // data;
			}
			if (tx_index == tx_len)	{
				USART3->CR1 |= USART_CR1_TCIE; // TCIE: Transmission complete interrupt enable
				USART3->SR |= USART_SR_TC;
				USART3->CR1 &= ~USART_CR1_TXEIE;
				memset(tx_buf, 0, strlen((char *)tx_buf)); // clear bufferTx
			}
		}
		// This bit is set by hw if the transmit of a frame is complete and if TXE is set.
		if (USART3->SR & USART_SR_TC && USART3->CR1 & USART_CR1_TCIE) {
			USART3->SR &= ~USART_SR_TC; // clear USART_SR_TC;
			USART3->CR1 &= ~USART_CR1_TCIE;
			Uart3_Status = UART3_TX_CMPLT | UART3_IDLE; // add 16 + 1 to cleared status (=)
		}
	}

	if(Uart_Mode == UART_Mode_DMA)
	{
		//The IDLE line interrupt fires when the RX line has been high (silent) for one frame duration, signaling the end of a transmission.
		if (USART3->SR & USART_SR_IDLE) {
			volatile uint32_t dummy_sr = USART3->SR;
			volatile uint32_t dummy_dr = USART3->DR;
			(void)dummy_sr; (void)dummy_dr;
			UART_Receive_DMA(&huart3, bufferRx, 128);
		}
	}

	// Check for Overrun Error (ORE) - happens if software/DMA is too slow
	if (USART3->SR & USART_SR_ORE) {
		volatile uint32_t dummy = USART3->SR;
		dummy = USART3->DR; (void)dummy; // Clear ORE flag by reading SR and DR
		}
}



BareM_StatusTypeDef UART_Send_IT(uint8_t *buf, uint16_t len) {
	// can use strlen(buf) for len

	while(!(Uart3_Status & UART3_IDLE)); // as long as status is not Idle (1) 	Uart3_Status = UART3_BUSY;
	tx_len = len;
	tx_index = 0;
	Uart_Mode = UART_Mode_IT;
#if defined(USE_COBS)
	encode_result = cobs_encode(tx_buf, sizeof(tx_buf), buf, len); 	//COBS-encode buf to tx_buf
	if(encode_result.status != COBS_ENCODE_OK) return;
#else
	memcpy(tx_buf, buf, len); // COBS encoding is disabled
#endif

	Uart3_Status = UART3_BUSY;
	USART3->SR &= ~USART_SR_TC; // TC flag is cleared before enabling interrupts. This step is important, as TC might still be set from a previous transmission.
	USART3->CR1 |= USART_CR1_TXEIE; // TXE interrupt enable: An USART interrupt is generated whenever TXE=1 in the USART_SR register. It is cleared by a write to the USART_DR register.
	/* and TXE=1 when idle ! (do not enable TXE interrupt until you have smthg to send. Disable it BEFORE writing the last char to be sent)
		TXE: Transmit data register empty: This bit is set by hardware when the content of the TDR register has been transferred into the shift register.
		0: Data is not transferred to the shift register ;  1: Data is transferred to the shift register  */
	return BareM_Status_Uart3 = Bare_OK;
}


// look for corresponding dma irq according to the Stream number
int Irq_dma (UART_DMA_Handle *huartTx) {
	int dmaStream;
	if(huartTx->DMA_Stream == DMA1_Stream0 ) { return DMA1_Stream0_IRQn; }
	else if(huartTx->DMA_Stream == DMA1_Stream1 ) { dmaStream = DMA1_Stream1_IRQn; }
	else if(huartTx->DMA_Stream == DMA1_Stream2 ) { dmaStream = DMA1_Stream2_IRQn; }
	else if(huartTx->DMA_Stream == DMA1_Stream3 ) { dmaStream = DMA1_Stream3_IRQn; }
	else if(huartTx->DMA_Stream == DMA1_Stream4 ) { dmaStream = DMA1_Stream4_IRQn; }
	else if(huartTx->DMA_Stream == DMA1_Stream5 ) { dmaStream = DMA1_Stream5_IRQn; }
	else if(huartTx->DMA_Stream == DMA1_Stream6 ) { dmaStream = DMA1_Stream6_IRQn; }
	else if(huartTx->DMA_Stream == DMA1_Stream7 ) { dmaStream = DMA1_Stream7_IRQn; }
	else if(huartTx->DMA_Stream == DMA2_Stream1 ) { dmaStream = DMA2_Stream1_IRQn; }
	else if(huartTx->DMA_Stream == DMA2_Stream2 ) { dmaStream = DMA2_Stream2_IRQn; }
	else if(huartTx->DMA_Stream == DMA2_Stream5 ) { dmaStream = DMA2_Stream5_IRQn; }
	else if(huartTx->DMA_Stream == DMA2_Stream6 ) { dmaStream = DMA2_Stream6_IRQn; }
	else if(huartTx->DMA_Stream == DMA2_Stream7 ) { dmaStream = DMA2_Stream7_IRQn; }
	else { dmaStream = 0; }

	return dmaStream;
}
