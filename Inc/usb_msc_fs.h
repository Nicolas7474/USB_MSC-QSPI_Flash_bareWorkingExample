#ifndef STM32F469_USBDFS_MSC_H_INCLUDED
#define STM32F469_USBDFS_MSC_H_INCLUDED


/****************************************************************
* STM32F469
* USB OTG FS device (MSC) implementation
*
* Nicolas Prata 2026
*
****************************************************************/

#include <string.h>
#include "stm32f469xx.h"

/******************************************************************************
* This section contains some macros and defines for better compatibility
* with other libs. If you find any conflicting types, delete them from here
*******************************************************************************/

#define USB_CLEAR_INTERRUPT(IRQ)    ((USB_OTG_FS->GINTSTS) &= (IRQ))
#define USB_MASK_INTERRUPT(IRQ)     (USB_OTG_FS->GINTMSK &= ~(IRQ))
#define USB_UNMASK_INTERRUPT(IRQ)   (USB_OTG_FS->GINTMSK |= (IRQ))

#define CLEAR_IN_EP_INTERRUPT(NUM, IRQ)          (USB_EP_IN(NUM)->DIEPINT = (IRQ))
#define CLEAR_OUT_EP_INTERRUPT(NUM, IRQ)         (USB_EP_OUT(NUM)->DOEPINT = (IRQ))

#define USB_OTG_DEVICE      		((USB_OTG_DeviceTypeDef *) (USB_OTG_FS_PERIPH_BASE + USB_OTG_DEVICE_BASE))

#define USB_EP_OUT(i) 			((USB_OTG_OUTEndpointTypeDef *) ((USB_OTG_FS_PERIPH_BASE +  USB_OTG_OUT_ENDPOINT_BASE) + ((i) * USB_OTG_EP_REG_SIZE)))
#define USB_EP_IN(i)    		((USB_OTG_INEndpointTypeDef *)	((USB_OTG_FS_PERIPH_BASE + USB_OTG_IN_ENDPOINT_BASE) + ((i) * USB_OTG_EP_REG_SIZE)))

#define USB_OTG_DFIFO(i)    *(__IO uint32_t *)((uint32_t)USB_OTG_FS_PERIPH_BASE  + USB_OTG_FIFO_BASE + (i) * USB_OTG_FIFO_SIZE)

#define USB_OTG_PCGCCTL      ((USB_OTG_PCGCCTLTypeDef *)( USB_OTG_FS_PERIPH_BASE + USB_OTG_PCGCCTL_BASE))

#define USB_CLEAR_INTERRUPT(IRQ)    ((USB_OTG_FS->GINTSTS) &= (IRQ))
#define USB_MASK_INTERRUPT(IRQ)     (USB_OTG_FS->GINTMSK &= ~(IRQ))
#define USB_UNMASK_INTERRUPT(IRQ)   (USB_OTG_FS->GINTMSK |= (IRQ))

typedef struct{
	__IO uint32_t PCGCCTL;
}
USB_OTG_PCGCCTLTypeDef;



/***************************************************
 * 			User settings
***************************************************/
/*On the F469, the total dedicated USB FIFO RAM is only 1.25 KB (1280 bytes). This is shared between:
    Rx FIFO (All OUT endpoints)
    Tx FIFOs (Each IN endpoint)
*/

#define FLUSH_FIFO_TIMEOUT		2000

/* In FS mode, the maximum packet size (MPS) for Bulk and Interrupt endpoints is fixed at 64 bytes. A 128-byte (32-word) FIFO allows
   for efficient double-buffering (2x 64bytes) to manage data transmission smoothly within 1ms frames */

#define RX_FIFO_SIZE			128				// 35 - minimum working / 128 - other values don't work
#define TX_EP0_FIFO_SIZE		64				// 16 - minimum working  64 - other values don't work
#define TX_EP1_FIFO_SIZE		128

/* FIFO total Memory = RX_FIFO_SIZE + TX_EP0_FIFO_SIZE + TX_EP1_FIFO_SIZE = 320 words of 4 bytes = 1280 bits = max allowed FIFO RAM */

#define EP1_DTFXSTS_SIZE    	TX_EP1_FIFO_SIZE	/* TX FIFO empty level */
#define EP1_MIN_DTFXSTS_LVL		16		/* Minimum TX FIFO empty level */

#define MAX_MSC_EP0_TX_SIZ  	64    /* Max TX transaction size for EP0. "64" means that you can send maximum one packet of max size
in TXCallback, then you send the rest bytes (or ZLP) in next function call. Max USB_OTG_DIEPTSIZ_XFRSIZ value.     */
#define MAX_MSC_EP1_TX_SIZ  	128   /* Max TX transaction size for EP1.  Max USB_OTG_DIEPTSIZ_XFRSIZ value.      */


#define DOEPT_TRANSFER_SIZE		0x40	// = 64 ; don't use higher values for transfer size/packets nb, Rx speed will be a few % slower !
#define DOEPT_TRANSFER_PCT 		0x01	// Value used in DOEPTSIZ for EP0 and EP1


/***************************************************
 * 			EP statuses
***************************************************/
#define EP_READY 				0U
#define EP_BUSY  				1U
#define EP_ZLP   				2U
#define EP_WAIT  				3U

/***************************************************
 * 			EP functions return values
***************************************************/

#define EP_OK				1U
#define EP_FAILED			0U

/***************************************************
 * 			Device states
 ***************************************************/
typedef enum {
	DEVICE_STATE_DEFAULT =			0,
	DEVICE_STATE_RESET =			1,
	DEVICE_STATE_ADDRESSED =		2,
	DEVICE_STATE_CONFIGURED =		8,
	DEVICE_STATE_TX_PR =			16, /* TX in PRogress */
} eDeviceState;

typedef enum {
    MSC_STATE_IDLE = 0,    // Waiting for a CBW
    MSC_STATE_DATA_OUT,    // Host is sending data (Write10)
    MSC_STATE_DATA_IN,     // Host is waiting for data (Read10, Inquiry)
    MSC_STATE_SEND_CSW,    // Ready to send the status packet
	MSC_STATE_WAIT_CSW_DONE, // Waiting for the CSW itself to finish
    MSC_STATE_ERROR        // Something went wrong (Stall)
} MSC_State_t;

// __attribute__((packed)) qualifier ensure the compiler doesn't add padding between the fields
typedef struct __attribute__((packed)) {
    uint32_t dSignature;          // 4 Bytes - "USBC" (0x43425355) - Windows uses this signature to uniquely identify a disk
    uint32_t dTag;                // 4 Bytes - Unique ID sent by host, must be echoed in CSW
    uint32_t dDataTransferLength; // 4 Bytes - Number of bytes host expects to transfer
    uint8_t  bmFlags;             // 1 Byte -Bit 7: 0=Out (Host to Dev), 1=In (Dev to Host)
    uint8_t  bLUN;                // 1 Byte - Logical Unit Number (usually 0)
    uint8_t  bCBLength;           // 1 Byte - Length of the SCSI command in the CB array (1-16)
    uint8_t  CB[16];              // 16 Bytes - SCSI Command Descriptor Block (CDB): the actual SCSI command bytes (Inquiry, Read10, etc)
} USB_MSC_CBW_t; 				  // Command Block Wrapper (CBW) structure:     CB[0]	0x28	The OpCode for READ(10)
							      /*											CB[1]	Flags	Usually 0x00
																				CB[2-5]	LBA	Logical Block Address (The "address" of the data on the disk)
																				CB[6]	Group	Usually 0x00
																				CB[7-8]	Length	Number of logical blocks (sectors) to read
																				CB[9]	Control	Usually 0x00		*/

// After you finish a command (like SCSI_INQUIRY), you must send this 13-byte packet back to the Host on the Bulk IN endpoint:
typedef struct __attribute__((packed)) {
    uint32_t dSignature;          // Always 0x53425355 ("USBS")
    uint32_t dTag;                // Must match the dTag from the CBW
    uint32_t dDataResidue;        // Difference between expected and actual data
    uint8_t  bStatus;             // 0 = Success, 1 = Failed, 2 = Phase Error
} USB_MSC_CSW_t;



/***************************************************
 * 			SETUP stage request templates
***************************************************/

#define REQ_TYPE_HOST_TO_DEVICE_GET_DEVICE_DECRIPTOR	0x0680
#define REQ_TYPE_DEVICE_TO_HOST_SET_ADDRESS				0x0500
#define REQ_TYPE_DEVICE_TO_HOST_SET_CONFIGURATION		0x0900

#define DESCRIPTOR_TYPE_DEVICE							0x0100
#define DESCRIPTOR_TYPE_CONFIGURATION					0x0200
#define DESCRIPTOR_TYPE_LANG_STRING						0x0300
#define DESCRIPTOR_TYPE_MFC_STRING						0x0301
#define DESCRIPTOR_TYPE_PROD_STRING						0x0302
#define DESCRIPTOR_TYPE_SERIAL_STRING					0x0303
#define DESCRIPTOR_TYPE_CONFIGURATION_STRING			0x0304
#define DESCRIPTOR_TYPE_INTERFACE_STRING				0x0305
#define DESCRIPTOR_TYPE_DEVICE_QUALIFIER				0x0600

#define CLEAR_FEATURE_ENDP								0x0102

/***************************************************
* 		Endpoint structure
***************************************************/

typedef struct EndPointStruct{
	volatile uint16_t statusRx; // Since it can be modified in the ISR, forces the CPU to read the actual memory location
	volatile uint16_t statusTx; // every single time instead of using a cached value in a register

	volatile uint32_t rxCounter;
	volatile uint32_t txCounter;

	uint8_t *rxBuffer_ptr;
	uint8_t *txBuffer_ptr;

	uint32_t (*txCallBack)(void);
	uint32_t (*rxCallBack)(uint32_t param);
	uint32_t (*setTxBuffer)(uint8_t EPnum, uint8_t *txBuff, uint32_t len);

	volatile uint16_t totXferLen;

} EndPointStruct;

extern EndPointStruct EndPoint[];

/****************************************************
* 	Setup packet structure
* 	is used in union to access data both
* 	as structure and as raw data*
***************************************************/

typedef struct __attribute__((packed)){
    uint8_t   bmRequestType;
    uint8_t   bRequest;
    uint16_t  wValue;
    uint16_t  wIndex;
    uint16_t  wLength;
} USB_setup_req;	/* SETUP packet buffer. Always 8 bytes */


typedef union{
	USB_setup_req setup_pkt;
	uint32_t raw_data[2];
} USB_setup_req_data;



/*************************************************** *
 * 		Functions' declaration *
***************************************************/

/* init functions */
uint32_t USB_OTG_FS_Init(void);
void USB_OTG_FS_init_registers(void);
void enumerate_Reset(void);
void enumerate_Setup(void);
// void initEndPoints(void); -> static

/* FIFO access */
uint32_t write_Fifo(uint8_t dfifo, uint8_t *src, uint16_t len);
void read_Setup_Fifo(void);
void read_Fifo(uint8_t dfifo, uint16_t len);
// uint32_t DTFXSTS_timeout(uint8_t Epnum, uint32_t dtxfsts_val); -> static

/* Endpoint functions */
uint32_t USB_MSC_setTxBuffer(uint8_t EPnum, uint8_t *txBuff, uint32_t len);
uint32_t USB_MSC_transferTXCallback(uint8_t EPnum);
uint32_t USB_MSC_transferRXCallback_EP1(uint32_t param);
// inline void toggle_Rx_EP_Status(uint8_t EPnum, uint8_t param);

/* misc */
uint32_t USB_FlushTxFifo(uint32_t EPnum, uint32_t timeout);
uint32_t USB_FlushRxFifo(uint32_t timeout);

uint32_t check_USB_device_status(eDeviceState state);
void clear_USB_device_status(eDeviceState state);

/* User code functions */


extern volatile uint32_t msTicks;



/******************************************************************************
* USB MSC device descriptors
* borrowed from STMicroelectronics for educational purposes*
*******************************************************************************/

#define LOBYTE(x) (uint8_t)(x & ~0xFF00)
#define HIBYTE(x) (uint8_t)((x >> 8) & ~0xFF00)

/*Why "64" is the most common choice
Even though the STM32F469 is capable of High-Speed, many developers use the built-in FS PHY. In this case:  You must use 64.
If you set it to 128 or 256 in FS mode, the hardware cannot physically fit that much data into a single USB frame transaction,
and the host controller will flag a "babble" error or a PID sequence error.*/
#define USB_MSC_MAX_PACKET_SIZE		64

#define MSC_CMD_PACKET_SIZE	8  /* Control Endpoint Packet size */
#define EP0_SIZE			64
#define EP_COUNT			2


#define USBD_VID				1155
#define USBD_LANGID_STRING			1033
#define USBD_MANUFACTURER_STRING		"STMicroelectronics"
#define USBD_PID_FS				0x5721
#define USBD_PRODUCT_STRING_FS			"STM32 Flash Mass Storage"

#define DEVICE_DESCRIPTOR_LENGTH		18
#define CONFIGURATION_DESCRIPTOR_LENGTH 32

#define LANG_DESCRIPTOR_LENGTH			4
#define MFC_DESCRIPTOR_LENGTH			38
#define PRODUCT_DESCRIPTOR_LENGTH		44
#define SERIAL_DESCRIPTOR_LENGTH		26
#define DEVICE_QUALIFIER_LENGTH			10
#define INTERFACE_STRING_LENGTH			28
#define CONFIG_STRING_LENGTH			22
#define MSC_LINE_CODING_LENGTH			7

#define MSC_GET_MAX_LUN                  0xFEA1
#define MSC_BOT_RESET                    0xFF21  // Host-to-Device, Class, Interface


/* String Standard Device Descriptor - has a fixed-length block of 18 bytes.*/
static const uint8_t deviceDescriptor[DEVICE_DESCRIPTOR_LENGTH] = {
		DEVICE_DESCRIPTOR_LENGTH, // bLength - Size of this descriptor in bytes: 18 bytes
		0x01,                     // bDescriptorType - DEVICE Descriptor Type: Device
		0x00, 0x02,               // bcdUSB - USB Specification Release Number in Binary-Coded Decimal: USB 2.00
		0x00,                     // bDeviceClass (Defined at Interface level) - not to lock the entire physical plug into one specific class.
								  // Since HID is almost always implemented at the interface level, this is the required approach.
		0x00,                     // bDeviceSubClass
		0x00,                     // bDeviceProtocol- These protocol codes are qualified by the value of the bDeviceClass and the bDeviceSubClass fields.
		EP0_SIZE,                 // bMaxPacketSize0 (64 bytes)
		LOBYTE(USBD_VID),		  // idVendor (0x0483 - STMicroelectronics)
		HIBYTE(USBD_VID),
		LOBYTE(USBD_PID_FS),   	  // Product ID (assigned by the manufacturer): 0x5720 (ST standard for MSC)
		HIBYTE(USBD_PID_FS),
		0x00, 0x02,               // bcdDevice Device release number in binary-coded decimal: 2.00
		0x01,                     // iManufacturer - Index of string descriptor describing manufacturer: 1
		0x02,                     // iProduct - Index of string descriptor describing	product: 2
		0x03,                     // iSerialNumber - Index of string descriptor describing the device’s serial number: 3
		0x01                      // bNumConfigurations - Number of possible configurations: 1
};

/* Configuration descriptor */
static const uint8_t configurationDescriptor[CONFIGURATION_DESCRIPTOR_LENGTH] = {

		// === Configuration Descriptor ===
		    0x09,                               // bLength: 9 bytes
		    0x02,                               // bDescriptorType: Configuration
		    0x20, 0x00,                         // wTotalLength: 32 bytes (9+9+7+7)
		    0x01,                               // bNumInterfaces: 1
		    0x01,                               // bConfigurationValue: 1
		    0x00,                               // iConfiguration: Index of string
		    0xC0,                               // bmAttributes: Self-powered
		    0x32,                               // bMaxPower: 100 mA

		    // === Interface Descriptor (MSC) ===
		    0x09,                               // bLength: 9 bytes
		    0x04,                               // bDescriptorType: Interface
		    0x00,                               // bInterfaceNumber: 0
		    0x00,                               // bAlternateSetting: 0
		    0x02,                               // bNumEndpoints: 2 (Bulk IN & OUT)
		    0x08,                               // bInterfaceClass: Mass Storage
		    0x06,                               // bInterfaceSubClass: SCSI Transparent
		    0x50,                               // bInterfaceProtocol: Bulk-Only Transport (BOT)
		    0x00,                               // iInterface: Index of string

		    // === Endpoint Descriptor (Bulk OUT) ===
		    0x07,                               // bLength: 7 bytes
		    0x05,                               // bDescriptorType: Endpoint
		    0x01,                               // bEndpointAddress: 0x01 (OUT)
		    0x02,                               // bmAttributes: Bulk
		    0x40, 0x00,                         // wMaxPacketSize: 64 bytes
		    0x00,                               // bInterval: Ignore for Bulk

		    // === Endpoint Descriptor (Bulk IN) ===
		    0x07,                               // bLength: 7 bytes
		    0x05,                               // bDescriptorType: Endpoint
		    0x81,                               // bEndpointAddress: 0x81 (IN)
		    0x02,                               // bmAttributes: Bulk
		    0x40, 0x00,                         // wMaxPacketSize: 64 bytes
		    0x00								// bInterval: Ignore for Bulk
};


/* Language string descriptor */
static const uint8_t languageStringDescriptor[LANG_DESCRIPTOR_LENGTH] = {
	LANG_DESCRIPTOR_LENGTH,				 /* USB_LEN_LANGID_STR_DESC */
	0x03,    			/* USB_DESC_TYPE_STRING */
	LOBYTE(USBD_LANGID_STRING),
	HIBYTE(USBD_LANGID_STRING)
};
/* Manufactor string descriptor */
static const uint8_t manufactorStringDescriptor[MFC_DESCRIPTOR_LENGTH] = {
	MFC_DESCRIPTOR_LENGTH,
	0x03,	/* USB_DESC_TYPE_STRING */
	'S', 0x00,
	'T', 0x00,
	'M', 0x00,
	'i', 0x00,
	'c', 0x00,
	'r', 0x00,
	'o', 0x00,
	'e', 0x00,
	'l', 0x00,
	'e', 0x00,
	'c', 0x00,
	't', 0x00,
	'r', 0x00,
	'o', 0x00,
	'n', 0x00,
	'i', 0x00,
	'c', 0x00,
	's', 0x00
};
/* Product string descriptor */
static const uint8_t productStringDescriptor[PRODUCT_DESCRIPTOR_LENGTH] = {
	PRODUCT_DESCRIPTOR_LENGTH,
	0x03,	/* USB_DESC_TYPE_STRING */
	'S', 0x00,
	'T', 0x00,
	'M', 0x00,
	'3', 0x00,
	'2', 0x00,
	' ', 0x00,
	'M', 0x00,
	'a', 0x00,
	's', 0x00,
	's', 0x00,
	' ', 0x00,
	's', 0x00,
	't', 0x00,
	'o', 0x00,
	'r', 0x00,
	'a', 0x00,
	'g', 0x00,
	'e', 0x00,
	' ', 0x00,
	' ', 0x00,
	' ', 0x00
};
/* Serial number string descriptor */
static const uint8_t serialNumberStringDescriptor[SERIAL_DESCRIPTOR_LENGTH] = {
	SERIAL_DESCRIPTOR_LENGTH,
	0x03,	/* USB_DESC_TYPE_STRING */
	0x34, 0x00,
	0x38, 0x00,
	0x05, 0x00,
	0x45, 0x00,
	0x37, 0x00,
	0x34, 0x00,
	0x46, 0x00,
	0x37, 0x00,
	0x36, 0x00,
	0x33, 0x00,
	0x30, 0x00,
	0x38, 0x00
};
/* Device qualifier string descriptor */
static const uint8_t deviceQualifierDescriptor[DEVICE_QUALIFIER_LENGTH] = {
	DEVICE_QUALIFIER_LENGTH,
	0x06,	/* Device Qualifier */
	0x00,
	0x02,
	0x00,
	0x00,
	0x00,
	0x40,
	0x01,
	0x00
};


static const uint8_t stringInterface[INTERFACE_STRING_LENGTH] = {
	INTERFACE_STRING_LENGTH,
	0x03,	/* USB_DESC_TYPE_STRING */
	'M', 0x00,
	'S', 0x00,
	'C', 0x00,
	' ', 0x00,
	'I', 0x00,
	'n', 0x00,
	't', 0x00,
	'e', 0x00,
	'r', 0x00,
	'f', 0x00,
	'a', 0x00,
	'c', 0x00,
	'e', 0x00
};

static const uint8_t configurationStringDescriptor[CONFIG_STRING_LENGTH] = {
	CONFIG_STRING_LENGTH,
	0x03,	/* USB_DESC_TYPE_STRING */
	'M', 0x00,
	'S', 0x00,
	'C', 0x00,
	' ', 0x00,
	'C', 0x00,
	'o', 0x00,
	'n', 0x00,
	'f', 0x00,
	'i', 0x00,
	'g', 0x00
};

#endif /* USB_DESC_H_INCLUDED */

