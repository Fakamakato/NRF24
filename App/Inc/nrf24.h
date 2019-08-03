#ifndef NRF24_H_

#define NRF24_H_

//------------------------------------------------

#include "stm32f10x_conf.h"

//------------------------------------------------

#define CS_GPIO_PORT GPIOB

#define CS_PIN GPIO_Pin_12

#define CS_ON CS_GPIO_PORT->BSRR = CS_PIN;

#define CS_OFF CS_GPIO_PORT->BRR = CS_PIN;

#define CE_GPIO_PORT GPIOB

#define CE_PIN GPIO_Pin_0

#define CE_RESET CE_GPIO_PORT->BRR = CE_PIN;

#define CE_SET CE_GPIO_PORT->BSRR = CE_PIN;

#define IRQ_GPIO_PORT GPIOB

#define IRQ_PIN GPIO_Pin_1

#define IRQ IRQ_GPIO_PORT->IDR & IRQ_PIN

#define LED_GPIO_PORT GPIOC

#define LED_PIN GPIO_Pin_13

#define LED_ON LED_GPIO_PORT->BSRR = LED_PIN;

#define LED_OFF LED_GPIO_PORT->BRR = LED_PIN;

#define LED_TGL LED_GPIO_PORT->ODR ^= LED_PIN;

#define NRF24_SPI SPI2

//------------------------------------------------



//------------------------------------------------

#define CONFIG 0x00 //'Config' register address
#define CONFIG_PWR_UP   (1<<1)
#define CONFIG_PRX      (1<<0)
#define CONFIG_EN_CRC   0x08
#define CONFIG_CRC1     0x00
#define CONFIG_CRC2     0x04
#define EN_AA 0x01 //'Enable Auto Acknowledgment' register address

#define EN_RXADDR 0x02 //'Enabled RX addresses' register address

#define SETUP_AW 0x03 //'Setup address width' register address

#define SETUP_RETR 0x04 //'Setup Auto. Retrans' register address
#define SETUP_RETR_W_250 0x00
#define SETUP_RETR_W_500 0x10
#define SETUP_RETR_W_750 0x20
#define SETUP_RETR_RC_0 0x00
#define SETUP_RETR_RC_5 0x05
#define SETUP_RETR_RC_10 0x0A
#define SETUP_RETR_RC_15 0x0F

#define RF_CH 0x05 //'RF channel' register address

#define RF_SETUP 0x06 //'RF setup' register address
#define RF_SETUP_PWR_0dBm 0x06
#define RF_SETUP_PWR_6dBm 0x04
#define RF_SETUP_PWR_12dBm 0x02
#define RF_SETUP_PWR_18dBm 0x00
#define RF_SETUP_DR_250k 0x20
#define RF_SETUP_DR_1M 0x00
#define RF_SETUP_DR_2M 0x08

#define STATUS 0x07 //'Status' register address
#define STATUS_RX_DR (1<<6)
#define STATUS_TX_DS (1<<5)
#define STATUS_MAX_RT (1<<4)

#define RX_ADDR_P0 0x0A //'RX address pipe0' register address

#define RX_ADDR_P1 0x0B //'RX address pipe1' register address

#define TX_ADDR 0x10 //'TX address' register address

#define RX_PW_P0 0x11 //'RX payload width, pipe0' register address

#define RX_PW_P1 0x12 //'RX payload width, pipe1' register address

#define FIFO_STATUS 0x17 //'FIFO Status Register' register address

#define DYNPD 0x1C

#define FEATURE 0x1D

#define R_STATE 0xFF

#define OBSERVE_TX 0x08

//------------------------------------------------

#define PRIM_RX 0x00 //RX/TX control (1: PRX, 0: PTX)

#define PWR_UP 0x01 //1: POWER UP, 0:POWER DOWN

#define RX_DR 0x40 //Data Ready RX FIFO interrupt

#define TX_DS 0x20 //Data Sent TX FIFO interrupt

#define MAX_RT 0x10 //Maximum number of TX retransmits interrupt

//------------------------------------------------
#define R_REGISTER 0x00
#define W_REGISTER 0x20 //запись в регистр
#define ACTIVATE 0x50 //

#define RD_RX_PLOAD 0x61 // Define RX payload register address

#define WR_TX_PLOAD 0xA0 // Define TX payload register address

#define FLUSH_TX 0xE1

#define FLUSH_RX 0xE2

//------------------------------------------------

#define CMD_FLUSH_TX      0xE1
#define CMD_FLUSH_RX      0xE2
//------------------------------------------------

#define NRF24_PipesNum 6

//------------------------------------------------

#define NRF24_Tx_TimeOut 500

//------------------------------------------------
typedef struct
{
  uint8_t bDynPayLoad   :1;
  uint8_t PayLoadLen    :6;
  uint8_t Address[5];
} tPipeSettings;

typedef struct
{
    uint8_t bPRIM_RX      :1;     // 00 Режим работы (0 = PTX; 1 = PRX)
    uint8_t bPWR_UP       :1;     // 01 Режим электропитания (1 = Power Up, 0 = Power Down)
    uint8_t bCRC_O        :1;     // 02 Размер поля CRC (0 = 1 байт, 1 = 2 байта)
    uint8_t bEN_CRC       :1;     // 03 Enable CRC
    uint8_t bMASK_MAX_RT  :1;     // 04 Прерывание по событию MAX_RT на ножке IRQ (0 = Включено; 1 = Отключено)
    uint8_t bMASK_TX_DS   :1;     // 05 Прерывание по событию TX_DS на ножке IRQ (0 = Включено; 1 = Отключено)
    uint8_t bMASK_RX_DR   :1;     // 06 Прерывание по событию RX_DR на ножке IRQ (0 = Включено; 1 = Отключено)
    uint8_t bRes          :1;     // 07 Reserved
} tNrf24Reg00h; // CONFIG       Configuration Register

typedef struct
{
    uint8_t bARC          :4;     // 00-03 Кол-во попыток передачи пакета при отсутствии ACK от приёмника
    uint8_t bARD          :4;     // 04-07 Пауза между попытками ((N + 1) * 250мкс)
} tNrf24Reg04h; // SETUP_RETR   Setup of Automatic Retransmission

typedef struct
{
    uint8_t bLNA_HCURR    :1;     // 00 Коэффициент усиления приёмника
    uint8_t bRF_PWR       :2;     // 01-02 Мощность передатчика (0 = -18dBm; 1 = -12dBm; 2 = -6dBm; 3 = 0dBm)
    uint8_t bRF_DR_HIGH   :1;     // 03 Частота передачи (старший бит). 00 = 1Mbps; 10 = 2Mbps; 01 = 250Kbps
    uint8_t bPLL_LOCK     :1;     // 04 Force PLL lock signal. Only used in test
    uint8_t bRF_DR_LOW    :1;     // 05 Частота передачи (младший бит). 00 = 1Mbps; 10 = 2Mbps; 01 = 250Kbps
    uint8_t bRes          :1;     // 06 Reserved
    uint8_t bCONT_WAVE    :1;     // 07 Enable continuous carrier transmit
} tNrf24Reg06h; // RF_SETUP     RF Setup Register

typedef struct
{
    uint8_t bARC_CNT      :4;     // 00-03 Счётчик повторов отправки пакета. Счётчик сбрасывается когда начинается отправка нового пакета.
    uint8_t bPLOS_CNT     :4;     // 04-07 Счётчик потерянных пакетов (не превышает 15). Счётчик сбрасывается записью в RF_CH.
} tNrf24Reg08h; // OBSERVE_TX   Transmit observe register

typedef struct
{
    uint8_t bEN_DYN_ACK   :1;     // 00 Enables the W_TX_PAYLOAD_NOACK command
    uint8_t bEN_ACK_PAY   :1;     // 01 Enables Payload with ACK
    uint8_t bEN_DPL       :1;     // 02 Enables Dynamic Payload Length
    uint8_t bRes          :5;     // 03-07 Reserved
} tNrf24Reg1Dh; // FEATURE

void NRF24_Init(void);

uint8_t NRF24_ReadReg(SPI_TypeDef* SPIx,uint8_t addr);

void NRF24_Read_Buf(uint8_t addr,uint8_t *pBuf,uint8_t bytes);
uint8_t NRF24_WriteReg(SPI_TypeDef* SPIx, uint8_t RegAddr, uint8_t Value);
uint8_t NRF24_Read_Rs(SPI_TypeDef* SPIx,uint8_t addr,uint8_t* Reg, uint8_t Len);
void NRF24_RxPipe_Setup(uint8_t Pipe, uint8_t *pAddress, uint8_t PayloadSize);
int8_t NRF24_Send(uint8_t *pBuff, uint8_t Len);
uint8_t NRF24_Recv( uint8_t *pBuff);
uint8_t NRF24_ReadState(void);
void NRF24L01_TX_Mode(void);
uint8_t R_SEND(uint8_t *buf,uint8_t len);
void R_READ(uint8_t *pBuff);
void NRF24L01_RX_Mode(void);
void delay_us(__IO uint32_t us);
void delay_ms(__IO uint32_t ms);
#endif /* NRF24_H_ */


