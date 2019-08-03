#include "nrf24.h"
#define ADR_WIDTH 5

#define TX_PLOAD_WIDTH 5
#define SYSCLK_FREQ_40MHz   40000000
uint8_t TX_ADDRESS[ADR_WIDTH] = {0xb1,0xb2,0xb3,0xb4,0xb5};
uint8_t RX_ADDRESS[ADR_WIDTH] = {0xb1,0xb2,0xb3,0xb4,0xb5};
uint8_t TX_ADDRESS2[ADR_WIDTH] = {0xc2,0xc2,0xc2,0xc2,0xc2};
uint8_t RX_ADDRESS2[ADR_WIDTH] = {0xc2,0xc2,0xc2,0xc2,0xc2};
//uint8_t RX_BUF[TX_PLOAD_WIDTH] = {0};
volatile uint8_t RX_BUF2[TX_PLOAD_WIDTH] = {0};
tPipeSettings RxPipes[6];
void delay_us(__IO uint32_t us)
{
  volatile unsigned int tick = 0;

  while (us--)
  {
    while (tick < 5)
    {
      tick++;
    }
    tick = 0;
  }
}
//==============================================================================


//==============================================================================
// Процедура программной задержки ~1 мс
//==============================================================================
void delay_ms(__IO uint32_t ms)
{
  while (ms--)
  {
    delay_us(1000);
  }
}
//==============================================================================
// Процедура получает массив 8-битных слов
//==============================================================================
void SPI_recv8b(SPI_TypeDef* SPIx, uint8_t *pBuff, uint16_t Len)
{
    for (uint16_t i = 0; i < Len; i++)
    {
        while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET);
        SPI_I2S_SendData(SPIx, 0xFF);
        while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_BSY) == SET) ;
        //while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_RXNE) == RESET);
        *(pBuff++) = SPI_I2S_ReceiveData(SPIx);
        delay_us(10);
    }
}
//==============================================================================

//==============================================================================
// Процедура отправляет 1 байт и возвращает принятый байт
//==============================================================================
uint8_t SPI_SendRecvByte(SPI_TypeDef* SPIx, uint8_t TxByte)
{
    while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET);
    SPI_I2S_SendData(SPIx, TxByte);
    while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_BSY) == SET) ;
    //while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_RXNE) == RESET);
    return SPI_I2S_ReceiveData(SPIx);
}
//==============================================================================

//==============================================================================
// Процедура отправляет массив 8-битных слов
//==============================================================================
void SPI_send8b(SPI_TypeDef* SPIx,volatile  uint8_t *pBuff,volatile uint16_t Len)
{
    volatile uint8_t tmp;
    for (uint16_t i = 0; i < Len; i++)
    {
        tmp = *pBuff;
        SPI_I2S_SendData(SPIx, *pBuff++);
        while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_BSY) == SET) ;
    }
}
//==============================================================================
uint8_t NRF24_ReadState(void)
{
    volatile uint8_t tmp;
    tmp = SPI_SendRecvByte(NRF24_SPI, 0xFF);

    return tmp;
}
//==============================================================================
uint8_t NRF24_Read(SPI_TypeDef* SPIx, uint8_t Cmd, uint8_t *pBuff, uint8_t Len)
{
    CS_OFF;

    // Передаём байт команды, параллельно принимаем байт состояния
    uint8_t State = SPI_SendRecvByte(SPIx, Cmd);
    // Принимаем указанное кол-во байт
    SPI_recv8b(SPIx, pBuff, Len);

    CS_ON;

    return State;
}

//==============================================================================

uint8_t NRF24_Write(SPI_TypeDef* SPIx, uint8_t Cmd, uint8_t *pBuff, uint8_t Len)
{
    CS_OFF

            // Передаём байт команды, параллельно принимаем байт состояния
            uint8_t State = SPI_SendRecvByte(SPIx, Cmd);
    // Передаём указанное кол-во байт
    SPI_send8b(SPIx, pBuff, Len);

    CS_ON
            return State;
}

//==============================================================================

uint8_t NRF24_ReadReg(SPI_TypeDef* SPIx,uint8_t addr)

{
    uint8_t Reg = 0;
    NRF24_Read(SPIx, addr |= R_REGISTER, &Reg, 1);
    return Reg;
}

//==============================================================================

uint8_t NRF24_WriteReg(SPI_TypeDef* SPIx, uint8_t RegAddr, uint8_t Value)
{

    return NRF24_Write(SPIx, W_REGISTER | RegAddr, &Value, 1);
}

//==============================================================================

void NRF24_WriteRegs(SPI_TypeDef* SPIx, uint8_t RegAddr, uint8_t *Data,uint8_t Len)
{

    NRF24_Write(SPIx, W_REGISTER | RegAddr, Data, Len);
}

//==============================================================================

uint8_t NRF24_Read_Rs(SPI_TypeDef* SPIx,uint8_t addr,uint8_t *Reg, uint8_t Len)
{
    uint8_t State;
    State = NRF24_Read(SPIx, addr | R_REGISTER, Reg, Len);
    return State;
}

//==============================================================================


uint8_t NRF24_FlushRX(SPI_TypeDef* SPIx)
{

    CS_OFF

            SPI_SendRecvByte(NRF24_SPI,FLUSH_RX);
    delay_us(1);
    CS_ON
}

//==============================================================================

uint8_t NRF24_FlushTX(SPI_TypeDef* SPIx)
{
    CS_OFF

            SPI_SendRecvByte(NRF24_SPI,FLUSH_TX);
    delay_us(1);
    CS_ON
}

//==============================================================================

//==============================================================================
void NRF24_AutoRetrasmission_Setup(uint8_t TryCount, uint8_t TryPeriod)
{
    NRF24_WriteReg(NRF24_SPI, SETUP_RETR, TryCount | (TryPeriod << 4));
}

//==============================================================================

void NRF24_SetOutputPower(uint8_t OPower){
    volatile uint8_t RegValue = 0;
    RegValue = NRF24_ReadReg(NRF24_SPI,RF_SETUP);
    RegValue &= 0xF9; //обнуляем 2-1 биты регистра RF_SETUP
    NRF24_WriteReg(NRF24_SPI,RF_SETUP , OPower | RegValue);
    RegValue = NRF24_ReadReg(NRF24_SPI,RF_SETUP);
}

//==============================================================================

void     NRF24_SetDataRate(uint8_t DataRate){
    volatile uint8_t RegValue=0;
    RegValue = NRF24_ReadReg(NRF24_SPI,RF_SETUP);
    RegValue &= 0xD7; //обнуляем 5,3 биты регистра RF_SETUP
    NRF24_WriteReg(NRF24_SPI,RF_SETUP , DataRate | RegValue | (1<<7)|(1<<4));
    RegValue = NRF24_ReadReg(NRF24_SPI,RF_SETUP);
}

//==============================================================================

void NRF24_SetCRCLen(uint8_t CRCLen){
    volatile uint8_t RegValue;
    RegValue = NRF24_ReadReg(NRF24_SPI,CONFIG);
    RegValue &= 0xF3; //обнуляем 3,2 биты регистра CONFIG
    if(CRCLen == 0)
    {
        NRF24_WriteReg(NRF24_SPI,CONFIG , RegValue);
    }
    else if(CRCLen == 1)
        NRF24_WriteReg(NRF24_SPI,CONFIG , RegValue | 0x08);

    else if(CRCLen == 2)
        NRF24_WriteReg(NRF24_SPI,CONFIG , RegValue | CONFIG_EN_CRC | CONFIG_CRC2);
    delay_us(10000);

    RegValue = NRF24_ReadReg(NRF24_SPI,CONFIG);
}

//==============================================================================

void NRF24_ResetStateFlags(uint8_t Flags){
    volatile uint8_t RegValue;
    RegValue = NRF24_ReadState();
    RegValue &= (0xF0 ^ Flags);
    NRF24_WriteReg(NRF24_SPI,STATUS , RegValue | Flags);

}

//==============================================================================

void NRF24_SetChannel(uint8_t Channel){
    Channel &= 0x7F;
    NRF24_WriteReg(NRF24_SPI,RF_CH , Channel);
}

//==============================================================================

uint8_t NRF24_Write_TxPayload(uint8_t *pBuff, uint8_t Len)
{
    if (Len > 32)
        Len = 32;
    return NRF24_Write(NRF24_SPI, WR_TX_PLOAD, pBuff, Len);
}

//==============================================================================

uint8_t NRF24_Read_RxPayload(uint8_t *pBuff, uint8_t Len)
{
    if (Len > 32)
        Len = 32;
    return NRF24_Read(NRF24_SPI, RD_RX_PLOAD, pBuff, Len);
}

//==============================================================================

void NRF24_Set_PTX_Mode(void)
{
    CE_RESET
            // Очищаем буферы FIFO
            NRF24_FlushRX(NRF24_SPI);
    NRF24_FlushTX(NRF24_SPI);
}

//==============================================================================

void NRF24_Set_PRX_Mode(uint8_t* Addr)
{

    // Переключаемся в режим приёмника и будим nRF24 (PowerUp)
    volatile uint8_t RegValue;
    RegValue = NRF24_ReadReg(NRF24_SPI,CONFIG);
    RegValue &= 0xFC; //сбрасываем 1 и 0 биты регистра CONFIG

    NRF24_WriteReg(NRF24_SPI,CONFIG, RegValue | CONFIG_PRX | CONFIG_PWR_UP);
    RegValue = NRF24_ReadReg(NRF24_SPI,CONFIG);

    // Сбрасываем флаги прерываний nRF24
    NRF24_ResetStateFlags(STATUS_MAX_RT | STATUS_TX_DS | STATUS_RX_DR);

    // Восстанавливаем адрес Rx для соединения №0, т.к. он мог быть затёрт при передаче
    NRF24_WriteRegs(NRF24_SPI, RX_ADDR_P0, Addr, 3);

    // Очищаем буферы FIFO

    NRF24_FlushRX(NRF24_SPI);
    NRF24_FlushTX(NRF24_SPI);

    CE_SET

            // Задержка как минимум 130 мкс
            delay_us(135);
}

//==============================================================================

void NRF24_RxPipe_Setup(uint8_t Pipe, uint8_t *pAddress, uint8_t PayloadSize)
{
    volatile uint8_t RegValue;
    // Некорректный номер соединения (pipe)
    if (Pipe >= NRF24_PipesNum)
        return;
    // Ограничиваем длину пакета
    if (PayloadSize > 32)
        PayloadSize = 32;


    // Для соединений 2..5 пишем только один последний байт адреса, остальные должны совпадать с байтами адреса соединения №1
    // Для соединений 0..1 пишем все 5 байт адреса
    if (Pipe < 2)
        NRF24_WriteRegs(NRF24_SPI, RX_ADDR_P0, pAddress, ADR_WIDTH);

    else
        NRF24_WriteReg(NRF24_SPI, RX_ADDR_P0 + Pipe, pAddress);

    // Устанавливаем размер пакета для соединения
    NRF24_WriteReg(NRF24_SPI, RX_PW_P0 + Pipe,0);
    NRF24_WriteReg(NRF24_SPI, EN_RXADDR, 0x03);
    NRF24_WriteReg(NRF24_SPI,DYNPD,3);
    RegValue = NRF24_ReadReg(NRF24_SPI,FEATURE);
    RegValue &= 0xFB; //сбрасываем 1 и 0 биты регистра FEATURE
    NRF24_WriteReg(NRF24_SPI,FEATURE, 0x04);
    // Включаем приём из указанного соединения (pipe)
    // RegValue = NRF24_ReadReg(NRF24_SPI, EN_RXADDR);
    //RegValue |= (1 << Pipe);

}

//==============================================================================
void NRF24_ToggleFeatures(void)

{

    CS_OFF;

    SPI_SendRecvByte(NRF24_SPI,ACTIVATE);
    delay_us(1);
    SPI_SendRecvByte(NRF24_SPI,0x73);
    CS_ON;
}

void NRF24L01_RX_Mode(void)

{

    volatile uint8_t regval=0x00;

    regval = NRF24_ReadReg(NRF24_SPI,CONFIG);

    //разбудим модуль и переведём его в режим приёмника, включив биты PWR_UP и PRIM_RX

    regval |= (1<<PWR_UP)|(1<<PRIM_RX);

    NRF24_WriteReg(NRF24_SPI,CONFIG,regval);

    CE_SET;

    delay_us(150); //Задержка минимум 130 мкс

    // Flush buffers

    NRF24_FlushRX(NRF24_SPI);

    NRF24_FlushTX(NRF24_SPI);

}

void NRF24L01_TX_Mode(void)

{

    volatile uint8_t regval=0x00;

    regval = NRF24_ReadReg(NRF24_SPI,CONFIG);

    //разбудим модуль и переведём его в режим приёмника, включив биты PWR_UP и PRIM_RX
    regval &= 0xFC;
    regval |= (1<<PWR_UP);

    NRF24_WriteReg(NRF24_SPI,CONFIG,regval);

    CE_RESET;

    delay_us(150); //Задержка минимум 130 мкс

    // Flush buffers

    NRF24_FlushRX(NRF24_SPI);

    NRF24_FlushTX(NRF24_SPI);

}
uint8_t R_SEND(uint8_t *buf,uint8_t len)
{
    volatile uint8_t REG;

    NRF24_Write(NRF24_SPI,WR_TX_PLOAD,buf,len);
    delay_us(2000);
    CE_SET
            delay_us(1000);
    CE_RESET
            return 1;
}
void R_READ(uint8_t *pBuff)
{
   volatile uint8_t REG;
   CE_RESET;
   NRF24_Read_Rs(NRF24_SPI,RD_RX_PLOAD,pBuff,5);
   REG = NRF24_ReadState();

   NRF24_WriteReg(NRF24_SPI,STATUS, REG | 0x40);
   CE_SET;
   delay_us(2000);
}
void NRF24_Init(void)
{

    volatile uint8_t RegVal;
    CS_ON;
    delay_us(5000);
    CE_SET;
    delay_us(10000);
    CE_RESET;
#if NRF_isTX
    NRF24_FlushTX(NRF24_SPI);

    // Сбрасываем флаги прерываний nRF24
    NRF24_ResetStateFlags(STATUS_MAX_RT | STATUS_TX_DS | STATUS_RX_DR);
    NRF24_WriteRegs(NRF24_SPI,TX_ADDR,TX_ADDRESS,ADR_WIDTH);
    NRF24_WriteRegs(NRF24_SPI,RX_ADDR_P0,RX_ADDRESS,ADR_WIDTH); //RX_ADDR = TX_ADDR !!!

   // NRF24_WriteReg(NRF24_SPI,EN_AA,0x01);
    NRF24_WriteReg(NRF24_SPI,EN_RXADDR,0x01);
    NRF24_WriteReg(NRF24_SPI,RF_CH,40);

    NRF24_WriteReg(NRF24_SPI,RF_SETUP,0x0F);
    NRF24_WriteReg(NRF24_SPI,CONFIG,0x0E);
#else
    RegVal = NRF24_ReadState();

    CE_RESET;

    NRF24_WriteRegs(NRF24_SPI,RX_ADDR_P0,RX_ADDRESS,ADR_WIDTH);
    NRF24_WriteRegs(NRF24_SPI,RX_ADDR_P1,RX_ADDRESS2,ADR_WIDTH);
    NRF24_Read_Rs(NRF24_SPI,RX_ADDR_P1,RX_BUF2,5);
    RegVal = NRF24_ReadReg(NRF24_SPI,0x0C);

    NRF24_WriteReg(NRF24_SPI,EN_AA,0x03);
    NRF24_WriteReg(NRF24_SPI,RX_PW_P0,0x05);
    NRF24_WriteReg(NRF24_SPI,RX_PW_P1,0x05);
    //NRF24_WriteReg(NRF24_SPI,0x13,0x05);
    NRF24_WriteReg(NRF24_SPI,EN_RXADDR,7);
    NRF24_WriteReg(NRF24_SPI,RF_CH,40);

    NRF24_WriteReg(NRF24_SPI,RF_SETUP,0x08);

    NRF24_WriteReg(NRF24_SPI,CONFIG,0x0F);

    delay_us(2000);

    CE_SET;

#endif

    // Очищаем буферы Rx/Tx
    //NRF24_FlushRX(NRF24_SPI);


    //NRF24_Set_PRX_Mode(Addr);
    // NRF24_RxPipe_Setup(0,Addr,21);


    // Настраиваем автоповтор передачи
    // NRF24_AutoRetrasmission_Setup(5, 2);          // 5 попыток с периодом 0.75 мс
    // RegVal = NRF24_ReadReg(NRF24_SPI,SETUP_RETR);
    // Устанавливаем мощность передатчика
    //NRF24_SetOutputPower(RF_SETUP_PWR_0dBm);    // 0dBm
    //delay_us(100);
    //   NRF24_SetCRCLen(2);
    // Устанавливаем частоту работы
    //NRF24_SetDataRate(RF_SETUP_DR_1M);//NRF24_DataRate_2MBps);
    //delay_us(100);
    // Устанавливаем канал
    //    NRF24_SetChannel(20);
    //    delay_us(100);
    //    NRF24_WriteReg(NRF24_SPI,SETUP_AW,1);
    //    delay_us(100);

    //    delay_us(100);

}
uint8_t NRF24_Recv( uint8_t *pBuff)
{
    volatile uint8_t status = 0;
    status = NRF24_ReadReg(NRF24_SPI,CONFIG);
    delay_us(1);
    status = NRF24_ReadState();
    uint8_t Len = 0;

    // Сброс флага окончания отправки ACK
    if (status & TX_DS)
        NRF24_ResetStateFlags(TX_DS);
    // Был принят пакет и флаг RX_DR ещё не сброшен
    if (status & RX_DR)
    {
        // Читаем номер соединения (pipe)
        Len = (status  & 0x0E) >> 1;

        // Читаем тело пакета
        if (Len == 7 || Len == 6)
            ;
        else

            LED_TGL
                    NRF24_Read_RxPayload(pBuff, Len);

        // Сбрасываем флаг RX_DR
        NRF24_ResetStateFlags(RX_DR);
    }

    return Len;
}
//==============================================================================
int8_t NRF24_Send( uint8_t *pBuff, uint8_t Len)
{
    volatile uint8_t State,RegVal;
    int8_t ReturnVal = -1;
    uint16_t NRF24_TO=0;


    // Пишем адреса на передачу и приём с соединения №0
    //nrf24_write_regs(nrf24_SPIx, NRF24_REG_RX_ADDR_P0, pAddress, 5);
    NRF24_WriteRegs(NRF24_SPI, TX_ADDR, TX_ADDRESS, ADR_WIDTH);

    NRF24_Set_PTX_Mode();

    // Переключаемся в режим передатчика и будим nRF24 (PowerUp)
    RegVal = NRF24_ReadReg(NRF24_SPI, CONFIG);
    RegVal &= 0xFC;
    // Переключаемся в режим передатчика (PTX) Будим nRF24, если он спал
    NRF24_WriteReg(NRF24_SPI, CONFIG, RegVal | CONFIG_PWR_UP);


    // Задержка минимум 130 мкс
    delay_us(135);

    // Пишем в nRF24 тело пакета на передачу
    NRF24_Write_TxPayload(pBuff, Len);
    RegVal = NRF24_ReadReg(NRF24_SPI, 0x17);
    // Выдаём положительный импульс на ножку CE
    CE_SET
            delay_us(20);         // Задержка минимум 10 мкс но мы ждем 20
    CE_RESET

            // Ждём событий от nRF24
            NRF24_TO = NRF24_Tx_TimeOut;
    do
    {
        State = NRF24_ReadState();
        RegVal = NRF24_ReadReg(NRF24_SPI, 0x17);
        delay_us(100);
        NRF24_TO--;
    }
    while ( (!(State & (MAX_RT | TX_DS))) && NRF24_TO);

    if (NRF24_TO)
        NRF24_ResetStateFlags(MAX_RT| TX_DS);

    // Переключаемся в режим приёмника и переводим nRF24 в режим PowerDown
    RegVal = NRF24_ReadReg(NRF24_SPI, CONFIG);
    RegVal &= 0xFC;
    // Переключаемся в режим приёмника (PRX) Усыпляем nRF24
    NRF24_WriteReg(NRF24_SPI, CONFIG, RegVal | CONFIG_PRX);


    if (State & TX_DS)// Пакет был успешно отправлен (ACK получен), возвращаем кол-во перезапросов
    {
        // Читаем счётчики отправки
        ReturnVal =  NRF24_ReadReg(NRF24_SPI, OBSERVE_TX);
    }

    NRF24_Set_PRX_Mode(RX_ADDRESS);

    return ReturnVal;
}
