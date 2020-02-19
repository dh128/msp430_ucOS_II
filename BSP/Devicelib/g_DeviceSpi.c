/*
*********************************************************************************************************
*
*                          (c) Copyright 2003-2014; Micrium, Inc.; Weston, FL
*
*               All rights reserved.  Protected by international copyright laws.
*               Knowledge of the source code may NOT be used to develop a similar product.
*               Please help us continue to provide the Embedded community with the finest
*               software available.  Your honesty is greatly appreciated.
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*
*
*                                      Texas Instruments MSP430
*                                               on the
*                                          MSP-EXP430F5259LP
*                                          Evaluation Board
*
* Filename      : g_DeviceSpi.c
* Version       : V1.00
* Programmer(s) : GLZ
*********************************************************************************************************
*/
#include <bsp.h>


/*******************************************************************************
* ??????	: g_Device_SendByte_SPI2
* ????		: SPI2??????????
* ???????  : chr
* ???????  : ??
*******************************************************************************/
void g_Device_SendByte_SPI2(uint8_t chr)
{
	UCB2TXBUF = chr;        // Send the data sitting at the pointer DATA to the TX Buffer
	while(!(UCB2IFG & UCTXIFG));
}

/*******************************************************************************
* Function Name  : g_Device_SendNByte_SPI2
* Description    :
* Input para     : data,len
* Output para    : None
*******************************************************************************/
void g_Device_SendNByte_SPI2(uint8_t *data,uint8_t len)
{
	static uint8_t m = 0;
	for(m=0;m<len;m++)
	{
		UCB2TXBUF = data[m];
		while(!(UCB2IFG & UCTXIFG));
	}
}
/*******************************************************************************
* ??????	: g_Device_SPI2_ReadWriteByte ????SD??
* ????		: SPI??��????
* ???????  : TxData
* ???????  : RXBUF1
*******************************************************************************/
uint8_t g_Device_SPI2_ReadWriteByte(uint8_t TxData)
{
	uint8_t delayVar;

	while(!(UCB2IFG & UCTXIFG));
	UCB2TXBUF = TxData;           // Send the data sitting at the pointer DATA to the TX Buffer
	while(!(UCB2IFG & UCRXIFG));
	delayVar = UCB2RXBUF;

	return delayVar;
}

/*******************************************************************************
* ??????	: g_Device_SendByte_SPI3
* ????		: SPI3??????????
* ???????  : chr
* ???????  : ??
*******************************************************************************/
void g_Device_SendByte_SPI3(uint8_t chr)
{
	UCB3TXBUF = chr;        // Send the data sitting at the pointer DATA to the TX Buffer
	while(!(UCB3IFG & UCTXIFG));
}
/*******************************************************************************
* ??????	: SPI_B2_Init
* ????:    SPI_B2?????
* ???????  : ??
* ???????  : ??
*******************************************************************************/
void g_Device_SPI2_Init(void)
{
	P7SEL |= BIT2+BIT3+BIT4;				    // Set P7.2-4 as SPI peripheral
	P7DIR |= BIT2+BIT3+BIT5;
	UCB2CTL1 |= UCSWRST;               		    // Enable SW reset
	UCB2CTL0 |= UCCKPH+UCMSB+UCMST+UCSYNC;;      //+UCCKPL;  //3��SPIģʽ����UCxSTE=1ʱ�ӻ�ʹ��
	                                        	//UCCKPH(SD CARD��Ҫ�������ض�д�����UCCKPL==0��
												//8λ���SPI�����״̬Ϊ�ߵ�ƽ����λ��ǰ
	UCB2CTL1 |= UCSSEL__SMCLK;                  //ѡ��ο�ʱ��ΪSCMLK=16MHz
	UCB2BR0 = 6;								//6��Ƶ
	UCB2BR1 = 0;
	UCB2CTL1 &= ~UCSWRST;						//��ɼĴ�������
	//UCB2IE |= UCRXIE;							//ʹ���ж�
	OSBsp.Device.Spi2.WriteData = g_Device_SendByte_SPI2;
	OSBsp.Device.Spi2.WriteNData = g_Device_SendNByte_SPI2;
	OSBsp.Device.Spi2.WriteReadData = g_Device_SPI2_ReadWriteByte;
}

/*******************************************************************************
* Function Name  : g_Device_SendNByte_SPI3
* Description    :
* Input para     : data,len
* Output para    : None
*******************************************************************************/
void g_Device_SendNByte_SPI3(uint8_t *data,uint8_t len)
{
	static uint8_t m = 0;
	for(m=0;m<len;m++)
	{
		UCB3TXBUF = data[m];        // Send the data sitting at the pointer DATA to the TX Buffer
		while(!(UCB3IFG & UCTXIFG));
	}
}

/*******************************************************************************
* ??????	: g_Device_SPI3_Init
* ????		: SPI3?????
* ???????  : ??
* ???????  : ??
*******************************************************************************/
void g_Device_SPI3_Init(void)
{
	P2SEL |= BIT2+BIT3+BIT4;						// Set P2.2-5 as SPI peripheral
	UCB3CTL1 |= UCSWRST;               				// Enable SW reset
	UCB3CTL0 |= UCMSB+UCMST+UCSYNC+UCCKPL;			//3??SPI??????UCxSTE=1???????
													//8��????SPI?????????????????????��???
	UCB3CTL1 |= UCSSEL_2;                     		// ???��?????SCMLK=16MHz
	UCB3BR0 = 2;									//2???
	UCB3BR1 = 0;
	UCB3CTL1 &= ~UCSWRST;							//???????????
	UCB3IE |= UCRXIE;								//????��?

	OSBsp.Device.Spi3.WriteData = g_Device_SendByte_SPI3;
	OSBsp.Device.Spi3.WriteNData = g_Device_SendNByte_SPI3;
}



//------USCI_B2?��?????????-------------------------------------------------+
#pragma vector=USCI_B2_VECTOR
__interrupt void USCI_B2_ISR(void)
{
  switch(__even_in_range(UCB2IV,4))
  {
  case 0:break;                             // Vector 0 - no interrupt
  case 2:                                   // Vector 2 - RXIFG
      while(!(UCB2IFG&UCTXIFG));            // USCI_B0 TX buffer ready?
	  {
	  }
      break;
  case 4:break;                             // Vector 4 - TXIFG
  default: break;
  }
}
//------USCI_B3?��?????????-------------------------------------------------+
#pragma vector=USCI_B3_VECTOR
__interrupt void USCI_B3_ISR(void)
{
  switch(__even_in_range(UCB3IV,4))
  {
  case 0:break;                             // Vector 0 - no interrupt
  case 2:                                   // Vector 2 - RXIFG
      while(!(UCB3IFG&UCTXIFG));            // USCI_B0 TX buffer ready?
	  {
	  }
      break;
  case 4:break;                             // Vector 4 - TXIFG
  default: break;
  }
}

