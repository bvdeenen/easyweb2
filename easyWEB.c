/******************************************************************
 *****                                                        *****
 *****  Name: easyweb.c                                       *****
 *****  Ver.: 1.0                                             *****
 *****  Date: 07/05/2001                                      *****
 *****  Auth: Andreas Dannenberg                              *****
 *****        HTWK Leipzig                                    *****
 *****        university of applied sciences                  *****
 *****        Germany                                         *****
 *****        adannenb@et.htwk-leipzig.de                     *****
 *****  Func: implements a dynamic HTTP-server by using       *****
 *****        the easyWEB-API                                 *****
 *****  Rem.: In IAR-C, use  linker option                    *****
 *****        "-e_medium_write=_formatted_write"              *****
 *****                                                        *****
 ******************************************************************/


#include <msp430x14x.h>
#include <msp430.h>
#include <legacymsp430.h>
#include "stdlib.h"
#include "stdio.h"
#include "string.h"
#include "easyweb.h"
#include "cs8900.h"                              // ethernet packet driver
#include "tcpip.h"                               // easyWEB TCP/IP stack

extern const unsigned char WebSide[] ;
#define bitset(var,bitno) ((var) |= 1 << (bitno))
#define bitclr(var,bitno) ((var) &= ~(1 << (bitno)))

#define          B1                 BIT4&P4IN         //B1 - P4.4
#define          B2                 BIT5&P4IN         //B2 - P4.5
#define          B3                 BIT6&P4IN         //B3 - P4.6
#define          B4                 BIT7&P4IN         //B4 - P4.7
#define          FREQ               BIT0&P1IN         //FREQuency input - P1.0
#define          DI1                BIT1&P1IN         //Digital Input 1 - P1.1
#define          DI2                BIT2&P1IN         //Digital Input 2 - P1.2
#define          DI3                BIT3&P1IN         //Digital Input 3 - P1.3
#define          DI4                BIT4&P1IN         //Digital Input 4 - P1.4
#define          DALLAS             BIT7&P1IN         //DALLAS input - P1.7
#define          P20                BIT0&P2IN         //P20 input
#define          SDA                BIT0&P4IN         //SDA
#define          SCL                BIT1&P4IN         //SCL
#define          STATUS_LED_ON      P2OUT &= ~BIT1    //STATUS_LED - P2.1
#define          STATUS_LED_OFF     P2OUT |= BIT1     //STATUS_LED - P2.1
#define          RELAY1_ON          P1OUT |= BIT5     //RELAY1 - P1.5
#define          RELAY1_OFF         P1OUT &= ~BIT5    //RELAY1 - P1.5
#define          RELAY2_ON          P1OUT |= BIT6     //RELAY1 - P1.6
#define          RELAY2_OFF         P1OUT &= ~BIT6    //RELAY1 - P1.6
#define          BUZ1_ON            P4OUT |= BIT2     //P4.2
#define          BUZ1_OFF           P4OUT &= ~BIT2    //P4.2
#define          BUZ2_ON            P4OUT |= BIT3     //P4.3
#define          BUZ2_OFF           P4OUT &= ~BIT3    //P4.3
#define          LCD_Data           P2OUT          
#define          _100us             66                //66 cycles *12 + 9 = 801 / 801*125ns = 100us
#define          _10us              6                 //6 cycles * 12 + 9 = 81 / 81*125ns=10us
#define          E                  3                 //P2.3
#define          RS                 2                 //P2.2
#define          CR                 0x0d
#define          LF                 0x0a
#define          BUTTON_TIME        100

#define		DISP_ON			0x0c	        //LCD control constants
#define		DISP_OFF		0x08	        //
#define		CLR_DISP		0x01    	//
#define		CUR_HOME		0x02	        //
#define		ENTRY_INC		0x06            //
#define		DD_RAM_ADDR		0x80	        //
#define		DD_RAM_ADDR2		0xc0	        //
#define		DD_RAM_ADDR3		0x28	        //
#define		CG_RAM_ADDR		0x40	        //


unsigned char TXData, RXData,i,j,k,temp,RX_flag,cntr,time_out;
const unsigned char UART_Message [] = "http://www.olimex.com/dev";
const unsigned char LCD_Message[] = "  Bart WEB ][    by OLIMEX Ltd. ";
const unsigned char DI1_Message[] = " DI1 ";
const unsigned char DI2_Message[] = " DI2 ";
const unsigned char DI3_Message[] = " DI3 ";
const unsigned char DI4_Message[] = " DI4 ";
const unsigned char DALLAS_Message[] = " DALLAS is Ok ";
const unsigned char FREQ_Message[] = "TIMER is clocked";
const unsigned char P6_error_Message[] = "    P6 error    ";
const unsigned char EXT_error_Message[] = "   EXT error    ";

void Delay (unsigned int a);
void UART_transmit (unsigned char transmit_data);
void InitUART0 (void);
void Delayx100us(unsigned char b);
void SEND_CHAR (unsigned char c);
void SEND_CMD (unsigned char e);
void _E(void);
void InitLCD(void);
void stopP6(void);
void stop_ext(void);

int main(void)
{

	InitOsc();  
	InitPorts();           
	InitUART0();
	InitLCD();

	TCPLowLevelInit();                            //after TCPLowLevelInit() UCLK = ACLK = MCLK/4 = 2 000 000 Hz   

	UART_transmit (CR);
	UART_transmit (LF);  
	for (i=0; i!=26; i++)  UART_transmit (UART_Message[i]); 
	UART_transmit (CR);
	UART_transmit (LF);  

	for (i=0; i!=32; i++)  
	{
		SEND_CHAR(LCD_Message[i]);
		if (i==15) SEND_CMD (DD_RAM_ADDR2);
	}  

	SEND_CMD(DD_RAM_ADDR);    
	RX_flag=0; 
	cntr = 0;   

	HTTPStatus = 0;                                // clear HTTP-server's flag register
	TCPLocalPort = TCP_PORT_HTTP;                  // set port we want to listen to

	while (1)                                      // repeat forever
	{

		//--------------buttons scan---------------------------------------------------------

		if ((B1) == 0)                                 //B1 is pressed
		{
			STATUS_LED_ON;                             //switch on status_led
			SEND_CMD(CLR_DISP);
			SEND_CMD(DD_RAM_ADDR);
			cntr=0;
		}
		else  STATUS_LED_OFF;                           //B1 is released 

		if ((B2) == 0)
		{      
			//        time_out = BUTTON_TIME;
			//        while (time_out != 0) 
			//        if ((B2) == 0) time_out--;
			//        else time_out = BUTTON_TIME;  

			Delayx100us(50);
			RELAY1_ON;                                        
		}
		else
		{      
			//        time_out = BUTTON_TIME;
			//        while (time_out != 0) 
			//        if ((B2) != 0) time_out--;
			//        else time_out = BUTTON_TIME;  

			Delayx100us(50);
			RELAY1_OFF;                                        
		}                   

		if ((B3) == 0)   
		{
			Delayx100us(50);
			RELAY2_ON;                     //B3 is pressed
		}
		else  
		{
			Delayx100us(50);
			RELAY2_OFF;                               //B3 is released
		}  

		while ((B4) == 0)                               //B4 is pressed
		{
			BUZ1_OFF;                                 
			BUZ2_ON;
			Delay(_100us);                            //buzzer with 5 000 Hz
			BUZ2_OFF;
			BUZ1_ON;
			Delay(_100us);          
		}    
		BUZ1_OFF;                                      //B4 is released
		BUZ2_OFF;      

		//--------UART0 receiv scan------------------------------------------------------------------     

		if (RX_flag == 1)                              //new receiv byte
		{                    
			STATUS_LED_ON;
			if (cntr == 0) 
			{
				SEND_CMD(CLR_DISP);
				SEND_CMD(DD_RAM_ADDR);                //set address for first row
			}
			SEND_CHAR(RXData);          
			if(cntr == 15) SEND_CMD(DD_RAM_ADDR2);    //set address for second row
			if(cntr++ == 31) cntr = 0;           
			RX_flag = 0;            
			STATUS_LED_OFF;
		}
		//---------Digital Inputs scan--------------------------------------------------------------

		if ((DI1) == 0)  for (i=0 ; i != 5; i++)UART_transmit(DI1_Message[i]);  
		if ((DI2) == 0)  for (i=0 ; i != 5; i++)UART_transmit(DI2_Message[i]);  
		if ((DI3) == 0)  for (i=0 ; i != 5; i++)UART_transmit(DI3_Message[i]);  
		if ((DI4) == 0)  for (i=0 ; i != 5; i++)UART_transmit(DI4_Message[i]);  

		//---------DALLAS scan ---------------------------------------------------------------------

		if ((DALLAS) == 0)                      
		{
			cntr=0;
			SEND_CMD(CLR_DISP);
			SEND_CMD(DD_RAM_ADDR);
			for (i=0 ; i!= 14; i++) SEND_CHAR(DALLAS_Message[i]);
		}   

		//---------FREQ scan ----------------------------------------------------------------------      

		if ((FREQ) != 0) 
		{
			cntr=0;
			SEND_CMD(CLR_DISP);
			SEND_CMD(DD_RAM_ADDR);
			for (i=0 ; i!= 16; i++) SEND_CHAR(FREQ_Message[i]);
		}   

		//***********************************************************************************    
		//this is the end of my programm
		//***********************************************************************************

		if (!(SocketStatus & SOCK_ACTIVE)) {
			TCPPassiveOpen();   // listen for incoming TCP-connection
		}
		DoNetworkStuff();                                      // handle network and easyWEB-stack
		// events
		HTTPServer();
	}                               
	return 0;
}

// This function implements a very simple dynamic HTTP-server.
// It waits until connected, then sends a HTTP-header and the
// HTML-code stored in memory. Before sending, it replaces
// some special strings with dynamic values.
// NOTE: For strings crossing page boundaries, replacing will
// not work. In this case, simply add some extra lines
// (e.g. CR and LFs) to the HTML-code.



void HTTPServer(void)
{
  if (SocketStatus & SOCK_CONNECTED)             // check if somebody has connected to our TCP
  {
    if (SocketStatus & SOCK_DATA_AVAILABLE)      // check if remote TCP sent data
      TCPReleaseRxBuffer();                      // and throw it away

    if (SocketStatus & SOCK_TX_BUF_RELEASED)     // check if buffer is free for TX
    {
      if (!(HTTPStatus & HTTP_SEND_PAGE))        // init byte-counter and pointer to webside
      {                                          // if called the 1st time
        HTTPBytesToSend = strlen(WebSide) - 1;   // get HTML length, ignore trailing zero
        PWebSide = (unsigned char *)WebSide;     // pointer to HTML-code
      }

      if (HTTPBytesToSend > MAX_TCP_TX_DATA_SIZE)     // transmit a segment of MAX_SIZE
      {
        if (!(HTTPStatus & HTTP_SEND_PAGE))           // 1st time, include HTTP-header
        {
          memcpy(TCP_TX_BUF, GetResponse, sizeof(GetResponse) - 1);
          memcpy(TCP_TX_BUF + sizeof(GetResponse) - 1, PWebSide, MAX_TCP_TX_DATA_SIZE - sizeof(GetResponse) + 1);
          HTTPBytesToSend -= MAX_TCP_TX_DATA_SIZE - sizeof(GetResponse) + 1;
          PWebSide += MAX_TCP_TX_DATA_SIZE - sizeof(GetResponse) + 1;
        }
        else
        {
          memcpy(TCP_TX_BUF, PWebSide, MAX_TCP_TX_DATA_SIZE);
          HTTPBytesToSend -= MAX_TCP_TX_DATA_SIZE;
          PWebSide += MAX_TCP_TX_DATA_SIZE;
        }
          
        TCPTxDataCount = MAX_TCP_TX_DATA_SIZE;   // bytes to xfer
        InsertDynamicValues();                   // exchange some strings...
        TCPTransmitTxBuffer();                   // xfer buffer
      }
      else if (HTTPBytesToSend)                  // transmit leftover bytes
      {
        memcpy(TCP_TX_BUF, PWebSide, HTTPBytesToSend);
        TCPTxDataCount = HTTPBytesToSend;        // bytes to xfer
        InsertDynamicValues();                   // exchange some strings...
        TCPTransmitTxBuffer();                   // send last segment
        TCPClose();                              // and close connection
        HTTPBytesToSend = 0;                     // all data sent
      }

      HTTPStatus |= HTTP_SEND_PAGE;              // ok, 1st loop executed
    }
  }
  else
    HTTPStatus &= ~HTTP_SEND_PAGE;               // reset help-flag if not connected
}

// samples and returns the AD-converter value of channel 7
// (associated with Port P6.7)

unsigned int GetAD7Val(void)
{
  ADC12CTL0 = ADC12ON | SHT0_15 | REF2_5V | REFON;   // ADC on, int. ref. on (2,5 V),
                                                     // single channel single conversion
  ADC12CTL1 = ADC12SSEL_2 | ADC12DIV_7 | CSTARTADD_0 | SHP;// MCLK / 8 = 1 MHz

  ADC12MCTL0 = SREF_1 | INCH_7;                  // int. ref., channel 7
  
  ADC12CTL0 |= ENC;                              // enable conversion
  ADC12CTL0 |= ADC12SC;                          // sample & convert
  
  while (ADC12CTL0 & ADC12SC);                   // wait until conversion is complete
  
  ADC12CTL0 &= ~ENC;                             // disable conversion

  return ADC12MEM0 / 41;                         // scale 12 bit value to 0..100%
}

// samples and returns AD-converter value of channel 10
// (MSP430's internal temperature reference diode)
// NOTE: to get a more exact value, 8-times oversampling is used

unsigned int GetTempVal(void)
{
  unsigned long ReturnValue;

  ADC12CTL0 = ADC12ON | SHT0_15 | MSH | REFON;   // ADC on, int. ref. on (1,5 V),
                                                 // multiple sample & conversion
  ADC12CTL1 = ADC12SSEL_2 | ADC12DIV_7 | CSTARTADD_0 | CONSEQ_1 | SHP;   // MCLK / 8 = 1 MHz

  ADC12MCTL0 = SREF_1 | INCH_10;                 // int. ref., channel 10
  ADC12MCTL1 = SREF_1 | INCH_10;                 // int. ref., channel 10
  ADC12MCTL2 = SREF_1 | INCH_10;                 // int. ref., channel 10
  ADC12MCTL3 = SREF_1 | INCH_10;                 // int. ref., channel 10
  ADC12MCTL4 = SREF_1 | INCH_10;                 // int. ref., channel 10
  ADC12MCTL5 = SREF_1 | INCH_10;                 // int. ref., channel 10
  ADC12MCTL6 = SREF_1 | INCH_10;                 // int. ref., channel 10
  ADC12MCTL7 = EOS | SREF_1 | INCH_10;           // int. ref., channel 10, last seg.
  
  ADC12CTL0 |= ENC;                              // enable conversion
  ADC12CTL0 |= ADC12SC;                          // sample & convert
  
  while (ADC12CTL0 & ADC12SC);                   // wait until conversion is complete
  
  ADC12CTL0 &= ~ENC;                             // disable conversion

  ReturnValue = ADC12MEM0;                       // sum up values...
  ReturnValue += ADC12MEM1;
  ReturnValue += ADC12MEM2;
  ReturnValue += ADC12MEM3;
  ReturnValue += ADC12MEM4;
  ReturnValue += ADC12MEM5;
  ReturnValue += ADC12MEM6;
  ReturnValue += ADC12MEM7;

  ReturnValue >>= 3;                             // ... and divide by 8

  if (ReturnValue < 2886) ReturnValue = 2886;    // lower bound (0% = 20°C)
  ReturnValue = (ReturnValue - 2886) / 2.43;     // convert AD-value to a temperature from
                                                 // 20°C...45°C represented by a value
                                                 // of 0...100%
  if (ReturnValue > 100) ReturnValue = 100;      // upper bound (100% = 45°C)

  return ReturnValue;
}

// searches the TX-buffer for special strings and replaces them
// with dynamic values (AD-converter results)

void InsertDynamicValues(void)
{
  unsigned char *Key;
  unsigned char NewKey[5];
  unsigned int i;
  
  if (TCPTxDataCount < 4) return;                     // there can't be any special string
  
  Key = TCP_TX_BUF;
  
  for (i = 0; i < (TCPTxDataCount - 3); i++)
  {
    if (*Key == 'A')
     if (*(Key + 1) == 'D')
       if (*(Key + 3) == '%')
         switch (*(Key + 2))
         {
           case '7' :                                 // "AD7%"?
           {
             sprintf(NewKey, "%3u", GetAD7Val());     // insert AD converter value
             memcpy(Key, NewKey, 3);                  // channel 7 (P6.7)
             break;
           }
           case 'A' :                                 // "ADA%"?
           {
             sprintf(NewKey, "%3u", GetTempVal());    // insert AD converter value
             memcpy(Key, NewKey, 3);                  // channel 10 (temp.-diode)
             break;
           }
         }
    Key++;
  }
}

// enables the 8MHz crystal on XT1 and use
// it as MCLK

void InitOsc(void)
{
  WDTCTL = WDTPW | WDTHOLD;                      // stop watchdog timer

  BCSCTL1 |= XTS;                                // XT1 as high-frequency
  _BIC_SR(OSCOFF);                               // turn on XT1 oscillator
                          
  do                                             // wait in loop until crystal is stable 
    IFG1 &= ~OFIFG;
  while (IFG1 & OFIFG);

  BCSCTL1 |= DIVA0;                              // ACLK = XT1 / 2
  BCSCTL1 &= ~DIVA1;
  
  IE1 &= ~WDTIE;                                 // disable WDT int.
  IFG1 &= ~WDTIFG;                               // clear WDT int. flag
  
  WDTCTL = WDTPW | WDTTMSEL | WDTCNTCL | WDTSSEL | WDTIS1; // use WDT as timer, flag each
                                                           // 512 pulses from ACLK
                                                           
  while (!(IFG1 & WDTIFG));                      // count 1024 pulses from XT1 (until XT1's
                                                 // amplitude is OK)

  IFG1 &= ~OFIFG;                                // clear osc. fault int. flag
  BCSCTL2 |= SELM0 | SELM1;                      // set XT1 as MCLK
}  

void InitPorts(void)
{
  P1SEL = 0;                                     // 
  P1OUT = 0;                                     //
  P1DIR = BIT5 | BIT6;                           //enable only Relay outputs

  P2SEL = 0;
  P2OUT = 0;
  P2DIR = ~BIT0;                                //only P2.0 is input
  
  P3SEL |= BIT4 | BIT5;                         //enable UART0    
  P3DIR |= BIT4;                                //enable TXD0 as output
  P3DIR &= ~BIT5;                               //enable RXD0 as input

  P4SEL = 0;
  P4OUT = 0;
  P4DIR = BIT2 | BIT3;                          //only buzzer pins are outputs

  P6SEL = 0x80;                                  
  P6OUT = 0;
  P6DIR = 0x00;                                  // all output
}

void Delay (unsigned int a)
{
  for (k=0 ; k != a; ++k);                      //9+a*12 cycles
}

void Delayx100us(unsigned char b)
{
  for (j=0; j!=b; ++j) Delay (_100us);
}

void UART_transmit (unsigned char Transmit_Data)              //UART0 Transmit Subroutine
{
  while ((IFG1 & UTXIFG0) == 0);                              //Wait for ready U0TXBUF
  U0TXBUF = Transmit_Data;                                    //send data
}

void InitUART0 (void)                           //UART0 init  
{
  BCSCTL1 &= ~DIVA0;                            // ACLK = XT1 / 4 = MCLK / 4
  BCSCTL1 |= DIVA1;  
  UCTL0 = CHAR;                                 //Sart bit, 8 data bits, no parity, 1 stop
  UTCTL0 = SSEL0;                               //ACLK is UART clock
  U0BR0 = 0xd0;                                 //2000000:9600=208
  U0BR1 = 0x00;
  UMCTL0 = 0x00;                                //no modulation
  ME1 |= UTXE0 | URXE0;                         //enable UART modul
  P3SEL |= 0x30;                                // P3.4,5 = USART0 TXD/RXD
  P3DIR |= BIT4;                                //enable TXD0 as output
  P3DIR &= ~BIT5;                               //enable RXD0 as input
  IE1 |= URXIE0;                                // Enable USART0 RX interrupt  
  _EINT();                                      //enable interrupt  
}

void _E(void)
{
        bitset(P2OUT,E);		//toggle E for LCD
	Delay(_10us);
	bitclr(P2OUT,E);
}

void SEND_CHAR (unsigned char d)
{
	Delayx100us(5);                 //.5ms	
	temp = d & 0xf0;		//get upper nibble	
	LCD_Data &= 0x0f;
	LCD_Data |= temp;               
	bitset(P2OUT,RS);     	        //set LCD to data mode
	_E();                           //toggle E for LCD
	temp = d & 0x0f;
	temp = temp << 4;               //get down nibble
	LCD_Data &= 0x0f;
	LCD_Data |= temp;
	bitset(P2OUT,RS);   	        //set LCD to data mode
	_E();                           //toggle E for LCD
}

void SEND_CMD (unsigned char e)
{
	Delayx100us(10);                //10ms
	temp = e & 0xf0;		//get upper nibble	
	LCD_Data &= 0x0f;
	LCD_Data |= temp;               //send CMD to LCD
	bitclr(P2OUT,RS);     	        //set LCD to CMD mode
	_E();                           //toggle E for LCD
	temp = e & 0x0f;
	temp = temp << 4;               //get down nibble
	LCD_Data &= 0x0f;
	LCD_Data |= temp;
	bitclr(P2OUT,RS);   	        //set LCD to CMD mode
	_E();                           //toggle E for LCD
}

void InitLCD(void)
{
    bitclr(P2OUT,RS);
    Delayx100us(250);                   //Delay 100ms
    Delayx100us(250);
    Delayx100us(250);
    Delayx100us(250);
    LCD_Data |= BIT4 | BIT5;            //D7-D4 = 0011
    LCD_Data &= ~BIT6 & ~BIT7;
    _E();                               //toggle E for LCD
    Delayx100us(100);                   //10ms
    _E();                               //toggle E for LCD
    Delayx100us(100);                   //10ms
    _E();                               //toggle E for LCD
    Delayx100us(100);                   //10ms
    LCD_Data &= ~BIT4;
    _E();                               //toggle E for LCD
    
    SEND_CMD(DISP_ON);
    SEND_CMD(CLR_DISP);   
}

void stopP6 (void)
{
    P6DIR = 0;
    cntr=0;
    SEND_CMD(CLR_DISP);
    SEND_CMD(DD_RAM_ADDR);
    for (i=0 ; i!= 16; i++) SEND_CHAR(P6_error_Message[i]);    
    while(1);
}

void stop_ext (void)
{
    P2DIR &= ~BIT0;
    P4DIR = 0;
    cntr=0;
    SEND_CMD(CLR_DISP);
    SEND_CMD(DD_RAM_ADDR);
    for (i=0 ; i!= 16; i++) SEND_CHAR(EXT_error_Message[i]);    
    while(1);
}

///*
interrupt(UART0RX_VECTOR)usart0_rx (void)
{
  RXData = RXBUF0; 
  UART_transmit (RXData+1);                  //transmit Echo + 1
  RX_flag = 1;                               //set RX_flag
}
//*/

