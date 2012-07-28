/******************************************************************
 *****                                                        *****
 *****  Name: cs8900.c                                        *****
 *****  Ver.: 1.0                                             *****
 *****  Date: 07/05/2001                                      *****
 *****  Auth: Andreas Dannenberg                              *****
 *****        HTWK Leipzig                                    *****
 *****        university of applied sciences                  *****
 *****        Germany                                         *****
 *****        adannenb@et.htwk-leipzig.de                     *****
 *****  Func: ethernet packet-driver for use with LAN-        *****
 *****        controller CS8900 from Crystal/Cirrus Logic     *****
 *****                                                        *****
 ******************************************************************/

#include <msp430x14x.h>
#include "cs8900.h"


// constants
const unsigned char MyMAC[] =                    // "M1-M2-M3-M4-M5-M6"
{
  MYMAC_1, MYMAC_2, MYMAC_3,
  MYMAC_4, MYMAC_5, MYMAC_6
};

const TInitSeq InitSeq[] =
{
  PP_IA, MYMAC_1 + (MYMAC_2 << 8),               // set our MAC as Individual Address
  PP_IA + 2, MYMAC_3 + (MYMAC_4 << 8),
  PP_IA + 4, MYMAC_5 + (MYMAC_6 << 8),
  PP_LineCTL, SERIAL_RX_ON | SERIAL_TX_ON,       // configure the Physical Interface
  PP_RxCTL, RX_OK_ACCEPT | RX_IA_ACCEPT | RX_BROADCAST_ACCEPT
};

// configure port-pins for use with LAN-controller,
// reset it and send the configuration-sequence
// (InitSeq[])

void Init8900(void)
{
  unsigned int i;

  P3SEL = 0x30;                                  // reserve P3.4 and P3.5 for rs232
  P3OUT = IOR | IOW;                             // reset outputs, control lines high
  P3DIR = 0xFF;                                  // port 3 as output (all pins but rs232)

  P5SEL = 0;                                     // select standard port functions
  P5OUT = 0;                                     // reset outputs
  P5DIR = 0xFF;                                  // switch data port to output

  Write8900(ADD_PORT, PP_SelfCTL);
  Write8900(DATA_PORT, POWER_ON_RESET);          // Reset the Ethernet-Controller

  Write8900(ADD_PORT, PP_SelfST);
  while (!(Read8900(DATA_PORT) & INIT_DONE));    // wait until chip-reset is done
  
  for (i = 0; i < sizeof InitSeq / sizeof (TInitSeq); i++) // configure the CS8900
  {
    Write8900(ADD_PORT, InitSeq[i].Addr);
    Write8900(DATA_PORT, InitSeq[i].Data);
  }
}

// writes a word in little-endian byte order to
// a specified port-address

void Write8900(unsigned char Address, unsigned int Data)
{
  P5DIR = 0xFF;                                  // data port to output
  P3OUT = IOR | IOW | Address;                   // put address on bus
  
  P5OUT = Data;                                  // write low order byte to data bus
  P3OUT &= ~IOW;                                 // toggle IOW-signal
  P3OUT = IOR | IOW | (Address + 1);             // and put next address on bus

  P5OUT = Data >> 8;                             // write high order byte to data bus
  P3OUT &= ~IOW;                                 // toggle IOW-signal
  P3OUT |= IOW;
}

// writes a word in little-endian byte order to TX_FRAME_PORT

void WriteFrame8900(unsigned int Data)
{
  P5DIR = 0xFF;                                  // data port to output
  P3OUT = IOR | IOW | TX_FRAME_PORT;             // put address on bus
  
  P5OUT = Data;                                  // write low order byte to data bus
  P3OUT &= ~IOW;                                 // toggle IOW-signal
  P3OUT = IOR | IOW | (TX_FRAME_PORT + 1);       // and put next address on bus

  P5OUT = Data >> 8;                             // write high order byte to data bus
  P3OUT &= ~IOW;                                 // toggle IOW-signal
  P3OUT |= IOW;
}

// copies bytes from MCU-memory to frame port
// NOTES: * an odd number of byte may only be transfered
//          if the frame is written to the end!
//        * MCU-memory MUST start at word-boundary

void CopyToFrame8900(void *Source, unsigned int Size)
{
  P5DIR = 0xFF;                                  // data port to output
  
  while (Size > 1) {
    WriteFrame8900(*((unsigned int *)Source));
	Source += 2;
    Size -= 2;
  }
  
  if (Size)                                      // if odd num. of bytes...
    WriteFrame8900(*(unsigned char *)Source);    // write leftover byte (the LAN-controller
}                                                // ignores the highbyte)

// reads a word in little-endian byte order from
// a specified port-address

unsigned int Read8900(unsigned char Address)
{
  unsigned int ReturnValue;

  P5DIR = 0x00;                                  // data port to input
  P3OUT = IOR | IOW | Address;                   // put address on bus

  P3OUT &= ~IOR;                                 // IOR-signal low
  
  ReturnValue = P5IN;                            // get low order byte from data bus
  P3OUT = IOR | IOW | (Address + 1);             // IOR high and put next address on bus
  P3OUT &= ~IOR;                                 // IOR-signal low

  ReturnValue |= P5IN << 8;                      // get high order byte from data bus
  
  P3OUT |= IOR;
  
  return ReturnValue;
}

// reads a word in little-endian byte order from RX_FRAME_PORT

unsigned int ReadFrame8900(void)
{
  unsigned int ReturnValue;

  P5DIR = 0x00;                                  // data port to input
  P3OUT = IOR | IOW | RX_FRAME_PORT;             // access to RX_FRAME_PORT

  P3OUT &= ~IOR;                                 // IOR-signal low
  
  ReturnValue = P5IN;                            // get 1st byte from data bus (low-byte)
  P3OUT = IOR | IOW | (RX_FRAME_PORT + 1);       // IOR high and put next address on bus
  P3OUT &= ~IOR;                                 // IOR-signal low

  ReturnValue |= P5IN << 8;                      // get 2nd byte from data bus (high-byte)
  
  P3OUT |= IOR;
  
  return ReturnValue;
}

// reads a word in big-endian byte order from RX_FRAME_PORT
// (useful to avoid permanent byte-swapping while reading
// TCP/IP-data)

unsigned int ReadFrameBE8900(void)
{
  unsigned int ReturnValue;

  P5DIR = 0x00;                                  // data port to input
  P3OUT = IOR | IOW | RX_FRAME_PORT;             // access to RX_FRAME_PORT

  P3OUT &= ~IOR;                                 // IOR-signal low
  
  ReturnValue = P5IN << 8;                       // get 1st byte from data bus (high-byte)
  P3OUT = IOR | IOW | (RX_FRAME_PORT + 1);       // IOR high and put next address on bus
  P3OUT &= ~IOR;                                 // IOR-signal low

  ReturnValue |= P5IN;                           // get 2nd byte from data bus (low-byte)
  
  P3OUT |= IOR;
  
  return ReturnValue;
}

// reads a word in little-endian byte order from
// a specified port-address
// NOTE: this func. xfers the high-byte 1st, must be used to
//       access some special registers (e.g. RxStatus)

unsigned int ReadHB1ST8900(unsigned char Address)
{
  unsigned int ReturnValue;

  P5DIR = 0x00;                                  // data port to input
  P3OUT = IOR | IOW | (Address + 1);             // put address on bus

  P3OUT &= ~IOR;                                 // IOR-signal low
  
  ReturnValue = P5IN << 8;                       // get high order byte from data bus
  P3OUT = IOR | IOW | Address;                   // IOR high and put next address on bus
  P3OUT &= ~IOR;                                 // IOR-signal low

  ReturnValue |= P5IN;                           // get low order byte from data bus
  
  P3OUT |= IOR;
  
  return ReturnValue;
}

// copies bytes from frame port to MCU-memory
// NOTES: * an odd number of byte may only be transfered
//          if the frame is read to the end!
//        * MCU-memory MUST start at word-boundary

void CopyFromFrame8900(void *Dest, unsigned int Size)
{
  while (Size > 1) {
    *((unsigned int *)Dest) = ReadFrame8900();
	Dest +=2;
    Size -= 2;
  }
  
  if (Size)                                      // check for leftover byte...
    *(unsigned char *)Dest = ReadFrame8900();    // the LAN-Controller will return 0
}                                                // for the highbyte

// does a dummy read on frame-I/O-port
// NOTE: only an even number of bytes is read!

void DummyReadFrame8900(unsigned int Size)       // discards an EVEN number of bytes
{                                                // from RX-fifo
  while (Size > 1) {
    ReadFrame8900();
    Size -= 2;
  }
}

// requests space in CS8900's on-chip memory for
// storing an outgoing frame

void RequestSend(unsigned int FrameSize)
{
  Write8900(TX_CMD_PORT, TX_START_ALL_BYTES);
  Write8900(TX_LEN_PORT, FrameSize);
}

// check if CS8900 is ready to accept the
// frame we want to send

unsigned int Rdy4Tx(void)
{
  Write8900(ADD_PORT, PP_BusST);
  return (Read8900(DATA_PORT) & READY_FOR_TX_NOW);
}

