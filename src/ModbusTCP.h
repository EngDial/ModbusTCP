#include <inttypes.h>
#include "Arduino.h"	

/* 
  MBTCP ModbusTCP = [MBAP] + [PDU] = [ {IDTRAN_HI} + {IDTRAN_LO} + {IDPROT_HI} + {IDPROT_LO} + {LEN_HI} + {LEN_LO} + {UID} ] + [{FUNC} + {DATA}]
  FCT=01 Read Out Discret or FCT=02 Read In Discret
        REQ = [MBAP] + [ {FUNC = 01 or 02} + {ADDR_HI} + {ADDR_LO} + {CNT_BITS_HI} + {CNT_BITS_LO} ]
        ACK = [MBAP] + [ {FUNC = 01 or 02} + {LEN_DATA_BYTES} + {DATA_ADDR0} + ... + {DATA_ADDR(CNT/8)} ]         // DATA_ADDR0 = {7,6,5,4,3,2,1,0}; DATA_ADDR1 = {15,14,13,12,11,10,9,8}; 
  FCT=03 Read Out Registers or FCT=04  Read In Registers
        REQ = [MBAP] + [ {FUNC = 03 or 04} + {ADDR_HI} + {ADDR_LO} + {CNT_REGS_HI} + {CNT_REGS_LO} ]
        ACK = [MBAP] + [ {FUNC = 03 or 04} + {LEN_DATA_BYTES} + {DATA_ADDR0_HI} + {DATA_ADDR0_LO} + ... + {DATA_ADDR(CNT)_HI} + {DATA_ADDR(CNT)_LO} ] // LEN - Length of DATA in Bytes;
  FCT=05 Write Single Discret
        REQ = [MBAP] + [ {FUNC = 05} + {ADDR_HI} + {ADDR_LO} + {DATA_HI} + {DATA_LO} ]         // DATA = FF 00 for ON; 00 00 for OFF
        ACK = REQ = [MBAP] + [ {FUNC = 05} + {ADDR_HI} + {ADDR_LO} + {DATA_HI} + {DATA_LO} ]
  FCT=06 Write Single Register
        REQ = [MBAP] + [ {FUNC = 06} + {ADDR_HI} + {ADDR_LO} + {DATA_HI} + {DATA_LO} ]         
        ACK = REQ = [MBAP] + [ {FUNC = 06} + {ADDR_HI} + {ADDR_LO} + {DATA_HI} + {DATA_LO} ]
  FCT=0F Write multiple Out Discret
        REQ = [MBAP] + [ {FUNC = 0F} + {ADDR_HI} + {ADDR_LO} + {CNT_BITS_HI} + {CNT_BITS_LO} + {LEN_DATA_BYTES} + {DATA_ADDR0} + ... + {DATA_ADDR(CNT/8)} ]
        ACK = [MBAP] + [ {FUNC = 0F} + {ADDR_HI} + {ADDR_LO} + {CNT_BITS_HI} + {CNT_BITS_LO} ]
  FCT=10 Write multiple Out Registers
        REQ = [MBAP] + [ {FUNC = 10} + {ADDR_HI} + {ADDR_LO} + {CNT_REGS_HI} + {CNT_REGS_LO} + {LEN_DATA_BYTES} + {DATA_ADDR0_HI} + {DATA_ADDR0_LO} + ... + {DATA_ADDR(CNT)_HI} + {DATA_ADDR(CNT)_LO} ]
        ACK = [MBAP] + [ {FUNC = 10} + {ADDR_HI} + {ADDR_LO} + {CNT_REGS_HI} + {CNT_REGS_LO} ]
  ERROR (len = 9)
        ACK = [MBAP] + [ {FUNC |= 80} + {ERROR} ]

01	Принятый код функции не может быть обработан.
02	Адрес данных, указанный в запросе, недоступен.
03	Значение, содержащееся в поле данных запроса, является недопустимой величиной.
04	Невосстанавливаемая ошибка имела место, пока ведомое устройство пыталось выполнить затребованное действие.
05	Ведомое устройство приняло запрос и обрабатывает его, но это требует много времени. Этот ответ предохраняет ведущее устройство от генерации ошибки тайм-аута.
06	Ведомое устройство занято обработкой команды. Ведущее устройство должно повторить сообщение позже, когда ведомое освободится.
07	Ведомое устройство не может выполнить программную функцию, заданную в запросе. Этот код возвращается для неуспешного программного запроса, использующего функции с номерами 13 или 14. Ведущее устройство должно запросить диагностическую информацию или информацию об ошибках от ведомого.
08	Ведомое устройство при чтении расширенной памяти обнаружило ошибку паритета. Ведущее устройство может повторить запрос, но обычно в таких случаях требуется ремонт.        
        
  */

enum MBTCP
{
//MBAP Header (Modbus Application Header)
  IDTRAN_HI = 0,  // 0 ID Transaction high byte
  IDTRAN_LO,      // 1 ID Transaction low byte
  IDPROT_HI,      // 2 ID Protocol high byte (always zero)
  IDPROT_LO,      // 3 ID Protocol low byte (always zero)
  LEN_HI,         // 4 Length of Packet high byte (1 + Length PDU)
  LEN_LO,         // 5 Length of Packet low byte
  UID, 				    // 6 Unit ID field
// PDU, Protocol Data Unit.
  FUNC, 			    // 7 Function code position
// DATA // Data for Function
  ADD_HI, 				// 8 Address high byte or ERROR code
  ADD_LO, 				// 9 Address low byte
  NB_HI, 				  // 10 Number of coils or registers high byte
  NB_LO, 				  // 11 Number of coils or registers low byte
  BYTE_CNT  			// 12 byte counter
};

enum MB_FC 
{
  MB_FC_NONE                     = 0,   // null operator
  MB_FC_READ_COILS               = 1,	  // FCT=1 -> read coils or digital outputs
  MB_FC_READ_DISCRETE_INPUT      = 2,	  // FCT=2 -> read digital inputs
  MB_FC_READ_REGISTERS           = 3,	  // FCT=3 -> read registers or analog outputs (hold registers)
  MB_FC_READ_INPUT_REGISTER      = 4,	  // FCT=4 -> read analog inputs (input registers)
  MB_FC_WRITE_COIL               = 5,	  // FCT=5 -> write single coil or output
  MB_FC_WRITE_REGISTER           = 6,	  // FCT=6 -> write single register
  MB_FC_WRITE_MULTIPLE_COILS     = 15,  // FCT=15 -> write multiple coils or outputs
  MB_FC_WRITE_MULTIPLE_REGISTERS = 16	  // FCT=16 -> write multiple registers
};

enum ERR_LIST 
{
  ERR_NOT_MASTER 	  = -1,	// Device in not MASTER mode
  ERR_POLLING       = -2,	// POLLING mode
  ERR_BUFF_OVERFLOW = -3,	// buffer OVERFLOW *
  ERR_BAD_CRC       = -4,	// error CRC
  ERR_EXCEPTION     = -5	// EXCEPTION
};

enum 
{ 
  NO_REPLY 			= 255, 	// device no reply
  EXC_FUNC_CODE 	= 1,	// Not support Func
  EXC_ADDR_RANGE 	= 2,  // Addr out of range
  EXC_REGS_QUANT 	= 3,  
  EXC_EXECUTE 		= 4 
};

const unsigned char fctsupported[] = // list of supported function
{ 
  MB_FC_READ_COILS,                   // FCT=1 -> read coils or digital outputs
  MB_FC_READ_DISCRETE_INPUT,          // FCT=2 -> read digital inputs
  MB_FC_READ_REGISTERS,               // FCT=3 -> read registers or analog outputs (hold registers)
  MB_FC_READ_INPUT_REGISTER,          // FCT=4 -> read analog inputs (input registers)
  MB_FC_WRITE_COIL,                   // FCT=5 -> write single coil or output
  MB_FC_WRITE_REGISTER,               // FCT=6 -> write single register
  MB_FC_WRITE_MULTIPLE_COILS,         // FCT=15 -> write multiple coils or outputs
  MB_FC_WRITE_MULTIPLE_REGISTERS      // FCT=16 -> write multiple registers
};

#define T35  		        (5)		// time = 3.5char
#define MAX_BUFFER      (64)	// maximum size for the communication buffer in bytes
#define MIN_SIZE_FRAME  (12)  //

class Modbus 
{
  private:
    HardwareSerial *port;		// Pointer to Serial class object
    uint8_t u8id;				    // 0 = master, 1..247 = slave number
    uint8_t u8serno;			  // serial port: 0-Serial, 1..3-Serial1..Serial3
    uint8_t u8txenpin;			// flow control pin: 0=USB or RS-232 mode, >0=RS-485 mode
    uint8_t u8state;			  // serial port state
    uint8_t u8lastError;		// last error
    uint8_t au8Buffer[MAX_BUFFER];	// buffer of telegram
    uint8_t u8BufferSize;		// number of bytes in buffer
    uint8_t u8lastRec;			// number of bytes last receive
    uint16_t *au16regs;			// point of 16bits registers
    uint16_t u16InCnt;			// count reserve temegrams
    uint16_t u16OutCnt;			// count send temegrams
    uint16_t u16errCnt;			// count errors
    uint32_t u32time;			  // timeout for reserve
    uint8_t u8regsize;			// size modbus mem

    void sendTxBuffer(); 		// send buffer
    int8_t getRxBuffer(); 	// reserve buffer
    uint8_t validateRequest(); 			// validate Request
    int8_t process_FC1(uint16_t *regs);   // FC1
    int8_t process_FC3(uint16_t *regs);   // FC3
    int8_t process_FC5(uint16_t *regs);   // FC5
    int8_t process_FC6(uint16_t *regs);   // FC6
    int8_t process_FC15(uint16_t *regs);  // FC15
    int8_t process_FC16(uint16_t *regs);  // FC16
    void buildException(uint8_t u8exception ); // build exception message

  public:
    Modbus(uint8_t u8id, uint8_t u8serno, uint8_t u8txenpin);	// init ID, serial, TXEpin
    void begin();							// config serial
    void begin(long u32speed, uint8_t u8config);
    uint16_t getInCnt();			// get number of incoming messages
    uint16_t getOutCnt(); 		// get number of outcoming messages
    uint16_t getErrCnt(); 		// get error counter
    uint8_t getState();       // get State
    int8_t poll( uint16_t *regs, uint8_t u8size ); // cyclic poll for slave
    uint8_t getLastError();		// get last error message
    void end(); 							// finish any communication and release serial communication port
};

//=====================================
// init ID, serial, TXEpin
Modbus::Modbus(uint8_t u8id, uint8_t u8serno, uint8_t u8txenpin) 
{
  this->u8id = u8id;
  this->u8serno = (u8serno > 3) ? 0 : u8serno;
  this->u8txenpin = u8txenpin;
}
//=====================================
void Modbus::begin(long u32speed, uint8_t u8config) 
{
  switch ( u8serno ) 
  {
    #if defined(Serial1)
	  case 1:
        port = &Serial1;
        break;
    #endif

    #if defined(Serial2)
	  case 2:
        port = &Serial2;
        break;
    #endif

    #if defined(Serial3)
      case 3:
        port = &Serial3;
        break;
    #endif

    case 0:
    default:
      u8serno = 0;
      port = &Serial;
      break;
  }
  port->begin(u32speed, u8config);
  if ( u8txenpin > 1 )  // pin 0 & pin 1 are reserved for RX/TX
  {// return RS485 transceiver to transmit mode
    pinMode(u8txenpin, OUTPUT);
    digitalWrite(u8txenpin, LOW);	// LOW -> RX, HIGH -> TX
  }
  port->flush();	// wait end prev send
  u8lastRec = 0;
  u8BufferSize = 0;
  u16InCnt = 0;
  u16OutCnt = 0;
  u16errCnt = 0;
}
//=====================================
void Modbus::begin() 
{
  begin(19200, SERIAL_8N2);
}
//=====================================
uint16_t Modbus::getInCnt() 
{ 
  return u16InCnt; 
}
//=====================================
uint16_t Modbus::getOutCnt() 
{ 
  return u16OutCnt; 
}
//=====================================
uint16_t Modbus::getErrCnt() 
{ 
  return u16errCnt; 
}
//=====================================
uint8_t Modbus::getState() 
{
  return u8state;
}
//=====================================
uint8_t Modbus::getLastError() 
{
  return u8lastError;
}
//=====================================
// pool slave
//=====================================
int8_t Modbus::poll( uint16_t *regs, uint8_t u8size )
{
uint8_t u8current;
int8_t i8state;
uint8_t u8exception;
//------------------------------
  au16regs = regs;		// point of 16bits registers
  u8regsize = u8size;	// size modbus mem
  // check if there is any incoming frame
  u8current = port->available();  	// number RX data
  if ( u8current == 0 ) return 0;	// no new data rx
  // check T35 after frame end or still no frame end
  if ( u8current != u8lastRec )
  {
    u8lastRec = u8current;
    u32time = millis() + T35;
    return 0;
  }
  if ( millis() < u32time ) return 0;

  u8lastRec = 0;
  i8state = getRxBuffer();  // get data from rxbuf to au8Buffer
  u8lastError = i8state;	  // number get data (temp)
  if ( i8state < MIN_SIZE_FRAME ) return i8state;

  // check slave id
  if ( au8Buffer[UID] != u8id ) return 0;	// not ID slave

  // validate message: CRC, FCT, address and size
  u8exception = validateRequest();
  if ( u8exception > 0 )  // exception!
  {
    if ( u8exception != NO_REPLY ) 
	  {
      buildException(u8exception);
      sendTxBuffer();
    }
    u8lastError = u8exception;
    return u8exception;
  }
  u8lastError = 0;
  // process message
  switch (au8Buffer[FUNC]) 
  {
    case MB_FC_READ_COILS:			    // FCT=1 -> read coils or digital outputs
    case MB_FC_READ_DISCRETE_INPUT:	// FCT=2 -> read digital inputs
      return process_FC1(regs);
      break;

    case MB_FC_READ_REGISTERS:		  // FCT=3 -> read registers or analog outputs (hold registers)
    case MB_FC_READ_INPUT_REGISTER: // FCT=4 -> read analog inputs (input registers)
      return process_FC3(regs);
      break;

    case MB_FC_WRITE_COIL:			    // FCT=5 -> write single coil or output
      return process_FC5(regs);
      break;

    case MB_FC_WRITE_REGISTER:		  // FCT=6 -> write single register
      return process_FC6(regs);
      break;

    case MB_FC_WRITE_MULTIPLE_COILS:  // FCT=15 -> write multiple coils or outputs
      return process_FC15(regs);
      break;

    case MB_FC_WRITE_MULTIPLE_REGISTERS:	// FCT=16 -> write multiple registers
      return process_FC16(regs);
      break;

    default:
      break;
  }
  return i8state;
}
//=====================================
int8_t Modbus::getRxBuffer() 
{
boolean bBuffOverflow = false;
//------------------------------
  if ( u8txenpin > 1 ) digitalWrite(u8txenpin, LOW);	// set TXE in rxmode
  u8BufferSize = 0;
  while ( port->available() )
  {
    au8Buffer[u8BufferSize] = port->read();
    u8BufferSize++;
    if ( u8BufferSize >= MAX_BUFFER ) bBuffOverflow = true;
  }
  u16InCnt++;
  if ( bBuffOverflow ) 
  {
    u16errCnt++;
    return ERR_BUFF_OVERFLOW;
  }
  return u8BufferSize;
}
//=====================================
void Modbus::sendTxBuffer() 
{
  // set RS485 transceiver to transmit mode
  if ( u8txenpin > 1 ) 
  {
    digitalWrite(u8txenpin, HIGH); 	// set TXE in txmode
  }
  // transfer buffer to serial line
  port->write(au8Buffer, u8BufferSize);
  // keep RS485 transceiver in transmit mode as long as sending
  port->flush();
  // return RS485 transceiver to receive mode
  if (u8txenpin > 1) 
  {
    digitalWrite(u8txenpin, LOW);	// set TXE in rxmode
  }
  u8BufferSize = 0;
  // increase message counter
  u16OutCnt++;
}
//=====================================
uint8_t Modbus::validateRequest() 
{
  uint16_t u16regs = 0;
  uint8_t u8regs;
  // check PDU length
  u16regs = word(au8Buffer[LEN_HI], au8Buffer[LEN_LO]);
  u8regs = (uint8_t) u16regs;
  if (u8regs != (u8BufferSize-6)) return EXC_REGS_QUANT;  // 3
  
  // check fct code
  boolean isSupported = false;
  for (uint8_t i = 0; i< sizeof( fctsupported ); i++) 
  {
    if ( fctsupported[i] == au8Buffer[FUNC] ) 
    {
      isSupported = 1;
      break;
    }
  }
  if ( !isSupported ) 
  {
    u16errCnt++;
    return EXC_FUNC_CODE; // 1
  }
  // check start address & nb range
  switch (au8Buffer[FUNC])
  {
    case MB_FC_READ_COILS:                                        // FCT=1 -> read coils or digital outputs
    case MB_FC_READ_DISCRETE_INPUT:                               // FCT=2 -> read digital inputs
    case MB_FC_WRITE_MULTIPLE_COILS:                              // FCT=15 -> write multiple coils or outputs
      u16regs = word(au8Buffer[ADD_HI], au8Buffer[ADD_LO]) / 16;
      u16regs += word(au8Buffer[NB_HI], au8Buffer[NB_LO]) / 16;
      u8regs = (uint8_t) u16regs;
      if (u8regs > u8regsize) return EXC_ADDR_RANGE;  // 2
      break;
 
    case MB_FC_WRITE_COIL:                                        // FCT=5 -> write single coil or output
      u16regs = word(au8Buffer[ADD_HI], au8Buffer[ADD_LO]) / 16;
      u8regs = (uint8_t)u16regs;
      if (u8regs > u8regsize) return EXC_ADDR_RANGE;  // 2
      break;  

    case MB_FC_WRITE_REGISTER:                                    // FCT=6 -> write single register
      u16regs = word(au8Buffer[ADD_HI], au8Buffer[ADD_LO]);
      u8regs = (uint8_t)u16regs;
      if (u8regs > u8regsize) return EXC_ADDR_RANGE;  // 2
      break;

    case MB_FC_READ_REGISTERS:                                    // FCT=3 -> read registers or analog outputs (hold registers)
    case MB_FC_READ_INPUT_REGISTER:                               // FCT=4 -> read analog inputs (input registers)
    case MB_FC_WRITE_MULTIPLE_REGISTERS:                          // FCT=16 -> write multiple registers
      u16regs = word(au8Buffer[ADD_HI], au8Buffer[ADD_LO]);
      u16regs += word(au8Buffer[NB_HI], au8Buffer[NB_LO]);
      u8regs = (uint8_t) u16regs;
      if (u8regs > u8regsize) return EXC_ADDR_RANGE;  // 2
      break;
  }
  return 0; // OK, no exception code thrown
}
//=====================================
void Modbus::buildException( uint8_t u8exception ) 
{
  uint8_t u8func = au8Buffer[FUNC];  // get the original FUNC code
  au8Buffer[LEN_HI] = 0;
  au8Buffer[LEN_LO] = 3;
  au8Buffer[UID] = u8id;
  au8Buffer[FUNC] = u8func + 0x80;
  au8Buffer[ADD_HI] = u8exception;
  u8BufferSize = 9;
}
//=====================================
// FCT=01 Read Out Discret or FCT=02 Read In Discret
// REQ = [MBAP] + [ {FUNC = 01 or 02} + {ADDR_HI} + {ADDR_LO} + {NB_HI} + {NB_LO} ]
// ACK = [MBAP] + [ {FUNC = 01 or 02} + {LEN_DATA_BYTES} + {DATA_ADDR0} + ... + {DATA_ADDR(CNT/8)} ]         // DATA_ADDR0 = {7,6,5,4,3,2,1,0}; DATA_ADDR1 = {15,14,13,12,11,10,9,8}; 
int8_t Modbus::process_FC1(uint16_t *regs)
{
uint8_t u8currentRegister;  // адрес текущего регистра
uint8_t u8currentBit;       // номер текущего бита
uint8_t u8bytesno;          // кол-во байт для данных
uint8_t u8bitsno;           // счетчик битов
uint8_t u8CopyBufferSize;   // кол-во данных в буфере
uint16_t u16currentCoil;    // счетчик битов
uint16_t u16coil;           // адрес бита
// get the first and last coil from the message
uint16_t u16StartCoil = word(au8Buffer[ADD_HI], au8Buffer[ADD_LO]); // стартовый адрес бита
uint16_t u16Coilno = word(au8Buffer[NB_HI], au8Buffer[NB_LO]); // кол-во битов
//------------------------------
  u8bytesno = (uint8_t)(u16Coilno / 8); // считаем количество байт 
  if ( (u16Coilno % 8) != 0) u8bytesno++; // добавляем 1 если кол-во бит не кратно 8
  au8Buffer[LEN_LO] = u8bytesno + 3;  // общая длина PDU
  au8Buffer[ADD_HI] = u8bytesno;  // кол-во байт данных
  u8BufferSize = 9; // размер уже записаных данных: заголовок(7) + func(1) + lenb(1)
  u8bitsno = 0; // текущий бит данных в буфере
  for (u16currentCoil = 0; u16currentCoil < u16Coilno; u16currentCoil++)   // читаем каждый бит из регистров и записываем в буфер
  {
    u16coil = u16StartCoil + u16currentCoil;  // адрес бита
    u8currentRegister = (uint8_t)(u16coil / 16);  // адрес регистра с текущем битом
    u8currentBit = (uint8_t)(u16coil % 16);       // номер текущего бита в регистре
    bitWrite(au8Buffer[u8BufferSize], u8bitsno, bitRead(regs[u8currentRegister], u8currentBit));  // читаем текущий бит из текущего регистра и записаваем в текущий бит текущих данных буфера
    u8bitsno++;
    if (u8bitsno > 7) // переходим к следующему байту
    {
      u8bitsno = 0;
      u8BufferSize++;
    }
  }
  // send outcoming message
  if ((u16Coilno % 8) != 0) u8BufferSize++; // кол-во бит не кратно 8, байт не закончен, увеличиваем кол-во на 1
  u8CopyBufferSize = u8BufferSize; // кол-во данных в буфере
  sendTxBuffer(); // отправляем буфер
  return u8CopyBufferSize;
}
//=====================================
// FCT=03 Read Out Registers or FCT=04  Read In Registers
// REQ = [MBAP] + [ {FUNC = 03 or 04} + {ADDR_HI} + {ADDR_LO} + {NB_HI} + {NB_HI} ]
// ACK = [MBAP] + [ {FUNC = 03 or 04} + {LEN_DATA_BYTES} + {DATA_ADDR0_HI} + {DATA_ADDR0_LO} + ... + {DATA_ADDR(CNT)_HI} + {DATA_ADDR(CNT)_LO} ] // LEN - Length of DATA in Bytes;
int8_t Modbus::process_FC3( uint16_t *regs)
{
uint8_t u8StartAdd = word(au8Buffer[ADD_HI], au8Buffer[ADD_LO]);  // стартовый адрес регистров
uint8_t u8regsno = word(au8Buffer[NB_HI], au8Buffer[NB_LO]);  // кол-во регистров
uint8_t u8CopyBufferSize; // кол-во данных в буфере
uint8_t i;  
//------------------------------
  au8Buffer[LEN_LO] = u8regsno * 2 + 3;  // общая длина PDU
  au8Buffer[ADD_HI] = u8regsno * 2;  // кол-во байт данных
  u8BufferSize = 9; // размер уже записаных данных: заголовок(7) + func(1) + lenb(1)
  for (i = u8StartAdd; i < u8StartAdd + u8regsno; i++) // читаем регистры и сохраняем в буфер
  {
    au8Buffer[u8BufferSize] = highByte(regs[i]);
    u8BufferSize++;
    au8Buffer[u8BufferSize] = lowByte(regs[i]);
    u8BufferSize++;
  }
  u8CopyBufferSize = u8BufferSize;  // кол-во данных в буфере
  sendTxBuffer(); // отправляем буфер
  return u8CopyBufferSize;
}
//=====================================
// FCT=05 Write Single Discret
// REQ = [MBAP] + [ {FUNC = 05} + {ADDR_HI} + {ADDR_LO} + {DATA_HI} + {DATA_LO} ]         // DATA = FF 00 for ON; 00 00 for OFF
// ACK = REQ = [MBAP] + [ {FUNC = 05} + {ADDR_HI} + {ADDR_LO} + {DATA_HI} + {DATA_LO} ]
int8_t Modbus::process_FC5( uint16_t *regs)
{
uint8_t u8currentRegister;  // адрес текущего регистра
uint8_t u8currentBit;       // номер текущего бита
uint8_t u8CopyBufferSize;   // кол-во данных в буфере
uint16_t u16coil = word(au8Buffer[ADD_HI], au8Buffer[ADD_LO]); // адрес бита
//------------------------------
  // point to the register and its bit
  u8currentRegister = (uint8_t)(u16coil / 16);  // адрес текущего регистра
  u8currentBit = (uint8_t)(u16coil % 16); // номер текущего бита
  // write to coil
  bitWrite(regs[u8currentRegister], u8currentBit, au8Buffer[NB_HI] == 0xff );
  // send answer to master
  u8BufferSize = 12;  // возвращаем пакет обратно
  u8CopyBufferSize = u8BufferSize;  // кол-во данных в буфере
  sendTxBuffer(); // отправляем буфер
  return u8CopyBufferSize;
}
//=====================================
// FCT=06 Write Single Register
// REQ = [MBAP] + [ {FUNC = 06} + {ADDR_HI} + {ADDR_LO} + {DATA_HI} + {DATA_LO} ]         
// ACK = REQ = [MBAP] + [ {FUNC = 06} + {ADDR_HI} + {ADDR_LO} + {DATA_HI} + {DATA_LO} ]
int8_t Modbus::process_FC6(uint16_t *regs)
{
uint8_t u8add = word(au8Buffer[ADD_HI], au8Buffer[ADD_LO]); // адрес регистра
uint8_t u8CopyBufferSize; // кол-во данных в буфере
uint16_t u16val = word(au8Buffer[NB_HI], au8Buffer[NB_LO]); // значение регистра
//------------------------------
  regs[u8add] = u16val; // сохраняем значение регистра
  // keep the same header
  u8BufferSize = 12;  // возвращаем пакет обратно
  u8CopyBufferSize = u8BufferSize;  // кол-во данных в буфере
  sendTxBuffer(); // отправляем буфер
  return u8CopyBufferSize;
}
//=====================================
// FCT=0F Write multiple Out Discret
// REQ = [MBAP] + [ {FUNC = 0F} + {ADDR_HI} + {ADDR_LO} + {NB_HI} + {NB_LO} + {LEN_DATA_BYTES} + {DATA_ADDR0} + ... + {DATA_ADDR(CNT/8)} ]
// ACK = [MBAP] + [ {FUNC = 0F} + {ADDR_HI} + {ADDR_LO} + {NB_HI} + {NB_LO} ]
int8_t Modbus::process_FC15(uint16_t *regs)
{
uint8_t u8currentRegister;  // адрес текущего регистра
uint8_t u8currentBit;       // номер текущего бита
uint8_t u8frameByte;        // номер текущих данных буфера
uint8_t u8bitsno;           // счетчик битов
uint8_t u8CopyBufferSize;   // кол-во данных в буфере
uint16_t u16currentCoil;    // счетчик битов
uint16_t u16coil;           // адрес бита
boolean bTemp;              // значение бита
uint16_t u16StartCoil = word(au8Buffer[ADD_HI], au8Buffer[ADD_LO]); // стартовый адрес бита
uint16_t u16Coilno = word(au8Buffer[NB_HI], au8Buffer[NB_LO]);  // кол-во битов
//------------------------------
  // get the first and last coil from the message
  // read each coil from the register map and put its value inside the outcoming message
  u8bitsno = 0; // счетчик битов из буфера
  u8frameByte = 13; // номер текущих данных буфера
  for ( u16currentCoil = 0; u16currentCoil < u16Coilno; u16currentCoil++ ) // читаем текущий бит из текущих данных буфера и записаваем в текущий бит текущего регистра
  {
    u16coil = u16StartCoil + u16currentCoil;  // адрес бита
    u8currentRegister = (uint8_t)(u16coil / 16);  // адрес текущего регистра
    u8currentBit = (uint8_t)(u16coil % 16); // номер текущего бита
    bTemp = bitRead(au8Buffer[u8frameByte], u8bitsno);  // читаем значение бита из текущих данных буфера
    bitWrite(regs[u8currentRegister], u8currentBit, bTemp); // записаваем в текущий бит текущего регистра
    u8bitsno++;
    if (u8bitsno > 7) // переходим к следующему байту
    {
      u8bitsno = 0;
      u8frameByte++;
    }
  }
  // send outcoming message
  au8Buffer[LEN_LO] = 6;  // общая длина PDU
  // it's just a copy of the incomping frame until 6th byte
  u8BufferSize = 12;
  u8CopyBufferSize = u8BufferSize;  // кол-во данных в буфере
  sendTxBuffer(); // отправляем буфер
  return u8CopyBufferSize;
}
//=====================================
// FCT=10 Write multiple Out Registers
// REQ = [MBAP] + [ {FUNC = 10} + {ADDR_HI} + {ADDR_LO} + {CNT_REGS_HI} + {CNT_REGS_LO} + {BYTE_CNT} + {DATA_ADDR0_HI} + {DATA_ADDR0_LO} + ... + {DATA_ADDR(CNT)_HI} + {DATA_ADDR(CNT)_LO} ]
// ACK = [MBAP] + [ {FUNC = 10} + {ADDR_HI} + {ADDR_LO} + {CNT_REGS_HI} + {CNT_REGS_LO} ]
int8_t Modbus::process_FC16(uint16_t *regs)
{
uint8_t u8StartAdd = word(au8Buffer[ADD_HI], au8Buffer[ADD_LO]);  // стартовый адрес регистров
uint8_t u8regsno = word(au8Buffer[NB_HI], au8Buffer[NB_LO]);  // кол-во регистров
uint8_t u8CopyBufferSize; // кол-во данных в буфере
uint8_t i;
uint16_t temp;
//------------------------------
  for (i = 0; i < u8regsno; i++ ) 
  {
    temp = word(au8Buffer[(BYTE_CNT+1)+i*2], au8Buffer[(BYTE_CNT+2)+i*2]);  // читаем данные из буфера
    regs[u8StartAdd+i] = temp;  // сохраняем в регистре
  }
  au8Buffer[LEN_LO] = 6;  // общая длина PDU
  u8BufferSize = 12;
  u8CopyBufferSize = u8BufferSize;  // кол-во данных в буфере
  sendTxBuffer(); // отправляем буфер
  return u8CopyBufferSize;
}
//=====================================
// END
