#pragma once
#include <stdint.h>
#include <string>
#include <sstream>
#include <boost/shared_ptr.hpp>
#if defined(_WIN32)
#include <windows.h>
#else
#include <termios.h>
#endif
#include "modbus-version.h"
#include "modbus-rtu-private.h"

#if defined(_WIN32)
#  include <winsock2.h>
#else
#  include <arpa/inet.h>
#endif

#ifndef FALSE
#define FALSE 0
#endif

#ifndef TRUE
#define TRUE 1
#endif

#ifndef OFF
#define OFF 0
#endif

#ifndef ON
#define ON 1
#endif
typedef int ssize_t;


class MBConnection {
public:
    virtual int connect() = 0;
    virtual void close() = 0;
    virtual bool flush() = 0;
    virtual int send(uint8_t *msg, int msg_length) = 0;
    virtual int select(fd_set * rset, timeval * tv, int length_to_read)=0;
    virtual ssize_t recv(uint8_t * rsp, int rsp_length)=0;
    virtual bool read_char(uint8_t & c, timeval * tv = NULL)=0;
    virtual void purge() = 0;
    virtual bool isSerial() = 0;
    virtual bool isDebug() = 0;
	virtual bool isErrorState() = 0;

};
typedef boost::shared_ptr<MBConnection> MBConnectionPtr;



/* Modbus function codes */
#define MODBUS_FC_READ_COILS                0x01
#define MODBUS_FC_READ_DISCRETE_INPUTS      0x02
#define MODBUS_FC_READ_HOLDING_REGISTERS    0x03
#define MODBUS_FC_READ_INPUT_REGISTERS      0x04
#define MODBUS_FC_WRITE_SINGLE_COIL         0x05
#define MODBUS_FC_WRITE_SINGLE_REGISTER     0x06
#define MODBUS_FC_READ_EXCEPTION_STATUS     0x07
#define MODBUS_FC_WRITE_MULTIPLE_COILS      0x0F
#define MODBUS_FC_WRITE_MULTIPLE_REGISTERS  0x10
#define MODBUS_FC_REPORT_SLAVE_ID           0x11
#define MODBUS_FC_MASK_WRITE_REGISTER       0x16
#define MODBUS_FC_WRITE_AND_READ_REGISTERS  0x17

#define MODBUS_BROADCAST_ADDRESS    0

/* Modbus_Application_Protocol_V1_1b.pdf (chapter 6 section 1 page 12)
* Quantity of Coils to read (2 bytes): 1 to 2000 (0x7D0)
* (chapter 6 section 11 page 29)
* Quantity of Coils to write (2 bytes): 1 to 1968 (0x7B0)
*/
#define MODBUS_MAX_READ_BITS              2000
#define MODBUS_MAX_WRITE_BITS             1968

/* Modbus_Application_Protocol_V1_1b.pdf (chapter 6 section 3 page 15)
* Quantity of Registers to read (2 bytes): 1 to 125 (0x7D)
* (chapter 6 section 12 page 31)
* Quantity of Registers to write (2 bytes) 1 to 123 (0x7B)
* (chapter 6 section 17 page 38)
* Quantity of Registers to write in R/W registers (2 bytes) 1 to 121 (0x79)
*/
#define MODBUS_MAX_READ_REGISTERS          125
#define MODBUS_MAX_WRITE_REGISTERS         123
#define MODBUS_MAX_WR_WRITE_REGISTERS      121
#define MODBUS_MAX_WR_READ_REGISTERS       125

/* The size of the MODBUS PDU is limited by the size constraint inherited from
* the first MODBUS implementation on Serial Line network (max. RS485 ADU = 256
* bytes). Therefore, MODBUS PDU for serial line communication = 256 - Server
* address (1 byte) - CRC (2 bytes) = 253 bytes.
*/
#define MODBUS_MAX_PDU_LENGTH              253

/* Consequently:
* - RTU MODBUS ADU = 253 bytes + Server address (1 byte) + CRC (2 bytes) = 256
*   bytes.
* - TCP MODBUS ADU = 253 bytes + MBAP (7 bytes) = 260 bytes.
* so the maximum of both backend in 260 bytes. This size can used to allocate
* an array of bytes to store responses and it will be compatible with the two
* backends.
*/
#define MODBUS_MAX_ADU_LENGTH              260

/* Random number to avoid errno conflicts */
#define MODBUS_ENOBASE 112345678

/* Protocol exceptions */
enum {
	MODBUS_EXCEPTION_ILLEGAL_FUNCTION = 0x01,
	MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS,
	MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE,
	MODBUS_EXCEPTION_SLAVE_OR_SERVER_FAILURE,
	MODBUS_EXCEPTION_ACKNOWLEDGE,
	MODBUS_EXCEPTION_SLAVE_OR_SERVER_BUSY,
	MODBUS_EXCEPTION_NEGATIVE_ACKNOWLEDGE,
	MODBUS_EXCEPTION_MEMORY_PARITY,
	MODBUS_EXCEPTION_NOT_DEFINED,
	MODBUS_EXCEPTION_GATEWAY_PATH,
	MODBUS_EXCEPTION_GATEWAY_TARGET,
	MODBUS_EXCEPTION_MAX
};

#define EMBXILFUN  (MODBUS_ENOBASE + MODBUS_EXCEPTION_ILLEGAL_FUNCTION)
#define EMBXILADD  (MODBUS_ENOBASE + MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS)
#define EMBXILVAL  (MODBUS_ENOBASE + MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE)
#define EMBXSFAIL  (MODBUS_ENOBASE + MODBUS_EXCEPTION_SLAVE_OR_SERVER_FAILURE)
#define EMBXACK    (MODBUS_ENOBASE + MODBUS_EXCEPTION_ACKNOWLEDGE)
#define EMBXSBUSY  (MODBUS_ENOBASE + MODBUS_EXCEPTION_SLAVE_OR_SERVER_BUSY)
#define EMBXNACK   (MODBUS_ENOBASE + MODBUS_EXCEPTION_NEGATIVE_ACKNOWLEDGE)
#define EMBXMEMPAR (MODBUS_ENOBASE + MODBUS_EXCEPTION_MEMORY_PARITY)
#define EMBXGPATH  (MODBUS_ENOBASE + MODBUS_EXCEPTION_GATEWAY_PATH)
#define EMBXGTAR   (MODBUS_ENOBASE + MODBUS_EXCEPTION_GATEWAY_TARGET)

/* Native libmodbus error codes */
#define EMBBADCRC  (EMBXGTAR + 1)
#define EMBBADDATA (EMBXGTAR + 2)
#define EMBBADEXC  (EMBXGTAR + 3)
#define EMBUNKEXC  (EMBXGTAR + 4)
#define EMBMDATA   (EMBXGTAR + 5)
#define EMBBADSLAVE (EMBXGTAR + 6)

extern const unsigned int libmodbus_version_major;
extern const unsigned int libmodbus_version_minor;
extern const unsigned int libmodbus_version_micro;

typedef struct _modbus_mapping_t {
	int nb_bits;
	int start_bits;
	int nb_input_bits;
	int start_input_bits;
	int nb_input_registers;
	int start_input_registers;
	int nb_registers;
	int start_registers;
	uint8_t *tab_bits;
	uint8_t *tab_input_bits;
	uint16_t *tab_input_registers;
	uint16_t *tab_registers;
} modbus_mapping_t;

typedef enum
{
	MODBUS_ERROR_RECOVERY_NONE = 0,
	MODBUS_ERROR_RECOVERY_LINK = (1 << 1),
	MODBUS_ERROR_RECOVERY_PROTOCOL = (1 << 2)
} modbus_error_recovery_mode;

typedef struct _sft {
	int slave;
	int function;
	int t_id;
} sft_t;

#define _MIN_REQ_LENGTH 12

#define _REPORT_SLAVE_ID 180

#define _MODBUS_EXCEPTION_RSP_LENGTH 5

/* Timeouts in microsecond (0.5 s) */
#define _RESPONSE_TIMEOUT    500000
#define _BYTE_TIMEOUT        500000

/*
*  ---------- Request     Indication ----------
*  | Client | ---------------------->| Server |
*  ---------- Confirmation  Response ----------
*/
typedef enum {
	/* Request message on the server side */
	MSG_INDICATION,
	/* Request message on the client side */
	MSG_CONFIRMATION
} msg_type_t;


void modbus_set_bits_from_byte(uint8_t *dest, int idx, const uint8_t value);
void modbus_set_bits_from_bytes(uint8_t *dest, int idx, unsigned int nb_bits, const uint8_t *tab_byte);
uint8_t modbus_get_byte_from_bits(const uint8_t *src, int idx, unsigned int nb_bits);
float modbus_get_float(const uint16_t *src);
float modbus_get_float_abcd(const uint16_t *src);
float modbus_get_float_dcba(const uint16_t *src);
float modbus_get_float_badc(const uint16_t *src);
float modbus_get_float_cdab(const uint16_t *src);

void modbus_set_float(float f, uint16_t *dest);
void modbus_set_float_abcd(float f, uint16_t *dest);
void modbus_set_float_dcba(float f, uint16_t *dest);
void modbus_set_float_badc(float f, uint16_t *dest);
void modbus_set_float_cdab(float f, uint16_t *dest);



void win32_ser_init(struct win32_ser *ws);

int win32_ser_select(struct win32_ser *ws, int max_len,
	const struct timeval *tv);

int win32_ser_read(struct win32_ser *ws, uint8_t *p_msg,
	unsigned int max_len);

int serial_connect(const std::string& device, const int baud, const uint8_t data_bit, const uint8_t stop_bit, const char parity, win32_ser *w_ser, DCB *old_dcb, std::ostringstream &err);


uint16_t crc16(uint8_t *buffer, uint16_t buffer_length);
