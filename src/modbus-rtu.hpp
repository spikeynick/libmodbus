#pragma once
#include "modbus.hpp"
#include <string>
#include "modbus-rtu-private.h"
#include "util.hpp"


class ModbusRTU : public Modbus {

public:
	ModbusRTU(int slave, const std::string &device,
		int baud, char parity, int data_bit,
		int stop_bit);

	virtual int connect();
	virtual void close();
	virtual void free();
	virtual bool flush();

protected:

	virtual int build_response_basis(sft_t *sft, uint8_t *rsp);
	virtual int send_msg_pre(uint8_t *msg, int msg_length);
	virtual int send(uint8_t *msg, int msg_length);
	virtual int select(fd_set *rset, struct timeval *tv, int length_to_read);
	virtual ssize_t recv(uint8_t *rsp, int rsp_length);
	virtual int check_integrity(uint8_t *msg, const int msg_length);
	virtual int receive(uint8_t *req);
	virtual int pre_check_confirmation(const uint8_t *req, const uint8_t *rsp, int rsp_length);
	virtual int prepare_response_tid(const uint8_t *req, int *req_length);
	virtual int build_request_basis(int function, int addr, int nb, uint8_t *req);


	/* Device: "/dev/ttyS0", "/dev/ttyUSB0" or "/dev/tty.USA19*" on Mac OS X. */
	const std::string device;
	/* Bauds: 9600, 19200, 57600, 115200, etc */
	const int baud;
	/* Data bit */
	const uint8_t data_bit;
	/* Stop bit */
	const uint8_t stop_bit;
	/* Parity: 'N', 'O', 'E' */
	const char parity;
#if defined(_WIN32)
	struct win32_ser w_ser;
	DCB old_dcb;
#else
	/* Save old termios settings */
	struct termios old_tios;
#endif
#if HAVE_DECL_TIOCSRS485
	int serial_mode;
#endif
#if HAVE_DECL_TIOCM_RTS
	int rts;
	int rts_delay;
	int onebyte_time;
	void(*set_rts) (modbus_t *ctx, int on);
#endif
	/* To handle many slaves on the same link */
	int confirmation_to_ignore;
};