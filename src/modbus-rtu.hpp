#pragma once
#include "modbus.hpp"
#include <string>
#include "modbus-rtu-private.h"
#include "util.hpp"


class ModbusRTU : public Modbus {

public:
	ModbusRTU(int slave, MBConnectionPtr connection);

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

    MBConnectionPtr connection;


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