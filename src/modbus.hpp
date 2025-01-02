#pragma once
#ifndef _MSC_VER
# include <stdint.h>
# include <sys/time.h>
#else
# include "stdint.h"
# include <time.h>
#endif
#include <sys/types.h>
#include <config.h>

#if defined(_WIN32)
#  include <winsock2.h>
#else
#  include <arpa/inet.h>
#endif
#include "util.hpp"


class Modbus {

protected:
	Modbus(int slave, int header_length, int checksum_length, int max_adu_length);


public:
	int modbus_get_slave();
	int modbus_set_error_recovery( modbus_error_recovery_mode error_recovery);
	int modbus_set_socket( int s);
	int modbus_get_socket();

	int modbus_get_response_timeout( uint32_t *to_sec, uint32_t *to_usec);
	int modbus_set_response_timeout( uint32_t to_sec, uint32_t to_usec);

	int modbus_get_byte_timeout( uint32_t *to_sec, uint32_t *to_usec);
	int modbus_set_byte_timeout( uint32_t to_sec, uint32_t to_usec);

	int modbus_get_indication_timeout( uint32_t *to_sec, uint32_t *to_usec);
	int modbus_set_indication_timeout( uint32_t to_sec, uint32_t to_usec);

	int modbus_get_header_length();

	int modbus_set_debug( int flag);

	const char *modbus_strerror(int errnum);

	int modbus_read_bits( int addr, int nb, uint8_t *dest);
	int modbus_read_input_bits( int addr, int nb, uint8_t *dest);
	int modbus_read_registers( int addr, int nb, uint16_t *dest);
	int modbus_read_input_registers( int addr, int nb, uint16_t *dest);
	int modbus_write_bit( int coil_addr, int status);
	int modbus_write_register( int reg_addr, const uint16_t value);
	int modbus_write_bits( int addr, int nb, const uint8_t *data);
	int modbus_write_registers( int addr, int nb, const uint16_t *data);
	int modbus_mask_write_register( int addr, uint16_t and_mask, uint16_t or_mask);
	int modbus_write_and_read_registers( int write_addr, int write_nb,
		const uint16_t *src, int read_addr, int read_nb,
		uint16_t *dest);
	int modbus_report_slave_id( int max_dest, uint8_t *dest);

	modbus_mapping_t* modbus_mapping_new_start_address(
		unsigned int start_bits, unsigned int nb_bits,
		unsigned int start_input_bits, unsigned int nb_input_bits,
		unsigned int start_registers, unsigned int nb_registers,
		unsigned int start_input_registers, unsigned int nb_input_registers);

	modbus_mapping_t* modbus_mapping_new(int nb_bits, int nb_input_bits,
		int nb_registers, int nb_input_registers);
	void modbus_mapping_free(modbus_mapping_t *mb_mapping);

	int modbus_send_raw_request( const uint8_t *raw_req, int raw_req_length);


	int modbus_receive( uint8_t *req);

	int modbus_receive_confirmation( uint8_t *rsp);

	int modbus_reply( const uint8_t *req,
		int req_length, modbus_mapping_t *mb_mapping);
	int modbus_reply_exception( const uint8_t *req,
		unsigned int exception_code);

	virtual int connect() = 0;
	virtual void close() = 0;
	virtual void free() = 0;
	virtual bool flush() = 0;



protected:
	void _error_print(const char *context);
	void _sleep_response_timeout();
	unsigned int compute_response_length_from_request(uint8_t *req);
	int send_msg(uint8_t *msg, int msg_length);
	uint8_t compute_meta_length_after_function(int function, msg_type_t msg_type);
	int compute_data_length_after_meta(uint8_t *msg, msg_type_t msg_type);
	int _modbus_receive_msg(uint8_t *msg, msg_type_t msg_type);
	int check_confirmation(uint8_t *req, uint8_t *rsp, int rsp_length);
	int response_io_status(uint8_t *tab_io_status, int address, int nb, uint8_t *rsp, int offset);
	int response_exception(sft_t *sft, int exception_code, uint8_t *rsp, unsigned int to_flush, const char* template_, ...);
	int read_io_status(int function, int addr, int nb, uint8_t *dest);
	int read_registers(int function, int addr, int nb, uint16_t *dest);
	int write_single(int function, int addr, const uint16_t value);

	virtual uint8_t get_next_read_size(uint8_t * msg, uint8_t msg_length, msg_type_t msg_type);


	virtual int build_response_basis(sft_t *sft, uint8_t *rsp) = 0;
	virtual int send_msg_pre(uint8_t *msg, int msg_length) = 0;
	virtual int send(uint8_t *msg, int msg_length) = 0;
	virtual int select(fd_set *rset, struct timeval *tv, int length_to_read) = 0;
	virtual ssize_t recv(uint8_t *rsp, int rsp_length) = 0;
	virtual int check_integrity(uint8_t *msg, const int msg_length) = 0;
	virtual int receive(uint8_t *req) = 0;
	virtual int pre_check_confirmation(const uint8_t *req, const uint8_t *rsp, int rsp_length) = 0;
	virtual int prepare_response_tid(const uint8_t *req, int *req_length) = 0;
	virtual int build_request_basis(int function, int addr, int nb, uint8_t *req) = 0;
	


public:
	/* Slave address */
	const uint8_t slave;
	/* Socket or file descriptor */
	int s;
	const unsigned short header_length;
	const unsigned short checksum_length;
	const unsigned short max_adu_length;
	int debug;
    bool trace;
	int error_recovery;
	struct timeval response_timeout;
	struct timeval byte_timeout;
	struct timeval indication_timeout;
	void *backend_data;




};

