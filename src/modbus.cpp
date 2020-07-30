/*
* Copyright � 2001-2011 St�phane Raimbault <stephane.raimbault@gmail.com>
*
* SPDX-License-Identifier: LGPL-2.1-or-later
*
* This library implements the Modbus protocol.
* http://libmodbus.org/
*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#include <errno.h>
#include <limits.h>
#include <time.h>
#ifndef _MSC_VER
#include <unistd.h>
#endif

#include <config.h>

//#include "modbus-private.h"


#include "modbus.hpp"


Modbus::Modbus(int slave, int header_length, int checksum_length, int max_adu_length) :
	slave(slave),
	s(-1),
	debug(FALSE),
	error_recovery(MODBUS_ERROR_RECOVERY_NONE),
	header_length(header_length),
	checksum_length(checksum_length),
	max_adu_length(max_adu_length)
{
	this->response_timeout.tv_sec = 0;
	this->response_timeout.tv_usec = _RESPONSE_TIMEOUT;

	this->byte_timeout.tv_sec = 0;
	this->byte_timeout.tv_usec = _BYTE_TIMEOUT;

	this->indication_timeout.tv_sec = 0;
	this->indication_timeout.tv_usec = 0;

}



/* Internal use */
#define MSG_LENGTH_UNDEFINED -1

/* Exported version */
const unsigned int libmodbus_version_major = LIBMODBUS_VERSION_MAJOR;
const unsigned int libmodbus_version_minor = LIBMODBUS_VERSION_MINOR;
const unsigned int libmodbus_version_micro = LIBMODBUS_VERSION_MICRO;

/* Max between RTU and TCP max adu length (so TCP) */
#define MAX_MESSAGE_LENGTH 260

/* 3 steps are used to parse the query */
typedef enum {
	_STEP_FUNCTION,
	_STEP_META,
	_STEP_DATA
} _step_t;

const char *Modbus::modbus_strerror(int errnum) {
	switch (errnum) {
	case EMBXILFUN:
		return "Illegal function";
	case EMBXILADD:
		return "Illegal data address";
	case EMBXILVAL:
		return "Illegal data value";
	case EMBXSFAIL:
		return "Slave device or server failure";
	case EMBXACK:
		return "Acknowledge";
	case EMBXSBUSY:
		return "Slave device or server is busy";
	case EMBXNACK:
		return "Negative acknowledge";
	case EMBXMEMPAR:
		return "Memory parity error";
	case EMBXGPATH:
		return "Gateway path unavailable";
	case EMBXGTAR:
		return "Target device failed to respond";
	case EMBBADCRC:
		return "Invalid CRC";
	case EMBBADDATA:
		return "Invalid data";
	case EMBBADEXC:
		return "Invalid exception code";
	case EMBMDATA:
		return "Too many data";
	case EMBBADSLAVE:
		return "Response not from requested slave";
	default:
		return strerror(errnum);
	}
}

void Modbus::_error_print( const char *context)
{
	if (this->debug) {
		fprintf(stderr, "ERROR %s", modbus_strerror(errno));
		if (context != NULL) {
			fprintf(stderr, ": %s\n", context);
		}
		else {
			fprintf(stderr, "\n");
		}
	}
}

void Modbus::_sleep_response_timeout()
{
	/* Response timeout is always positive */
#ifdef _WIN32
	/* usleep doesn't exist on Windows */
	Sleep((this->response_timeout.tv_sec * 1000) +
		(this->response_timeout.tv_usec / 1000));
#else
	/* usleep source code */
	struct timespec request, remaining;
	request.tv_sec = this->response_timeout.tv_sec;
	request.tv_nsec = ((long int)this->response_timeout.tv_usec) * 1000;
	while (nanosleep(&request, &remaining) == -1 && errno == EINTR) {
		request = remaining;
	}
#endif
}


/* Computes the length of the expected response */
 unsigned int Modbus::compute_response_length_from_request(uint8_t *req)
{
	int length;
	const int offset = this->header_length;

	switch (req[offset]) {
	case MODBUS_FC_READ_COILS:
	case MODBUS_FC_READ_DISCRETE_INPUTS: {
		/* Header + nb values (code from write_bits) */
		int nb = (req[offset + 3] << 8) | req[offset + 4];
		length = 2 + (nb / 8) + ((nb % 8) ? 1 : 0);
	}
										 break;
	case MODBUS_FC_WRITE_AND_READ_REGISTERS:
	case MODBUS_FC_READ_HOLDING_REGISTERS:
	case MODBUS_FC_READ_INPUT_REGISTERS:
		/* Header + 2 * nb values */
		length = 2 + 2 * (req[offset + 3] << 8 | req[offset + 4]);
		break;
	case MODBUS_FC_READ_EXCEPTION_STATUS:
		length = 3;
		break;
	case MODBUS_FC_REPORT_SLAVE_ID:
		/* The response is device specific (the header provides the
		length) */
		return MSG_LENGTH_UNDEFINED;
	case MODBUS_FC_MASK_WRITE_REGISTER:
		length = 7;
		break;
	default:
		length = 5;
	}

	return offset + length + this->checksum_length;
}

/* Sends a request/response */
int Modbus::send_msg(uint8_t *msg, int msg_length)
{
	int rc;
	int i;

	msg_length = this->send_msg_pre(msg, msg_length);

	if (this->debug) {
		for (i = 0; i < msg_length; i++)
			printf("[%.2X]", msg[i]);
		printf("\n");
	}

	/* In recovery mode, the write command will be issued until to be
	successful! Disabled by default. */
	do {
		rc = this->send(msg, msg_length);
		if (rc == -1) {
			_error_print(NULL);
			if (this->error_recovery & MODBUS_ERROR_RECOVERY_LINK) {
				int saved_errno = errno;

				if ((errno == EBADF || errno == ECONNRESET || errno == EPIPE)) {
					close();
					_sleep_response_timeout();
					connect();
				}
				else {
					_sleep_response_timeout();
					flush();
				}
				errno = saved_errno;
			}
		}
	} while ((this->error_recovery & MODBUS_ERROR_RECOVERY_LINK) &&
		rc == -1);

	if (rc > 0 && rc != msg_length) {
		errno = EMBBADDATA;
		return -1;
	}

	return rc;
}

int Modbus::modbus_send_raw_request(const uint8_t *raw_req, int raw_req_length)
{
	sft_t sft;
	uint8_t req[MAX_MESSAGE_LENGTH];
	int req_length;

	if (raw_req_length < 2 || raw_req_length >(MODBUS_MAX_PDU_LENGTH + 1)) {
		/* The raw request must contain function and slave at least and
		must not be longer than the maximum pdu length plus the slave
		address. */
		errno = EINVAL;
		return -1;
	}

	sft.slave = raw_req[0];
	sft.function = raw_req[1];
	/* The t_id is left to zero */
	sft.t_id = 0;
	/* This response function only set the header so it's convenient here */
	req_length = this->build_response_basis(&sft, req);

	if (raw_req_length > 2) {
		/* Copy data after function code */
		memcpy(req + req_length, raw_req + 2, raw_req_length - 2);
		req_length += raw_req_length - 2;
	}

	return send_msg(req, req_length);
}



uint8_t Modbus::get_next_read_size(uint8_t *msg, uint8_t msg_length, msg_type_t msg_type) {


	int length = this->header_length + 1;
	uint8_t function = msg[this->header_length];
	
	length += compute_meta_length_after_function(function, msg_type);
	
	if (msg_length == this->header_length + 1) {
		if (length > 0) {
			return length - msg_length;
		}
	}
	length += compute_data_length_after_meta(msg, msg_type);


	return length - msg_length;
}

/*
*  ---------- Request     Indication ----------
*  | Client | ---------------------->| Server |
*  ---------- Confirmation  Response ----------
*/

/* Computes the length to read after the function received */
uint8_t Modbus::compute_meta_length_after_function(int function, msg_type_t msg_type)
{
	int length;

	if (msg_type == MSG_INDICATION) {
		if (function <= MODBUS_FC_WRITE_SINGLE_REGISTER) {
			length = 4;
		}
		else if (function == MODBUS_FC_WRITE_MULTIPLE_COILS ||
			function == MODBUS_FC_WRITE_MULTIPLE_REGISTERS) {
			length = 5;
		}
		else if (function == MODBUS_FC_MASK_WRITE_REGISTER) {
			length = 6;
		}
		else if (function == MODBUS_FC_WRITE_AND_READ_REGISTERS) {
			length = 9;
		}
		else {
			/* MODBUS_FC_READ_EXCEPTION_STATUS, MODBUS_FC_REPORT_SLAVE_ID */
			length = 0;
		}
	}
	else {
		/* MSG_CONFIRMATION */
		switch (function) {
		case MODBUS_FC_WRITE_SINGLE_COIL:
		case MODBUS_FC_WRITE_SINGLE_REGISTER:
		case MODBUS_FC_WRITE_MULTIPLE_COILS:
		case MODBUS_FC_WRITE_MULTIPLE_REGISTERS:
			length = 4;
			break;
		case MODBUS_FC_MASK_WRITE_REGISTER:
			length = 6;
			break;
		default:
			length = 1;
		}
	}

	return length;
}

/* Computes the length to read after the meta information (address, count, etc) */
int Modbus::compute_data_length_after_meta(uint8_t *msg,
	msg_type_t msg_type)
{
	int function = msg[this->header_length];
	int length;

	if (msg_type == MSG_INDICATION) {
		switch (function) {
		case MODBUS_FC_WRITE_MULTIPLE_COILS:
		case MODBUS_FC_WRITE_MULTIPLE_REGISTERS:
			length = msg[this->header_length + 5];
			break;
		case MODBUS_FC_WRITE_AND_READ_REGISTERS:
			length = msg[this->header_length + 9];
			break;
		default:
			length = 0;
		}
	}
	else {
		/* MSG_CONFIRMATION */
		if (function <= MODBUS_FC_READ_INPUT_REGISTERS ||
			function == MODBUS_FC_REPORT_SLAVE_ID ||
			function == MODBUS_FC_WRITE_AND_READ_REGISTERS) {
			length = msg[this->header_length + 1];
		}
		else {
			length = 0;
		}
	}

	length += this->checksum_length;

	return length;
}


/* Waits a response from a modbus server or a request from a modbus client.
This function blocks if there is no replies (3 timeouts).

The function shall return the number of received characters and the received
message in an array of uint8_t if successful. Otherwise it shall return -1
and errno is set to one of the values defined below:
- ECONNRESET
- EMBBADDATA
- EMBUNKEXC
- ETIMEDOUT
- read() or recv() error codes
*/

int Modbus::_modbus_receive_msg(uint8_t *msg, msg_type_t msg_type)
{
	int rc;
	fd_set rset;
	struct timeval tv;
	struct timeval *p_tv;
	int length_to_read;
	int msg_length = 0;
	_step_t step;

	if (this->debug) {
		if (msg_type == MSG_INDICATION) {
			printf("Waiting for an indication...\n");
		}
		else {
			printf("Waiting for a confirmation...\n");
		}
	}

	/* Add a file descriptor to the set */
	FD_ZERO(&rset);
	FD_SET(this->s, &rset);

	/* We need to analyse the message step by step.  At the first step, we want
	* to reach the function code because all packets contain this
	* information. */
	step = _STEP_FUNCTION;
	length_to_read = this->header_length + 1;

	if (msg_type == MSG_INDICATION) {
		/* Wait for a message, we don't know when the message will be
		* received */
		if (this->indication_timeout.tv_sec == 0 && this->indication_timeout.tv_usec == 0) {
			/* By default, the indication timeout isn't set */
			p_tv = NULL;
		}
		else {
			/* Wait for an indication (name of a received request by a server, see schema) */
			tv.tv_sec = this->indication_timeout.tv_sec;
			tv.tv_usec = this->indication_timeout.tv_usec;
			p_tv = &tv;
		}
	}
	else {
		tv.tv_sec = this->response_timeout.tv_sec;
		tv.tv_usec = this->response_timeout.tv_usec;
		p_tv = &tv;
	}

	while (length_to_read != 0) {
		rc = this->select(&rset, p_tv, length_to_read);
		if (rc == -1) {
			_error_print("select");
			if (this->error_recovery & MODBUS_ERROR_RECOVERY_LINK) {
				int saved_errno = errno;

				if (errno == ETIMEDOUT) {
					_sleep_response_timeout();
					flush();
				}
				else if (errno == EBADF) {
					close();
					connect();
				}
				errno = saved_errno;
			}
			return -1;
		}

		rc = this->recv(msg + msg_length, length_to_read);
		if (rc == 0) {
			errno = ECONNRESET;
			rc = -1;
		}

		if (rc == -1) {
			_error_print("read");
			if ((this->error_recovery & MODBUS_ERROR_RECOVERY_LINK) &&
				(errno == ECONNRESET || errno == ECONNREFUSED ||
					errno == EBADF)) {
				int saved_errno = errno;
				close();
				connect();
				/* Could be removed by previous calls */
				errno = saved_errno;
			}
			return -1;
		}

		/* Display the hex code of each character received */
		if (this->debug) {
			int i;
			for (i = 0; i < rc; i++)
				printf("<%.2X>", msg[msg_length + i]);
		}

		/* Sums bytes received */
		msg_length += rc;
		/* Computes remaining bytes */
		length_to_read -= rc;

		if (length_to_read == 0) {
			length_to_read = get_next_read_size(msg, msg_length, msg_type);

		}

		if (length_to_read > 0 &&
			(this->byte_timeout.tv_sec > 0 || this->byte_timeout.tv_usec > 0)) {
			/* If there is no character in the buffer, the allowed timeout
			interval between two consecutive bytes is defined by
			byte_timeout */
			tv.tv_sec = this->byte_timeout.tv_sec;
			tv.tv_usec = this->byte_timeout.tv_usec;
			p_tv = &tv;
		}
		/* else timeout isn't set again, the full response must be read before
		expiration of response timeout (for CONFIRMATION only) */
	}

	if (this->debug)
		printf("\n");

	return this->check_integrity(msg, msg_length);
}

/* Receive the request from a modbus master */
int Modbus::modbus_receive(uint8_t *req)
{
	return this->receive(req);
}

/* Receives the confirmation.

The function shall store the read response in rsp and return the number of
values (bits or words). Otherwise, its shall return -1 and errno is set.

The function doesn't check the confirmation is the expected response to the
initial request.
*/
int Modbus::modbus_receive_confirmation(uint8_t *rsp)
{

	return _modbus_receive_msg(rsp, MSG_CONFIRMATION);
}

int Modbus::check_confirmation(uint8_t *req,
	uint8_t *rsp, int rsp_length)
{
	int rc;
	int rsp_length_computed;
	const int offset = this->header_length;
	const int function = rsp[offset];

	rc = this->pre_check_confirmation(req, rsp, rsp_length);
	if (rc == -1) {
		if (this->error_recovery & MODBUS_ERROR_RECOVERY_PROTOCOL) {
			_sleep_response_timeout();
			flush();
		}
		return -1;
	}

	rsp_length_computed = compute_response_length_from_request(req);

	/* Exception code */
	if (function >= 0x80) {
		if (rsp_length == (offset + 2 + (int)this->checksum_length) &&
			req[offset] == (rsp[offset] - 0x80)) {
			/* Valid exception code received */

			int exception_code = rsp[offset + 1];
			if (exception_code < MODBUS_EXCEPTION_MAX) {
				errno = MODBUS_ENOBASE + exception_code;
			}
			else {
				errno = EMBBADEXC;
			}
			_error_print(NULL);
			return -1;
		}
		else {
			errno = EMBBADEXC;
			_error_print(NULL);
			return -1;
		}
	}

	/* Check length */
	if ((rsp_length == rsp_length_computed ||
		rsp_length_computed == MSG_LENGTH_UNDEFINED) &&
		function < 0x80) {
		int req_nb_value;
		int rsp_nb_value;

		/* Check function code */
		if (function != req[offset]) {
			if (this->debug) {
				fprintf(stderr,
					"Received function not corresponding to the request (0x%X != 0x%X)\n",
					function, req[offset]);
			}
			if (this->error_recovery & MODBUS_ERROR_RECOVERY_PROTOCOL) {
				_sleep_response_timeout();
				flush();
			}
			errno = EMBBADDATA;
			return -1;
		}

		/* Check the number of values is corresponding to the request */
		switch (function) {
		case MODBUS_FC_READ_COILS:
		case MODBUS_FC_READ_DISCRETE_INPUTS:
			/* Read functions, 8 values in a byte (nb
			* of values in the request and byte count in
			* the response. */
			req_nb_value = (req[offset + 3] << 8) + req[offset + 4];
			req_nb_value = (req_nb_value / 8) + ((req_nb_value % 8) ? 1 : 0);
			rsp_nb_value = rsp[offset + 1];
			break;
		case MODBUS_FC_WRITE_AND_READ_REGISTERS:
		case MODBUS_FC_READ_HOLDING_REGISTERS:
		case MODBUS_FC_READ_INPUT_REGISTERS:
			/* Read functions 1 value = 2 bytes */
			req_nb_value = (req[offset + 3] << 8) + req[offset + 4];
			rsp_nb_value = (rsp[offset + 1] / 2);
			break;
		case MODBUS_FC_WRITE_MULTIPLE_COILS:
		case MODBUS_FC_WRITE_MULTIPLE_REGISTERS:
			/* N Write functions */
			req_nb_value = (req[offset + 3] << 8) + req[offset + 4];
			rsp_nb_value = (rsp[offset + 3] << 8) | rsp[offset + 4];
			break;
		case MODBUS_FC_REPORT_SLAVE_ID:
			/* Report slave ID (bytes received) */
			req_nb_value = rsp_nb_value = rsp[offset + 1];
			break;
		default:
			/* 1 Write functions & others */
			req_nb_value = rsp_nb_value = 1;
		}

		if (req_nb_value == rsp_nb_value) {
			rc = rsp_nb_value;
		}
		else {
			if (this->debug) {
				fprintf(stderr,
					"Quantity not corresponding to the request (%d != %d)\n",
					rsp_nb_value, req_nb_value);
			}

			if (this->error_recovery & MODBUS_ERROR_RECOVERY_PROTOCOL) {
				_sleep_response_timeout();
				flush();
			}

			errno = EMBBADDATA;
			rc = -1;
		}
	}
	else {
		if (this->debug) {
			fprintf(stderr,
				"Message length not corresponding to the computed length (%d != %d)\n",
				rsp_length, rsp_length_computed);
		}
		if (this->error_recovery & MODBUS_ERROR_RECOVERY_PROTOCOL) {
			_sleep_response_timeout();
			flush();
		}
		errno = EMBBADDATA;
		rc = -1;
	}

	return rc;
}

int Modbus::response_io_status(uint8_t *tab_io_status,
	int address, int nb,
	uint8_t *rsp, int offset)
{
	int shift = 0;
	/* Instead of byte (not allowed in Win32) */
	int one_byte = 0;
	int i;

	for (i = address; i < address + nb; i++) {
		one_byte |= tab_io_status[i] << shift;
		if (shift == 7) {
			/* Byte is full */
			rsp[offset++] = one_byte;
			one_byte = shift = 0;
		}
		else {
			shift++;
		}
	}

	if (shift != 0)
		rsp[offset++] = one_byte;

	return offset;
}

/* Build the exception response */
int Modbus::response_exception(sft_t *sft,
	int exception_code, uint8_t *rsp,
	unsigned int to_flush,
	const char* template_, ...)
{
	int rsp_length;

	/* Print debug message */
	if (this->debug) {
		va_list ap;

		va_start(ap, template_);
		vfprintf(stderr, template_, ap);
		va_end(ap);
	}

	/* Flush if required */
	if (to_flush) {
		_sleep_response_timeout();
		flush();
	}

	/* Build exception response */
	sft->function = sft->function + 0x80;
	rsp_length = this->build_response_basis(sft, rsp);
	rsp[rsp_length++] = exception_code;

	return rsp_length;
}

/* Send a response to the received request.
Analyses the request and constructs a response.

If an error occurs, this function construct the response
accordingly.
*/
int Modbus::modbus_reply(const uint8_t *req,
	int req_length, modbus_mapping_t *mb_mapping)
{
	int offset;
	int slave;
	int function;
	uint16_t address;
	uint8_t rsp[MAX_MESSAGE_LENGTH];
	int rsp_length = 0;
	sft_t sft;


	offset = this->header_length;
	slave = req[offset - 1];
	function = req[offset];
	address = (req[offset + 1] << 8) + req[offset + 2];

	sft.slave = slave;
	sft.function = function;
	sft.t_id = this->prepare_response_tid(req, &req_length);

	/* Data are flushed on illegal number of values errors. */
	switch (function) {
	case MODBUS_FC_READ_COILS:
	case MODBUS_FC_READ_DISCRETE_INPUTS: {
		unsigned int is_input = (function == MODBUS_FC_READ_DISCRETE_INPUTS);
		int start_bits = is_input ? mb_mapping->start_input_bits : mb_mapping->start_bits;
		int nb_bits = is_input ? mb_mapping->nb_input_bits : mb_mapping->nb_bits;
		uint8_t *tab_bits = is_input ? mb_mapping->tab_input_bits : mb_mapping->tab_bits;
		const char * const name = is_input ? "read_input_bits" : "read_bits";
		int nb = (req[offset + 3] << 8) + req[offset + 4];
		/* The mapping can be shifted to reduce memory consumption and it
		doesn't always start at address zero. */
		int mapping_address = address - start_bits;

		if (nb < 1 || MODBUS_MAX_READ_BITS < nb) {
			rsp_length = response_exception(
				&sft, MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE, rsp, TRUE,
				"Illegal nb of values %d in %s (max %d)\n",
				nb, name, MODBUS_MAX_READ_BITS);
		}
		else if (mapping_address < 0 || (mapping_address + nb) > nb_bits) {
			rsp_length = response_exception(
				&sft,
				MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS, rsp, FALSE,
				"Illegal data address 0x%0X in %s\n",
				mapping_address < 0 ? address : address + nb, name);
		}
		else {
			rsp_length = this->build_response_basis(&sft, rsp);
			rsp[rsp_length++] = (nb / 8) + ((nb % 8) ? 1 : 0);
			rsp_length = response_io_status(tab_bits, mapping_address, nb,
				rsp, rsp_length);
		}
	}
										 break;
	case MODBUS_FC_READ_HOLDING_REGISTERS:
	case MODBUS_FC_READ_INPUT_REGISTERS: {
		unsigned int is_input = (function == MODBUS_FC_READ_INPUT_REGISTERS);
		int start_registers = is_input ? mb_mapping->start_input_registers : mb_mapping->start_registers;
		int nb_registers = is_input ? mb_mapping->nb_input_registers : mb_mapping->nb_registers;
		uint16_t *tab_registers = is_input ? mb_mapping->tab_input_registers : mb_mapping->tab_registers;
		const char * const name = is_input ? "read_input_registers" : "read_registers";
		int nb = (req[offset + 3] << 8) + req[offset + 4];
		/* The mapping can be shifted to reduce memory consumption and it
		doesn't always start at address zero. */
		int mapping_address = address - start_registers;

		if (nb < 1 || MODBUS_MAX_READ_REGISTERS < nb) {
			rsp_length = response_exception(
				&sft, MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE, rsp, TRUE,
				"Illegal nb of values %d in %s (max %d)\n",
				nb, name, MODBUS_MAX_READ_REGISTERS);
		}
		else if (mapping_address < 0 || (mapping_address + nb) > nb_registers) {
			rsp_length = response_exception(
				&sft, MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS, rsp, FALSE,
				"Illegal data address 0x%0X in %s\n",
				mapping_address < 0 ? address : address + nb, name);
		}
		else {
			int i;

			rsp_length = this->build_response_basis(&sft, rsp);
			rsp[rsp_length++] = nb << 1;
			for (i = mapping_address; i < mapping_address + nb; i++) {
				rsp[rsp_length++] = tab_registers[i] >> 8;
				rsp[rsp_length++] = tab_registers[i] & 0xFF;
			}
		}
	}
										 break;
	case MODBUS_FC_WRITE_SINGLE_COIL: {
		int mapping_address = address - mb_mapping->start_bits;

		if (mapping_address < 0 || mapping_address >= mb_mapping->nb_bits) {
			rsp_length = response_exception(
				&sft, MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS, rsp, FALSE,
				"Illegal data address 0x%0X in write_bit\n",
				address);
		}
		else {
			int data = (req[offset + 3] << 8) + req[offset + 4];

			if (data == 0xFF00 || data == 0x0) {
				mb_mapping->tab_bits[mapping_address] = data ? ON : OFF;
				memcpy(rsp, req, req_length);
				rsp_length = req_length;
			}
			else {
				rsp_length = response_exception(
					&sft,
					MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE, rsp, FALSE,
					"Illegal data value 0x%0X in write_bit request at address %0X\n",
					data, address);
			}
		}
	}
									  break;
	case MODBUS_FC_WRITE_SINGLE_REGISTER: {
		int mapping_address = address - mb_mapping->start_registers;

		if (mapping_address < 0 || mapping_address >= mb_mapping->nb_registers) {
			rsp_length = response_exception(
				&sft,
				MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS, rsp, FALSE,
				"Illegal data address 0x%0X in write_register\n",
				address);
		}
		else {
			int data = (req[offset + 3] << 8) + req[offset + 4];

			mb_mapping->tab_registers[mapping_address] = data;
			memcpy(rsp, req, req_length);
			rsp_length = req_length;
		}
	}
										  break;
	case MODBUS_FC_WRITE_MULTIPLE_COILS: {
		int nb = (req[offset + 3] << 8) + req[offset + 4];
		int nb_bits = req[offset + 5];
		int mapping_address = address - mb_mapping->start_bits;

		if (nb < 1 || MODBUS_MAX_WRITE_BITS < nb || nb_bits * 8 < nb) {
			/* May be the indication has been truncated on reading because of
			* invalid address (eg. nb is 0 but the request contains values to
			* write) so it's necessary to flush. */
			rsp_length = response_exception(
				&sft, MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE, rsp, TRUE,
				"Illegal number of values %d in write_bits (max %d)\n",
				nb, MODBUS_MAX_WRITE_BITS);
		}
		else if (mapping_address < 0 ||
			(mapping_address + nb) > mb_mapping->nb_bits) {
			rsp_length = response_exception(
				&sft,
				MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS, rsp, FALSE,
				"Illegal data address 0x%0X in write_bits\n",
				mapping_address < 0 ? address : address + nb);
		}
		else {
			/* 6 = byte count */
			modbus_set_bits_from_bytes(mb_mapping->tab_bits, mapping_address, nb,
				&req[offset + 6]);

			rsp_length = this->build_response_basis(&sft, rsp);
			/* 4 to copy the bit address (2) and the quantity of bits */
			memcpy(rsp + rsp_length, req + rsp_length, 4);
			rsp_length += 4;
		}
	}
										 break;
	case MODBUS_FC_WRITE_MULTIPLE_REGISTERS: {
		int nb = (req[offset + 3] << 8) + req[offset + 4];
		int nb_bytes = req[offset + 5];
		int mapping_address = address - mb_mapping->start_registers;

		if (nb < 1 || MODBUS_MAX_WRITE_REGISTERS < nb || nb_bytes != nb * 2) {
			rsp_length = response_exception(
				&sft, MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE, rsp, TRUE,
				"Illegal number of values %d in write_registers (max %d)\n",
				nb, MODBUS_MAX_WRITE_REGISTERS);
		}
		else if (mapping_address < 0 ||
			(mapping_address + nb) > mb_mapping->nb_registers) {
			rsp_length = response_exception(
				&sft, MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS, rsp, FALSE,
				"Illegal data address 0x%0X in write_registers\n",
				mapping_address < 0 ? address : address + nb);
		}
		else {
			int i, j;
			for (i = mapping_address, j = 6; i < mapping_address + nb; i++, j += 2) {
				/* 6 and 7 = first value */
				mb_mapping->tab_registers[i] =
					(req[offset + j] << 8) + req[offset + j + 1];
			}

			rsp_length = this->build_response_basis(&sft, rsp);
			/* 4 to copy the address (2) and the no. of registers */
			memcpy(rsp + rsp_length, req + rsp_length, 4);
			rsp_length += 4;
		}
	}
											 break;
	case MODBUS_FC_REPORT_SLAVE_ID: {
		int str_len;
		int byte_count_pos;

		rsp_length = this->build_response_basis(&sft, rsp);
		/* Skip byte count for now */
		byte_count_pos = rsp_length++;
		rsp[rsp_length++] = _REPORT_SLAVE_ID;
		/* Run indicator status to ON */
		rsp[rsp_length++] = 0xFF;
		/* LMB + length of LIBMODBUS_VERSION_STRING */
		str_len = 3 + strlen(LIBMODBUS_VERSION_STRING);
		memcpy(rsp + rsp_length, "LMB" LIBMODBUS_VERSION_STRING, str_len);
		rsp_length += str_len;
		rsp[byte_count_pos] = rsp_length - byte_count_pos - 1;
	}
									break;
	case MODBUS_FC_READ_EXCEPTION_STATUS:
		if (this->debug) {
			fprintf(stderr, "FIXME Not implemented\n");
		}
		errno = ENOPROTOOPT;
		return -1;
		break;
	case MODBUS_FC_MASK_WRITE_REGISTER: {
		int mapping_address = address - mb_mapping->start_registers;

		if (mapping_address < 0 || mapping_address >= mb_mapping->nb_registers) {
			rsp_length = response_exception(
				&sft, MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS, rsp, FALSE,
				"Illegal data address 0x%0X in write_register\n",
				address);
		}
		else {
			uint16_t data = mb_mapping->tab_registers[mapping_address];
			uint16_t and = (req[offset + 3] << 8) + req[offset + 4];
			uint16_t or = (req[offset + 5] << 8) + req[offset + 6];

			data = (data & and) | (or &(~and));
			mb_mapping->tab_registers[mapping_address] = data;
			memcpy(rsp, req, req_length);
			rsp_length = req_length;
		}
	}
										break;
	case MODBUS_FC_WRITE_AND_READ_REGISTERS: {
		int nb = (req[offset + 3] << 8) + req[offset + 4];
		uint16_t address_write = (req[offset + 5] << 8) + req[offset + 6];
		int nb_write = (req[offset + 7] << 8) + req[offset + 8];
		int nb_write_bytes = req[offset + 9];
		int mapping_address = address - mb_mapping->start_registers;
		int mapping_address_write = address_write - mb_mapping->start_registers;

		if (nb_write < 1 || MODBUS_MAX_WR_WRITE_REGISTERS < nb_write ||
			nb < 1 || MODBUS_MAX_WR_READ_REGISTERS < nb ||
			nb_write_bytes != nb_write * 2) {
			rsp_length = response_exception(
				&sft, MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE, rsp, TRUE,
				"Illegal nb of values (W%d, R%d) in write_and_read_registers (max W%d, R%d)\n",
				nb_write, nb, MODBUS_MAX_WR_WRITE_REGISTERS, MODBUS_MAX_WR_READ_REGISTERS);
		}
		else if (mapping_address < 0 ||
			(mapping_address + nb) > mb_mapping->nb_registers ||
			mapping_address < 0 ||
			(mapping_address_write + nb_write) > mb_mapping->nb_registers) {
			rsp_length = response_exception(
				&sft, MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS, rsp, FALSE,
				"Illegal data read address 0x%0X or write address 0x%0X write_and_read_registers\n",
				mapping_address < 0 ? address : address + nb,
				mapping_address_write < 0 ? address_write : address_write + nb_write);
		}
		else {
			int i, j;
			rsp_length = this->build_response_basis(&sft, rsp);
			rsp[rsp_length++] = nb << 1;

			/* Write first.
			10 and 11 are the offset of the first values to write */
			for (i = mapping_address_write, j = 10;
				i < mapping_address_write + nb_write; i++, j += 2) {
				mb_mapping->tab_registers[i] =
					(req[offset + j] << 8) + req[offset + j + 1];
			}

			/* and read the data for the response */
			for (i = mapping_address; i < mapping_address + nb; i++) {
				rsp[rsp_length++] = mb_mapping->tab_registers[i] >> 8;
				rsp[rsp_length++] = mb_mapping->tab_registers[i] & 0xFF;
			}
		}
	}
											 break;

	default:
		rsp_length = response_exception(
			&sft, MODBUS_EXCEPTION_ILLEGAL_FUNCTION, rsp, TRUE,
			"Unknown Modbus function code: 0x%0X\n", function);
		break;
	}

	/* Suppress any responses when the request was a broadcast */
	return (slave == MODBUS_BROADCAST_ADDRESS) ? 0 : send_msg(rsp, rsp_length);
}

int Modbus::modbus_reply_exception(const uint8_t *req,
	unsigned int exception_code)
{
	int offset;
	int slave;
	int function;
	uint8_t rsp[MAX_MESSAGE_LENGTH];
	int rsp_length;
	int dummy_length = 99;
	sft_t sft;

	offset = this->header_length;
	slave = req[offset - 1];
	function = req[offset];

	sft.slave = slave;
	sft.function = function + 0x80;
	sft.t_id = this->prepare_response_tid(req, &dummy_length);
	rsp_length = this->build_response_basis(&sft, rsp);

	/* Positive exception code */
	if (exception_code < MODBUS_EXCEPTION_MAX) {
		rsp[rsp_length++] = exception_code;
		return send_msg(rsp, rsp_length);
	}
	else {
		errno = EINVAL;
		return -1;
	}
}

/* Reads IO status */
int Modbus::read_io_status(int function,
	int addr, int nb, uint8_t *dest)
{
	int rc;
	int req_length;

	uint8_t req[_MIN_REQ_LENGTH];
	uint8_t rsp[MAX_MESSAGE_LENGTH];

	req_length = this->build_request_basis(function, addr, nb, req);

	rc = send_msg(req, req_length);
	if (rc > 0) {
		int i, temp, bit;
		int pos = 0;
		int offset;
		int offset_end;

		rc = _modbus_receive_msg(rsp, MSG_CONFIRMATION);
		if (rc == -1)
			return -1;

		rc = check_confirmation(req, rsp, rc);
		if (rc == -1)
			return -1;

		offset = this->header_length + 2;
		offset_end = offset + rc;
		for (i = offset; i < offset_end; i++) {
			/* Shift reg hi_byte to temp */
			temp = rsp[i];

			for (bit = 0x01; (bit & 0xff) && (pos < nb);) {
				dest[pos++] = (temp & bit) ? TRUE : FALSE;
				bit = bit << 1;
			}

		}
	}

	return rc;
}

/* Reads the boolean status of bits and sets the array elements
in the destination to TRUE or FALSE (single bits). */
int Modbus::modbus_read_bits(int addr, int nb, uint8_t *dest)
{
	int rc;

	if (nb > MODBUS_MAX_READ_BITS) {
		if (this->debug) {
			fprintf(stderr,
				"ERROR Too many bits requested (%d > %d)\n",
				nb, MODBUS_MAX_READ_BITS);
		}
		errno = EMBMDATA;
		return -1;
	}

	rc = read_io_status(MODBUS_FC_READ_COILS, addr, nb, dest);

	if (rc == -1)
		return -1;
	else
		return nb;
}


/* Same as modbus_read_bits but reads the remote device input table */
int Modbus::modbus_read_input_bits(int addr, int nb, uint8_t *dest)
{
	int rc;

	if (nb > MODBUS_MAX_READ_BITS) {
		if (this->debug) {
			fprintf(stderr,
				"ERROR Too many discrete inputs requested (%d > %d)\n",
				nb, MODBUS_MAX_READ_BITS);
		}
		errno = EMBMDATA;
		return -1;
	}

	rc = read_io_status(MODBUS_FC_READ_DISCRETE_INPUTS, addr, nb, dest);

	if (rc == -1)
		return -1;
	else
		return nb;
}

/* Reads the data from a remove device and put that data into an array */
int Modbus::read_registers(int function, int addr, int nb,
	uint16_t *dest)
{
	int rc;
	int req_length;
	uint8_t req[_MIN_REQ_LENGTH];
	uint8_t rsp[MAX_MESSAGE_LENGTH];

	if (nb > MODBUS_MAX_READ_REGISTERS) {
		if (this->debug) {
			fprintf(stderr,
				"ERROR Too many registers requested (%d > %d)\n",
				nb, MODBUS_MAX_READ_REGISTERS);
		}
		errno = EMBMDATA;
		return -1;
	}

	req_length = this->build_request_basis(function, addr, nb, req);

	rc = send_msg(req, req_length);
	if (rc > 0) {
		int offset;
		int i;

		rc = _modbus_receive_msg(rsp, MSG_CONFIRMATION);
		if (rc == -1)
			return -1;

		rc = check_confirmation(req, rsp, rc);
		if (rc == -1)
			return -1;

		offset = this->header_length;

		for (i = 0; i < rc; i++) {
			/* shift reg hi_byte to temp OR with lo_byte */
			dest[i] = (rsp[offset + 2 + (i << 1)] << 8) |
				rsp[offset + 3 + (i << 1)];
		}
	}

	return rc;
}

/* Reads the holding registers of remote device and put the data into an
array */
int Modbus::modbus_read_registers(int addr, int nb, uint16_t *dest)
{
	int status;

	if (nb > MODBUS_MAX_READ_REGISTERS) {
		if (this->debug) {
			fprintf(stderr,
				"ERROR Too many registers requested (%d > %d)\n",
				nb, MODBUS_MAX_READ_REGISTERS);
		}
		errno = EMBMDATA;
		return -1;
	}

	status = read_registers(MODBUS_FC_READ_HOLDING_REGISTERS,
		addr, nb, dest);
	return status;
}

/* Reads the input registers of remote device and put the data into an array */
int Modbus::modbus_read_input_registers(int addr, int nb,
	uint16_t *dest)
{
	int status;

	if (nb > MODBUS_MAX_READ_REGISTERS) {
		fprintf(stderr,
			"ERROR Too many input registers requested (%d > %d)\n",
			nb, MODBUS_MAX_READ_REGISTERS);
		errno = EMBMDATA;
		return -1;
	}

	status = read_registers(MODBUS_FC_READ_INPUT_REGISTERS,
		addr, nb, dest);

	return status;
}

/* Write a value to the specified register of the remote device.
Used by write_bit and write_register */
int Modbus::write_single(int function, int addr, const uint16_t value)
{
	int rc;
	int req_length;
	uint8_t req[_MIN_REQ_LENGTH];


	req_length = this->build_request_basis(function, addr, (int)value, req);

	rc = send_msg(req, req_length);
	if (rc > 0) {
		/* Used by write_bit and write_register */
		uint8_t rsp[MAX_MESSAGE_LENGTH];

		rc = _modbus_receive_msg(rsp, MSG_CONFIRMATION);
		if (rc == -1)
			return -1;

		rc = check_confirmation(req, rsp, rc);
	}

	return rc;
}

/* Turns ON or OFF a single bit of the remote device */
int Modbus::modbus_write_bit(int addr, int status)
{
	return write_single(MODBUS_FC_WRITE_SINGLE_COIL, addr,
		status ? 0xFF00 : 0);
}

/* Writes a value in one register of the remote device */
int Modbus::modbus_write_register(int addr, const uint16_t value)
{

	return write_single(MODBUS_FC_WRITE_SINGLE_REGISTER, addr, value);
}

/* Write the bits of the array in the remote device */
int Modbus::modbus_write_bits(int addr, int nb, const uint8_t *src)
{
	int rc;
	int i;
	int byte_count;
	int req_length;
	int bit_check = 0;
	int pos = 0;
	uint8_t req[MAX_MESSAGE_LENGTH];

	if (nb > MODBUS_MAX_WRITE_BITS) {
		if (this->debug) {
			fprintf(stderr, "ERROR Writing too many bits (%d > %d)\n",
				nb, MODBUS_MAX_WRITE_BITS);
		}
		errno = EMBMDATA;
		return -1;
	}

	req_length = this->build_request_basis(MODBUS_FC_WRITE_MULTIPLE_COILS, addr, nb, req);
	byte_count = (nb / 8) + ((nb % 8) ? 1 : 0);
	req[req_length++] = byte_count;

	for (i = 0; i < byte_count; i++) {
		int bit;

		bit = 0x01;
		req[req_length] = 0;

		while ((bit & 0xFF) && (bit_check++ < nb)) {
			if (src[pos++])
				req[req_length] |= bit;
			else
				req[req_length] &= ~bit;

			bit = bit << 1;
		}
		req_length++;
	}

	rc = send_msg(req, req_length);
	if (rc > 0) {
		uint8_t rsp[MAX_MESSAGE_LENGTH];

		rc = _modbus_receive_msg(rsp, MSG_CONFIRMATION);
		if (rc == -1)
			return -1;

		rc = check_confirmation(req, rsp, rc);
	}


	return rc;
}

/* Write the values from the array to the registers of the remote device */
int Modbus::modbus_write_registers(int addr, int nb, const uint16_t *src)
{
	int rc;
	int i;
	int req_length;
	int byte_count;
	uint8_t req[MAX_MESSAGE_LENGTH];

	if (nb > MODBUS_MAX_WRITE_REGISTERS) {
		if (this->debug) {
			fprintf(stderr,
				"ERROR Trying to write to too many registers (%d > %d)\n",
				nb, MODBUS_MAX_WRITE_REGISTERS);
		}
		errno = EMBMDATA;
		return -1;
	}

	req_length = this->build_request_basis(MODBUS_FC_WRITE_MULTIPLE_REGISTERS, addr, nb, req);
	byte_count = nb * 2;
	req[req_length++] = byte_count;

	for (i = 0; i < nb; i++) {
		req[req_length++] = src[i] >> 8;
		req[req_length++] = src[i] & 0x00FF;
	}

	rc = send_msg(req, req_length);
	if (rc > 0) {
		uint8_t rsp[MAX_MESSAGE_LENGTH];

		rc = _modbus_receive_msg(rsp, MSG_CONFIRMATION);
		if (rc == -1)
			return -1;

		rc = check_confirmation(req, rsp, rc);
	}

	return rc;
}

int Modbus::modbus_mask_write_register(int addr, uint16_t and_mask, uint16_t or_mask)
{
	int rc;
	int req_length;
	/* The request length can not exceed _MIN_REQ_LENGTH - 2 and 4 bytes to
	* store the masks. The ugly substraction is there to remove the 'nb' value
	* (2 bytes) which is not used. */
	uint8_t req[_MIN_REQ_LENGTH + 2];

	req_length = this->build_request_basis(
		MODBUS_FC_MASK_WRITE_REGISTER,
		addr, 0, req);

	/* HACKISH, count is not used */
	req_length -= 2;

	req[req_length++] = and_mask >> 8;
	req[req_length++] = and_mask & 0x00ff;
	req[req_length++] = or_mask >> 8;
	req[req_length++] = or_mask & 0x00ff;

	rc = send_msg(req, req_length);
	if (rc > 0) {
		/* Used by write_bit and write_register */
		uint8_t rsp[MAX_MESSAGE_LENGTH];

		rc = _modbus_receive_msg(rsp, MSG_CONFIRMATION);
		if (rc == -1)
			return -1;

		rc = check_confirmation(req, rsp, rc);
	}

	return rc;
}

/* Write multiple registers from src array to remote device and read multiple
registers from remote device to dest array. */
int Modbus::modbus_write_and_read_registers(
	int write_addr, int write_nb,
	const uint16_t *src,
	int read_addr, int read_nb,
	uint16_t *dest)

{
	int rc;
	int req_length;
	int i;
	int byte_count;
	uint8_t req[MAX_MESSAGE_LENGTH];
	uint8_t rsp[MAX_MESSAGE_LENGTH];


	if (write_nb > MODBUS_MAX_WR_WRITE_REGISTERS) {
		if (this->debug) {
			fprintf(stderr,
				"ERROR Too many registers to write (%d > %d)\n",
				write_nb, MODBUS_MAX_WR_WRITE_REGISTERS);
		}
		errno = EMBMDATA;
		return -1;
	}

	if (read_nb > MODBUS_MAX_WR_READ_REGISTERS) {
		if (this->debug) {
			fprintf(stderr,
				"ERROR Too many registers requested (%d > %d)\n",
				read_nb, MODBUS_MAX_WR_READ_REGISTERS);
		}
		errno = EMBMDATA;
		return -1;
	}
	req_length = this->build_request_basis(MODBUS_FC_WRITE_AND_READ_REGISTERS,
		read_addr, read_nb, req);

	req[req_length++] = write_addr >> 8;
	req[req_length++] = write_addr & 0x00ff;
	req[req_length++] = write_nb >> 8;
	req[req_length++] = write_nb & 0x00ff;
	byte_count = write_nb * 2;
	req[req_length++] = byte_count;

	for (i = 0; i < write_nb; i++) {
		req[req_length++] = src[i] >> 8;
		req[req_length++] = src[i] & 0x00FF;
	}

	rc = send_msg(req, req_length);
	if (rc > 0) {
		int offset;

		rc = _modbus_receive_msg(rsp, MSG_CONFIRMATION);
		if (rc == -1)
			return -1;

		rc = check_confirmation(req, rsp, rc);
		if (rc == -1)
			return -1;

		offset = this->header_length;
		for (i = 0; i < rc; i++) {
			/* shift reg hi_byte to temp OR with lo_byte */
			dest[i] = (rsp[offset + 2 + (i << 1)] << 8) |
				rsp[offset + 3 + (i << 1)];
		}
	}

	return rc;
}

/* Send a request to get the slave ID of the device (only available in serial
communication). */
int Modbus::modbus_report_slave_id(int max_dest, uint8_t *dest)
{
	int rc;
	int req_length;
	uint8_t req[_MIN_REQ_LENGTH];

	if (max_dest <= 0) {
		errno = EINVAL;
		return -1;
	}

	req_length = this->build_request_basis(MODBUS_FC_REPORT_SLAVE_ID,
		0, 0, req);

	/* HACKISH, addr and count are not used */
	req_length -= 4;

	rc = send_msg(req, req_length);
	if (rc > 0) {
		int i;
		int offset;
		uint8_t rsp[MAX_MESSAGE_LENGTH];

		rc = _modbus_receive_msg(rsp, MSG_CONFIRMATION);
		if (rc == -1)
			return -1;

		rc = check_confirmation(req, rsp, rc);
		if (rc == -1)
			return -1;

		offset = this->header_length + 2;

		/* Byte count, slave id, run indicator status and
		additional data. Truncate copy to max_dest. */
		for (i = 0; i < rc && i < max_dest; i++) {
			dest[i] = rsp[offset + i];
		}
	}

	return rc;
}

int Modbus::modbus_get_slave()
{

	return this->slave;
}

int Modbus::modbus_set_error_recovery(
	modbus_error_recovery_mode error_recovery)
{
	/* The type of modbus_error_recovery_mode is unsigned enum */
	this->error_recovery = (uint8_t)error_recovery;
	return 0;
}

int Modbus::modbus_set_socket(int s)
{

	this->s = s;
	return 0;
}

int Modbus::modbus_get_socket()
{

	return this->s;
}

/* Get the timeout interval used to wait for a response */
int Modbus::modbus_get_response_timeout(uint32_t *to_sec, uint32_t *to_usec)
{

	*to_sec = this->response_timeout.tv_sec;
	*to_usec = this->response_timeout.tv_usec;
	return 0;
}

int Modbus::modbus_set_response_timeout(uint32_t to_sec, uint32_t to_usec)
{
	if ((to_sec == 0 && to_usec == 0) || to_usec > 999999) {
		errno = EINVAL;
		return -1;
	}

	this->response_timeout.tv_sec = to_sec;
	this->response_timeout.tv_usec = to_usec;
	return 0;
}

/* Get the timeout interval between two consecutive bytes of a message */
int Modbus::modbus_get_byte_timeout(uint32_t *to_sec, uint32_t *to_usec)
{
	*to_sec = this->byte_timeout.tv_sec;
	*to_usec = this->byte_timeout.tv_usec;
	return 0;
}

int Modbus::modbus_set_byte_timeout(uint32_t to_sec, uint32_t to_usec)
{
	/* Byte timeout can be disabled when both values are zero */
	if ( to_usec > 999999) {
		errno = EINVAL;
		return -1;
	}

	this->byte_timeout.tv_sec = to_sec;
	this->byte_timeout.tv_usec = to_usec;
	return 0;
}

/* Get the timeout interval used by the server to wait for an indication from a client */
int Modbus::modbus_get_indication_timeout(uint32_t *to_sec, uint32_t *to_usec)
{

	*to_sec = this->indication_timeout.tv_sec;
	*to_usec = this->indication_timeout.tv_usec;
	return 0;
}

int Modbus::modbus_set_indication_timeout(uint32_t to_sec, uint32_t to_usec)
{
	/* Indication timeout can be disabled when both values are zero */
	if (to_usec > 999999) {
		errno = EINVAL;
		return -1;
	}

	this->indication_timeout.tv_sec = to_sec;
	this->indication_timeout.tv_usec = to_usec;
	return 0;
}

int Modbus::modbus_get_header_length()
{
	return this->header_length;
}


int Modbus::modbus_set_debug(int flag)
{
	this->debug = flag;
	return 0;
}


#ifndef HAVE_STRLCPY
/*
* Function strlcpy was originally developed by
* Todd C. Miller <Todd.Miller@courtesan.com> to simplify writing secure code.
* See ftp://ftp.openbsd.org/pub/OpenBSD/src/lib/libc/string/strlcpy.3
* for more information.
*
* Thank you Ulrich Drepper... not!
*
* Copy src to string dest of size dest_size.  At most dest_size-1 characters
* will be copied.  Always NUL terminates (unless dest_size == 0).  Returns
* strlen(src); if retval >= dest_size, truncation occurred.
*/
size_t strlcpy(char *dest, const char *src, size_t dest_size)
{
	register char *d = dest;
	register const char *s = src;
	register size_t n = dest_size;

	/* Copy as many bytes as will fit */
	if (n != 0 && --n != 0) {
		do {
			if ((*d++ = *s++) == 0)
				break;
		} while (--n != 0);
	}

	/* Not enough room in dest, add NUL and traverse rest of src */
	if (n == 0) {
		if (dest_size != 0)
			*d = '\0'; /* NUL-terminate dest */
		while (*s++)
			;
	}

	return (s - src - 1); /* count does not include NUL */
}
#endif
