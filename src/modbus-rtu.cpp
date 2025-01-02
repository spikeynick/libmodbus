#include "modbus-rtu.hpp"
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <fcntl.h>
#include <string.h>
#ifndef _MSC_VER
#include <unistd.h>
#endif
#include "util.hpp"
#include "bl/logger/log.hpp"

ModbusRTU::ModbusRTU(int slave, MBConnectionPtr connection) :
	Modbus(slave, _MODBUS_RTU_HEADER_LENGTH, _MODBUS_RTU_CHECKSUM_LENGTH, MODBUS_RTU_MAX_ADU_LENGTH),
    connection(connection)
{
}



void ModbusRTU::close()
{
    connection->close();
}

void ModbusRTU::free()
{

}

bool ModbusRTU::flush()
{
    return connection->flush();

}

int ModbusRTU::build_response_basis(sft_t * sft, uint8_t * rsp)
{
	/* In this case, the slave is certainly valid because a check is already
	* done in _modbus_rtu_listen */
	rsp[0] = sft->slave;
	rsp[1] = sft->function;

	return _MODBUS_RTU_PRESET_RSP_LENGTH;
}

int ModbusRTU::send_msg_pre(uint8_t * msg, int msg_length)
{
	uint16_t crc = crc16(msg, msg_length);
	msg[msg_length++] = crc >> 8;
	msg[msg_length++] = crc & 0x00FF;

	return msg_length;
}

int ModbusRTU::send(uint8_t * msg, int msg_length)
{
    return connection->send(msg, msg_length);
}

int ModbusRTU::select(fd_set * rset, timeval * tv, int length_to_read)
{
    return connection->select(rset, tv, length_to_read);
}

ssize_t ModbusRTU::recv(uint8_t * rsp, int rsp_length)
{
    return connection->recv(rsp, rsp_length);
}

int ModbusRTU::check_integrity(uint8_t * msg, const int msg_length)
{
	uint16_t crc_calculated;
	uint16_t crc_received;
	int slave = msg[0];

	/* Filter on the Modbus unit identifier (slave) in RTU mode to avoid useless
	* CRC computing. */
	if (slave != this->slave && slave != MODBUS_BROADCAST_ADDRESS) {
		if (this->debug) {
			printf("Request for slave %d ignored (not %d)\n", slave, this->slave);
		}
		/* Following call to check_confirmation handles this error */
		return 0;
	}

	crc_calculated = crc16(msg, msg_length - 2);
	crc_received = (msg[msg_length - 2] << 8) | msg[msg_length - 1];

	/* Check CRC of msg */
	if (crc_calculated == crc_received) {
		return msg_length;
	}
	else {
		//if (this->debug) {
			bl_log_error("ERROR CRC received 0x" << std::hex << (short)crc_received << " != CRC calculated 0x"<< std::hex<< crc_calculated);
			std::ostringstream ss;
			ss << "INPUT: ";
			for (int i = 0; i < msg_length; ++i) {
				ss << "<" << std::hex << (short)msg[i] << ">";
			}
			bl_log_error(ss.str());
		//}

		if (this->error_recovery & MODBUS_ERROR_RECOVERY_PROTOCOL) {
			this->flush();
		}
		errno = EMBBADCRC;
		return -1;
	}
}

int ModbusRTU::receive(uint8_t * req)
{
	int rc;

	if (this->confirmation_to_ignore) {
		this->_modbus_receive_msg(req, MSG_CONFIRMATION);
		/* Ignore errors and reset the flag */
		this->confirmation_to_ignore = FALSE;
		rc = 0;
		if (this->debug) {
			printf("Confirmation to ignore\n");
		}
	}
	else {
		rc = this->_modbus_receive_msg(req, MSG_INDICATION);
		if (rc == 0) {
			/* The next expected message is a confirmation to ignore */
			this->confirmation_to_ignore = TRUE;
		}
	}
	return rc;
}

int ModbusRTU::pre_check_confirmation(const uint8_t * req, const uint8_t * rsp, int rsp_length)
{
	/* Check responding slave is the slave we requested (except for broacast
	* request) */
	if (req[0] != rsp[0] && req[0] != MODBUS_BROADCAST_ADDRESS) {
		if (this->debug) {
			fprintf(stderr,
				"The responding slave %d isn't the requested slave %d\n",
				rsp[0], req[0]);
		}
		errno = EMBBADSLAVE;
		return -1;
	}
	else {
		return 0;
	}
}

int ModbusRTU::prepare_response_tid(const uint8_t * req, int * req_length)
{
	(*req_length) -= _MODBUS_RTU_CHECKSUM_LENGTH;
	/* No TID */
	return 0;
}

int ModbusRTU::build_request_basis(int function, int addr, int nb, uint8_t * req)
{
	req[0] = this->slave;
	req[1] = function;
	req[2] = addr >> 8;
	req[3] = addr & 0x00ff;
	req[4] = nb >> 8;
	req[5] = nb & 0x00ff;

	return _MODBUS_RTU_PRESET_REQ_LENGTH;
}
