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

ModbusRTU::ModbusRTU(int slave, const std::string &device, int baud, char parity, int data_bit, int stop_bit) :
	Modbus(slave, _MODBUS_RTU_HEADER_LENGTH, _MODBUS_RTU_CHECKSUM_LENGTH, MODBUS_RTU_MAX_ADU_LENGTH),
	device(device),
	baud(baud),
	parity(parity),
	data_bit(data_bit),
	stop_bit(stop_bit)
{
}

int ModbusRTU::connect()
{
#if defined(_WIN32)
	DCB dcb;
#else
	struct termios tios;
	speed_t speed;
	int flags;
#endif
	if (this->debug) {
		printf("Opening %s at %d bauds (%c, %d, %d)\n",
			this->device.c_str(), this->baud, this->parity,
			this->data_bit, this->stop_bit);
	}

#if defined(_WIN32)
	/* Some references here:
	* http://msdn.microsoft.com/en-us/library/aa450602.aspx
	*/
	win32_ser_init(&this->w_ser);

	/* this->device should contain a string like "COMxx:" xx being a decimal
	* number */
	this->w_ser.fd = CreateFileA(this->device.c_str(),
		GENERIC_READ | GENERIC_WRITE,
		0,
		NULL,
		OPEN_EXISTING,
		0,
		NULL);

	/* Error checking */
	if (this->w_ser.fd == INVALID_HANDLE_VALUE) {
		if (this->debug) {
			fprintf(stderr, "ERROR Can't open the device %s (LastError %d)\n",
				this->device.c_str(), (int)GetLastError());
		}
		return -1;
	}

	/* Save params */
	this->old_dcb.DCBlength = sizeof(DCB);
	if (!GetCommState(this->w_ser.fd, &this->old_dcb)) {
		if (this->debug) {
			fprintf(stderr, "ERROR Error getting configuration (LastError %d)\n",
				(int)GetLastError());
		}
		CloseHandle(this->w_ser.fd);
		this->w_ser.fd = INVALID_HANDLE_VALUE;
		return -1;
	}

	/* Build new configuration (starting from current settings) */
	dcb = this->old_dcb;

	/* Speed setting */
	switch (this->baud) {
	case 110:
		dcb.BaudRate = CBR_110;
		break;
	case 300:
		dcb.BaudRate = CBR_300;
		break;
	case 600:
		dcb.BaudRate = CBR_600;
		break;
	case 1200:
		dcb.BaudRate = CBR_1200;
		break;
	case 2400:
		dcb.BaudRate = CBR_2400;
		break;
	case 4800:
		dcb.BaudRate = CBR_4800;
		break;
	case 9600:
		dcb.BaudRate = CBR_9600;
		break;
	case 14400:
		dcb.BaudRate = CBR_14400;
		break;
	case 19200:
		dcb.BaudRate = CBR_19200;
		break;
	case 38400:
		dcb.BaudRate = CBR_38400;
		break;
	case 57600:
		dcb.BaudRate = CBR_57600;
		break;
	case 115200:
		dcb.BaudRate = CBR_115200;
		break;
	case 230400:
		/* CBR_230400 - not defined */
		dcb.BaudRate = 230400;
		break;
	case 250000:
		dcb.BaudRate = 250000;
		break;
	case 460800:
		dcb.BaudRate = 460800;
		break;
	case 500000:
		dcb.BaudRate = 500000;
		break;
	case 921600:
		dcb.BaudRate = 921600;
		break;
	case 1000000:
		dcb.BaudRate = 1000000;
		break;
	default:
		dcb.BaudRate = CBR_9600;
		if (this->debug) {
			fprintf(stderr, "WARNING Unknown baud rate %d for %s (B9600 used)\n",
				this->baud, this->device.c_str());
		}
	}

	/* Data bits */
	switch (this->data_bit) {
	case 5:
		dcb.ByteSize = 5;
		break;
	case 6:
		dcb.ByteSize = 6;
		break;
	case 7:
		dcb.ByteSize = 7;
		break;
	case 8:
	default:
		dcb.ByteSize = 8;
		break;
	}

	/* Stop bits */
	if (this->stop_bit == 1)
		dcb.StopBits = ONESTOPBIT;
	else /* 2 */
		dcb.StopBits = TWOSTOPBITS;

	/* Parity */
	if (this->parity == 'N') {
		dcb.Parity = NOPARITY;
		dcb.fParity = FALSE;
	}
	else if (this->parity == 'E') {
		dcb.Parity = EVENPARITY;
		dcb.fParity = TRUE;
	}
	else {
		/* odd */
		dcb.Parity = ODDPARITY;
		dcb.fParity = TRUE;
	}

	/* Hardware handshaking left as default settings retrieved */

	/* No software handshaking */
	dcb.fTXContinueOnXoff = TRUE;
	dcb.fOutX = FALSE;
	dcb.fInX = FALSE;

	/* Binary mode (it's the only supported on Windows anyway) */
	dcb.fBinary = TRUE;

	/* Don't want errors to be blocking */
	dcb.fAbortOnError = FALSE;

	/* Setup port */
	if (!SetCommState(this->w_ser.fd, &dcb)) {
		if (this->debug) {
			fprintf(stderr, "ERROR Error setting new configuration (LastError %d)\n",
				(int)GetLastError());
		}
		CloseHandle(this->w_ser.fd);
		this->w_ser.fd = INVALID_HANDLE_VALUE;
		return -1;
	}
#else
	/* The O_NOCTTY flag tells UNIX that this program doesn't want
	to be the "controlling terminal" for that port. If you
	don't specify this then any input (such as keyboard abort
	signals and so forth) will affect your process

	Timeouts are ignored in canonical input mode or when the
	NDELAY option is set on the file via open or fcntl */
	flags = O_RDWR | O_NOCTTY | O_NDELAY | O_EXCL;
#ifdef O_CLOEXEC
	flags |= O_CLOEXEC;
#endif

	this->s = open(this->device, flags);
	if (this->s == -1) {
		if (this->debug) {
			fprintf(stderr, "ERROR Can't open the device %s (%s)\n",
				this->device, strerror(errno));
		}
		return -1;
	}

	/* Save */
	tcgetattr(this->s, &this->old_tios);

	memset(&tios, 0, sizeof(struct termios));

	/* C_ISPEED     Input baud (new interface)
	C_OSPEED     Output baud (new interface)
	*/
	switch (this->baud) {
	case 110:
		speed = B110;
		break;
	case 300:
		speed = B300;
		break;
	case 600:
		speed = B600;
		break;
	case 1200:
		speed = B1200;
		break;
	case 2400:
		speed = B2400;
		break;
	case 4800:
		speed = B4800;
		break;
	case 9600:
		speed = B9600;
		break;
	case 19200:
		speed = B19200;
		break;
	case 38400:
		speed = B38400;
		break;
#ifdef B57600
	case 57600:
		speed = B57600;
		break;
#endif
#ifdef B115200
	case 115200:
		speed = B115200;
		break;
#endif
#ifdef B230400
	case 230400:
		speed = B230400;
		break;
#endif
#ifdef B460800
	case 460800:
		speed = B460800;
		break;
#endif
#ifdef B500000
	case 500000:
		speed = B500000;
		break;
#endif
#ifdef B576000
	case 576000:
		speed = B576000;
		break;
#endif
#ifdef B921600
	case 921600:
		speed = B921600;
		break;
#endif
#ifdef B1000000
	case 1000000:
		speed = B1000000;
		break;
#endif
#ifdef B1152000
	case 1152000:
		speed = B1152000;
		break;
#endif
#ifdef B1500000
	case 1500000:
		speed = B1500000;
		break;
#endif
#ifdef B2500000
	case 2500000:
		speed = B2500000;
		break;
#endif
#ifdef B3000000
	case 3000000:
		speed = B3000000;
		break;
#endif
#ifdef B3500000
	case 3500000:
		speed = B3500000;
		break;
#endif
#ifdef B4000000
	case 4000000:
		speed = B4000000;
		break;
#endif
	default:
		speed = B9600;
		if (this->debug) {
			fprintf(stderr,
				"WARNING Unknown baud rate %d for %s (B9600 used)\n",
				this->baud, this->device);
		}
	}

	/* Set the baud rate */
	if ((cfsetispeed(&tios, speed) < 0) ||
		(cfsetospeed(&tios, speed) < 0)) {
		close(this->s);
		this->s = -1;
		return -1;
	}

	/* C_CFLAG      Control options
	CLOCAL       Local line - do not change "owner" of port
	CREAD        Enable receiver
	*/
	tios.c_cflag |= (CREAD | CLOCAL);
	/* CSIZE, HUPCL, CRTSCTS (hardware flow control) */

	/* Set data bits (5, 6, 7, 8 bits)
	CSIZE        Bit mask for data bits
	*/
	tios.c_cflag &= ~CSIZE;
	switch (this->data_bit) {
	case 5:
		tios.c_cflag |= CS5;
		break;
	case 6:
		tios.c_cflag |= CS6;
		break;
	case 7:
		tios.c_cflag |= CS7;
		break;
	case 8:
	default:
		tios.c_cflag |= CS8;
		break;
	}

	/* Stop bit (1 or 2) */
	if (this->stop_bit == 1)
		tios.c_cflag &= ~CSTOPB;
	else /* 2 */
		tios.c_cflag |= CSTOPB;

	/* PARENB       Enable parity bit
	PARODD       Use odd parity instead of even */
	if (this->parity == 'N') {
		/* None */
		tios.c_cflag &= ~PARENB;
	}
	else if (this->parity == 'E') {
		/* Even */
		tios.c_cflag |= PARENB;
		tios.c_cflag &= ~PARODD;
	}
	else {
		/* Odd */
		tios.c_cflag |= PARENB;
		tios.c_cflag |= PARODD;
	}

	/* Read the man page of termios if you need more information. */

	/* This field isn't used on POSIX systems
	tios.c_line = 0;
	*/

	/* C_LFLAG      Line options

	ISIG Enable SIGINTR, SIGSUSP, SIGDSUSP, and SIGQUIT signals
	ICANON       Enable canonical input (else raw)
	XCASE        Map uppercase \lowercase (obsolete)
	ECHO Enable echoing of input characters
	ECHOE        Echo erase character as BS-SP-BS
	ECHOK        Echo NL after kill character
	ECHONL       Echo NL
	NOFLSH       Disable flushing of input buffers after
	interrupt or quit characters
	IEXTEN       Enable extended functions
	ECHOCTL      Echo control characters as ^char and delete as ~?
	ECHOPRT      Echo erased character as character erased
	ECHOKE       BS-SP-BS entire line on line kill
	FLUSHO       Output being flushed
	PENDIN       Retype pending input at next read or input char
	TOSTOP       Send SIGTTOU for background output

	Canonical input is line-oriented. Input characters are put
	into a buffer which can be edited interactively by the user
	until a CR (carriage return) or LF (line feed) character is
	received.

	Raw input is unprocessed. Input characters are passed
	through exactly as they are received, when they are
	received. Generally you'll deselect the ICANON, ECHO,
	ECHOE, and ISIG options when using raw input
	*/

	/* Raw input */
	tios.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

	/* C_IFLAG      Input options

	Constant     Description
	INPCK        Enable parity check
	IGNPAR       Ignore parity errors
	PARMRK       Mark parity errors
	ISTRIP       Strip parity bits
	IXON Enable software flow control (outgoing)
	IXOFF        Enable software flow control (incoming)
	IXANY        Allow any character to start flow again
	IGNBRK       Ignore break condition
	BRKINT       Send a SIGINT when a break condition is detected
	INLCR        Map NL to CR
	IGNCR        Ignore CR
	ICRNL        Map CR to NL
	IUCLC        Map uppercase to lowercase
	IMAXBEL      Echo BEL on input line too long
	*/
	if (this->parity == 'N') {
		/* None */
		tios.c_iflag &= ~INPCK;
	}
	else {
		tios.c_iflag |= INPCK;
	}

	/* Software flow control is disabled */
	tios.c_iflag &= ~(IXON | IXOFF | IXANY);

	/* C_OFLAG      Output options
	OPOST        Postprocess output (not set = raw output)
	ONLCR        Map NL to CR-NL

	ONCLR ant others needs OPOST to be enabled
	*/

	/* Raw ouput */
	tios.c_oflag &= ~OPOST;

	/* C_CC         Control characters
	VMIN         Minimum number of characters to read
	VTIME        Time to wait for data (tenths of seconds)

	UNIX serial interface drivers provide the ability to
	specify character and packet timeouts. Two elements of the
	c_cc array are used for timeouts: VMIN and VTIME. Timeouts
	are ignored in canonical input mode or when the NDELAY
	option is set on the file via open or fcntl.

	VMIN specifies the minimum number of characters to read. If
	it is set to 0, then the VTIME value specifies the time to
	wait for every character read. Note that this does not mean
	that a read call for N bytes will wait for N characters to
	come in. Rather, the timeout will apply to the first
	character and the read call will return the number of
	characters immediately available (up to the number you
	request).

	If VMIN is non-zero, VTIME specifies the time to wait for
	the first character read. If a character is read within the
	time given, any read will block (wait) until all VMIN
	characters are read. That is, once the first character is
	read, the serial interface driver expects to receive an
	entire packet of characters (VMIN bytes total). If no
	character is read within the time allowed, then the call to
	read returns 0. This method allows you to tell the serial
	driver you need exactly N bytes and any read call will
	return 0 or N bytes. However, the timeout only applies to
	the first character read, so if for some reason the driver
	misses one character inside the N byte packet then the read
	call could block forever waiting for additional input
	characters.

	VTIME specifies the amount of time to wait for incoming
	characters in tenths of seconds. If VTIME is set to 0 (the
	default), reads will block (wait) indefinitely unless the
	NDELAY option is set on the port with open or fcntl.
	*/
	/* Unused because we use open with the NDELAY option */
	tios.c_cc[VMIN] = 0;
	tios.c_cc[VTIME] = 0;

	if (tcsetattr(this->s, TCSANOW, &tios) < 0) {
		close(this->s);
		this->s = -1;
		return -1;
	}
#endif

	return 0;
}

void ModbusRTU::close()
{
#if defined(_WIN32)
	/* Revert settings */
	if (!SetCommState(this->w_ser.fd, &this->old_dcb) && this->debug) {
		fprintf(stderr, "ERROR Couldn't revert to configuration (LastError %d)\n",
			(int)GetLastError());
	}

	if (!CloseHandle(this->w_ser.fd) && this->debug) {
		fprintf(stderr, "ERROR Error while closing handle (LastError %d)\n",
			(int)GetLastError());
	}
#else
	if (this->s != -1) {
		tcsetattr(this->s, TCSANOW, &this->old_tios);
		close(this->s);
		this->s = -1;
	}
#endif
}

void ModbusRTU::free()
{

}

bool ModbusRTU::flush()
{
#if defined(_WIN32)
	this->w_ser.n_bytes = 0;
	return (PurgeComm(this->w_ser.fd, PURGE_RXCLEAR) == FALSE);
#else
	return tcflush(ctx->s, TCIOFLUSH);
#endif

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
#if defined(_WIN32)
	DWORD n_bytes = 0;
	return (WriteFile(this->w_ser.fd, msg, msg_length, &n_bytes, NULL)) ? (ssize_t)n_bytes : -1;
#else
#if HAVE_DECL_TIOCM_RTS
	modbus_rtu_t *ctx_rtu = ctx->backend_data;
	if (ctx_rtu->rts != MODBUS_RTU_RTS_NONE) {
		ssize_t size;

		if (ctx->debug) {
			fprintf(stderr, "Sending request using RTS signal\n");
		}

		ctx_rtu->set_rts(ctx, ctx_rtu->rts == MODBUS_RTU_RTS_UP);
		usleep(ctx_rtu->rts_delay);

		size = write(ctx->s, req, req_length);

		usleep(ctx_rtu->onebyte_time * req_length + ctx_rtu->rts_delay);
		ctx_rtu->set_rts(ctx, ctx_rtu->rts != MODBUS_RTU_RTS_UP);

		return size;
	}
	else {
#endif
		return write(ctx->s, req, req_length);
#if HAVE_DECL_TIOCM_RTS
	}
#endif
#endif
}

int ModbusRTU::select(fd_set * rset, timeval * tv, int length_to_read)
{
	int s_rc;
#if defined(_WIN32)
	s_rc = win32_ser_select(&this->w_ser,
		length_to_read, tv);
	if (s_rc == 0) {
		errno = ETIMEDOUT;
		return -1;
	}

	if (s_rc < 0) {
		return -1;
	}
#else
	while ((s_rc = select(ctx->s + 1, rset, NULL, NULL, tv)) == -1) {
		if (errno == EINTR) {
			if (ctx->debug) {
				fprintf(stderr, "A non blocked signal was caught\n");
			}
			/* Necessary after an error */
			FD_ZERO(rset);
			FD_SET(ctx->s, rset);
		}
		else {
			return -1;
		}
	}

	if (s_rc == 0) {
		/* Timeout */
		errno = ETIMEDOUT;
		return -1;
	}
#endif

	return s_rc;
}

ssize_t ModbusRTU::recv(uint8_t * rsp, int rsp_length)
{
#if defined(_WIN32)
	return win32_ser_read(&this->w_ser, rsp, rsp_length);
#else
	return read(ctx->s, rsp, rsp_length);
#endif
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
		if (this->debug) {
			fprintf(stderr, "ERROR CRC received 0x%0X != CRC calculated 0x%0X\n",
				crc_received, crc_calculated);
		}

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
