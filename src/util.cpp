#include "util.hpp"


/* Table of CRC values for high-order byte */
static const uint8_t table_crc_hi[] = {
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
	0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
	0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
	0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
	0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
	0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
	0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
	0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
	0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
	0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
	0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
	0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
	0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
	0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
	0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
	0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
	0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
	0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
	0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
	0x80, 0x41, 0x00, 0xC1, 0x81, 0x40
};

/* Table of CRC values for low-order byte */
static const uint8_t table_crc_lo[] = {
	0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06,
	0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD,
	0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
	0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A,
	0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC, 0x14, 0xD4,
	0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
	0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3,
	0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4,
	0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
	0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29,
	0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED,
	0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
	0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60,
	0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67,
	0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
	0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68,
	0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA, 0xBE, 0x7E,
	0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
	0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71,
	0x70, 0xB0, 0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92,
	0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
	0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B,
	0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B,
	0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
	0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42,
	0x43, 0x83, 0x41, 0x81, 0x80, 0x40
};


uint16_t crc16(uint8_t *buffer, uint16_t buffer_length)
{
	uint8_t crc_hi = 0xFF; /* high CRC byte initialized */
	uint8_t crc_lo = 0xFF; /* low CRC byte initialized */
	unsigned int i; /* will index into CRC lookup */

					/* pass through message buffer */
	while (buffer_length--) {
		i = crc_hi ^ *buffer++; /* calculate the CRC  */
		crc_hi = crc_lo ^ table_crc_hi[i];
		crc_lo = table_crc_lo[i];
	}

	return (crc_hi << 8 | crc_lo);
}

#if defined(_WIN32)

/* This simple implementation is sort of a substitute of the select() call,
* working this way: the win32_ser_select() call tries to read some data from
* the serial port, setting the timeout as the select() call would. Data read is
* stored into the receive buffer, that is then consumed by the win32_ser_read()
* call.  So win32_ser_select() does both the event waiting and the reading,
* while win32_ser_read() only consumes the receive buffer.
*/

void win32_ser_init(struct win32_ser *ws)
{
	/* Clear everything */
	memset(ws, 0x00, sizeof(struct win32_ser));

	/* Set file handle to invalid */
	ws->fd = INVALID_HANDLE_VALUE;
}

/* FIXME Try to remove length_to_read -> max_len argument, only used by win32 */
int win32_ser_select(struct win32_ser *ws, int max_len,
	const struct timeval *tv)
{
	COMMTIMEOUTS comm_to;
	unsigned int msec = 0;

	/* Check if some data still in the buffer to be consumed */
	if (ws->n_bytes > 0) {
		return 1;
	}

	/* Setup timeouts like select() would do.
	FIXME Please someone on Windows can look at this?
	Does it possible to use WaitCommEvent?
	When tv is NULL, MAXDWORD isn't infinite!
	*/
	if (tv == NULL) {
		msec = MAXDWORD;
	}
	else {
		msec = tv->tv_sec * 1000 + tv->tv_usec / 1000;
		if (msec < 1)
			msec = 1;
	}

	comm_to.ReadIntervalTimeout = msec;
	comm_to.ReadTotalTimeoutMultiplier = 0;
	comm_to.ReadTotalTimeoutConstant = msec;
	comm_to.WriteTotalTimeoutMultiplier = 0;
	comm_to.WriteTotalTimeoutConstant = 1000;
	SetCommTimeouts(ws->fd, &comm_to);

	/* Read some bytes */
	if ((max_len > PY_BUF_SIZE) || (max_len < 0)) {
		max_len = PY_BUF_SIZE;
	}

	if (ReadFile(ws->fd, &ws->buf, max_len, &ws->n_bytes, NULL)) {
		/* Check if some bytes available */
		if (ws->n_bytes > 0) {
			/* Some bytes read */
			return 1;
		}
		else {
			/* Just timed out */
			return 0;
		}
	}
	else {
		/* Some kind of error */
		return -1;
	}
}

int win32_ser_read(struct win32_ser *ws, uint8_t *p_msg,
	unsigned int max_len)
{
	unsigned int n = ws->n_bytes;

	if (max_len < n) {
		n = max_len;
	}

	if (n > 0) {
		memcpy(p_msg, ws->buf, n);
	}

	ws->n_bytes -= n;

	return n;
}
#endif


int serial_connect(const std::string& device, const int baud, const uint8_t data_bit, const uint8_t stop_bit, const char parity, win32_ser *w_ser, DCB *old_dcb, std::ostringstream &err) {


#if defined(_WIN32)
	DCB dcb;
#else
	struct termios tios;
	speed_t speed;
	int flags;
#endif
    err << std::string("Opening ") << device << " at " << baud << " bauds (" << (int)parity << ", " << (int)data_bit << ", " << (int)stop_bit << ")";

#if defined(_WIN32)
	/* Some references here:
	* http://msdn.microsoft.com/en-us/library/aa450602.aspx
	*/
	win32_ser_init(w_ser);

	/* device should contain a string like "COMxx:" xx being a decimal
	* number */
	w_ser->fd = CreateFileA(device.c_str(),
		GENERIC_READ | GENERIC_WRITE,
		0,
		NULL,
		OPEN_EXISTING,
		0,
		NULL);

	/* Error checking */
	if (w_ser->fd == INVALID_HANDLE_VALUE) {
        err << "ERROR Can't open the device " << device << " (LastError " << (int)GetLastError() << ")";
		return -1;
	}

	/* Save params */
	old_dcb->DCBlength = sizeof(DCB);
	if (!GetCommState(w_ser->fd, old_dcb)) {
		
        err << "Problem getting com port configuration (LastError " << (int)GetLastError()<<")";
		CloseHandle(w_ser->fd);
		w_ser->fd = INVALID_HANDLE_VALUE;
		return -1;
	}

	/* Build new configuration (starting from current settings) */
	dcb = *old_dcb;

	/* Speed setting */
	switch (baud) {
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
		
	}

	/* Data bits */
	switch (data_bit) {
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
	if (stop_bit == 1)
		dcb.StopBits = ONESTOPBIT;
	else /* 2 */
		dcb.StopBits = TWOSTOPBITS;

	/* Parity */
	if (parity == 'N') {
		dcb.Parity = NOPARITY;
		dcb.fParity = FALSE;
	}
	else if (parity == 'E') {
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
	if (!SetCommState(w_ser->fd, &dcb)) {
		
			err << "Problem setting new com port configuration (LastError "<< (int)GetLastError()<<")";
		
		CloseHandle(w_ser->fd);
		w_ser->fd = INVALID_HANDLE_VALUE;
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

	s = open(device, flags);
	if (s == -1) {
		if (debug) {
			fprintf(stderr, "ERROR Can't open the device %s (%s)\n",
				device, strerror(errno));
		}
		return -1;
	}

	/* Save */
	tcgetattr(s, &old_tios);

	memset(&tios, 0, sizeof(struct termios));

	/* C_ISPEED     Input baud (new interface)
	C_OSPEED     Output baud (new interface)
	*/
	switch (baud) {
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
		if (debug) {
			fprintf(stderr,
				"WARNING Unknown baud rate %d for %s (B9600 used)\n",
				baud, device);
		}
	}

	/* Set the baud rate */
	if ((cfsetispeed(&tios, speed) < 0) ||
		(cfsetospeed(&tios, speed) < 0)) {
		close(s);
		s = -1;
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
	switch (data_bit) {
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
	if (stop_bit == 1)
		tios.c_cflag &= ~CSTOPB;
	else /* 2 */
		tios.c_cflag |= CSTOPB;

	/* PARENB       Enable parity bit
	PARODD       Use odd parity instead of even */
	if (parity == 'N') {
		/* None */
		tios.c_cflag &= ~PARENB;
	}
	else if (parity == 'E') {
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
	if (parity == 'N') {
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

	if (tcsetattr(s, TCSANOW, &tios) < 0) {
		close(s);
		s = -1;
		return -1;
	}
#endif
	return 0;

}