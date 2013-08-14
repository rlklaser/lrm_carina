/*
 * serial.h
 *
 *  Created on: 26/09/2011
 *      Author: leandro
 */

#ifndef SERIAL_H_
#define SERIAL_H_

#include <termios.h>
#include <stdexcept>
#include <cstdio>

#define _MAKE_PRINTABLE_CMD_(cmd_out,cmd,n) for(int i=0, j=0; i<n; i++){ if( cmd[i] <= 0x1F || cmd[i] >= 0x7F ) { std::sprintf(&cmd_out[j],"\'0x%02x\'\0", cmd[i]); j+=6; } else { std::sprintf(&cmd_out[j],"\'%c\'\0", cmd[i]); j+=3; } }


/**
 * This class represents a communication model to send and receive data through a serial port.
 */
class Serial {
private:
	int fd; 					/**< The file descriptor used to identify the serial port. */
	char portname[80];			/**< This string holds the serial port name path. */
	struct termios settings;	/**< Used to define all port settings. */
	struct termios oldsettings;	/**< Holds the old serial port settings, before opening. */

public:
	Serial();
	virtual ~Serial();

	int openPort(const char *port_name);
	void closePort();
	bool isConnected();
	int configure(int baud, int databits, bool isCanonical, int parity, int startbit, int stopbit, int vmin, int vtime);
	int send(void *buff, int nbytes);
	int receive(void *buff, int nbytes);
	void printCurrentSettings();
};

class SerialException: public std::runtime_error {
public:
	SerialException(std::string msg): runtime_error(msg) {}
};

#endif /* SERIAL_H_ */


//	void serial::setInputModes(
//		bool IGNBRK, //Ignore BREAK condition on input.
//		bool BRKINT, //If IGNBRK is set, a BREAK is ignored. If it is not set but BRKINT is set, then a BREAK causes the input and output queues to be flushed, and if the terminal is the controlling terminal of a foreground process group, it will cause a SIGINT to be sent to this foreground process group. When neither IGNBRK nor BRKINT are set, a BREAK reads as a null byte ('\0'), except when PARMRK is set, in which case it reads as the sequence \377 \0 \0.
//		bool IGNPAR, //Ignore framing errors and parity errors.
//		bool PARMRK, //If IGNPAR is not set, prefix a character with a parity error or framing error with \377 \0. If neither IGNPAR nor PARMRK is set, read a character with a parity error or framing error as \0.
//		bool INPCK,  //Enable input parity checking.
//		bool ISTRIP, //Strip off eighth bit.
//		bool INLCR,  //Translate NL to CR on input.
//		bool IGNCR,  //Ignore carriage return on input.
//		bool ICRNL,  //Translate carriage return to newline on input (unless IGNCR is set).
//		bool IUCLC,  //(not in POSIX) Map uppercase characters to lowercase on input.
//		bool IXON,   //Enable XON/XOFF flow control on output.
//		bool IXANY,  //(XSI) Typing any character will restart stopped output. (The default is to allow just the START character to restart output.)
//		bool IXOFF,  //Enable XON/XOFF flow control on input.
//		bool IMAXBEL //(not in POSIX) Ring bell when input queue is full. Linux does not implement this bit, and acts as if it is always set.
//	);
//
//	void serial::setOutputModes(
//		bool OPOST, //Enable implementation-defined output processing.
//		bool OLCUC, //(not in POSIX) Map lowercase characters to uppercase on output.
//		bool ONLCR, //(XSI) Map NL to CR-NL on output.
//		bool OCRNL, //Map CR to NL on output.
//		bool ONOCR, //Don't output CR at column 0.
//		bool ONLRET,//Don't output CR.
//		bool OFILL, //Send fill characters for a delay, rather than using a timed delay.
//		bool NLDLY, //Newline delay mask. Values are NL0 and NL1. [requires _BSD_SOURCE or _SVID_SOURCE or _XOPEN_SOURCE]
//		bool CRDLY, //Carriage return delay mask. Values are CR0, CR1, CR2, or CR3. [requires _BSD_SOURCE or _SVID_SOURCE or _XOPEN_SOURCE]
//		bool TABDLY,//Horizontal tab delay mask. Values are TAB0, TAB1, TAB2, TAB3 (or XTABS). A value of TAB3, that is, XTABS, expands tabs to spaces (with tab stops every eight columns). [requires _BSD_SOURCE or _SVID_SOURCE or _XOPEN_SOURCE]
//		bool BSDLY, //Backspace delay mask. Values are BS0 or BS1. (Has never been implemented.) [requires _BSD_SOURCE or _SVID_SOURCE or _XOPEN_SOURCE]
//		bool VTDLY, //Vertical tab delay mask. Values are VT0 or VT1.
//		bool FFDLY  //Form feed delay mask. Values are FF0 or FF1. [requires _BSD_SOURCE or _SVID_SOURCE or _XOPEN_SOURCE]
//	);
//
//	void serial::setControlModes (
//		int  CBAUD,  //(not in POSIX) Baud speed mask (4+1 bits). [requires _BSD_SOURCE or _SVID_SOURCE]
//		bool CBAUDEX,//(not in POSIX) Extra baud speed mask (1 bit), included in CBAUD. [requires _BSD_SOURCE or _SVID_SOURCE] (POSIX says that the baud speed is stored in the termios structure without specifying where precisely, and provides cfgetispeed() and cfsetispeed() for getting at it. Some systems use bits selected by CBAUD in c_cflag, other systems use separate fields, e.g. sg_ispeed and sg_ospeed.)
//		int  CSIZE,  //Character size mask. Values are CS5, CS6, CS7, or CS8.
//		bool CSTOPB, //Set two stop bits, rather than one.
//		bool CREAD,  //Enable receiver.
//		bool PARENB, //Enable parity generation on output and parity checking for input.
//		bool PARODD, //Parity for input and output is odd.
//		bool HUPCL,  //Lower modem control lines after last process closes the device (hang up).
//		bool CLOCAL, //Ignore modem control lines.
//		bool CRTSCTS //(not in POSIX) Enable RTS/CTS (hardware) flow control. [requires _BSD_SOURCE or _SVID_SOURCE]
//	);
//
//	void serial::setLocalModes(
//		bool ISIG,   //When any of the characters INTR, QUIT, SUSP, or DSUSP are received, generate the corresponding signal.
//		bool ICANON, //Enable canonical mode. This enables the special characters EOF, EOL, EOL2, ERASE, KILL, LNEXT, REPRINT, STATUS, and WERASE, and buffers by lines.
//		bool XCASE,  //(not in POSIX; not supported under Linux) If ICANON is also set, terminal is uppercase only. Input is converted to lowercase, except for characters preceded by \. On output, uppercase characters are preceded by \ and lowercase characters are converted to uppercase.
//		bool ECHO,   //Echo input characters.
//		bool ECHOE,  //If ICANON is also set, the ERASE character erases the preceding input character, and WERASE erases the preceding word.
//		bool ECHOK,  //If ICANON is also set, the KILL character erases the current line.
//		bool ECHONL, //If ICANON is also set, echo the NL character even if ECHO is not set.
//		bool ECHOCTL,//(not in POSIX) If ECHO is also set, ASCII control signals other than TAB, NL, START, and STOP are echoed as ^X, where X is the character with ASCII code 0x40 greater than the control signal. For example, character 0x08 (BS) is echoed as ^H. [requires _BSD_SOURCE or _SVID_SOURCE]
//		bool ECHOPRT,//(not in POSIX) If ICANON and IECHO are also set, characters are printed as they are being erased. [requires _BSD_SOURCE or _SVID_SOURCE]
//		bool ECHOKE, //(not in POSIX) If ICANON is also set, KILL is echoed by erasing each character on the line, as specified by ECHOE and ECHOPRT. [requires _BSD_SOURCE or _SVID_SOURCE]
//		bool FLUSHO, //(not in POSIX; not supported under Linux) Output is being flushed. This flag is toggled by typing the DISCARD character. [requires _BSD_SOURCE or _SVID_SOURCE]
//		bool NOFLSH, //Disable flushing the input and output queues when generating the SIGINT, SIGQUIT and SIGSUSP signals.
//		bool TOSTOP, //Send the SIGTTOU signal to the process group of a background process which tries to write to its controlling terminal.
//		bool PENDIN, //(not in POSIX; not supported under Linux) All characters in the input queue are reprinted when the next character is read. (bash handles typeahead this way.) [requires _BSD_SOURCE or _SVID_SOURCE]
//		bool IEXTEN  //Enable implementation-defined input processing. This flag, as well as ICANON must be enabled for the special characters EOL2, LNEXT, REPRINT, WERASE to be interpreted, and for the IUCLC flag to be effective.
//	);
//
//	void serial::setControlChars(
//		bool VINTR,   //(003, ETX, Ctrl-C, or also 0177, DEL, rubout) Interrupt character. Send a SIGINT signal. Recognized when ISIG is set, and then not passed as input.
//		bool VQUIT,   //(034, FS, Ctrl-\) Quit character. Send SIGQUIT signal. Recognized when ISIG is set, and then not passed as input.
//		bool VERASE,  //(0177, DEL, rubout, or 010, BS, Ctrl-H, or also #) Erase character. This erases the previous not-yet-erased character, but does not erase past EOF or beginning-of-line. Recognized when ICANON is set, and then not passed as input.
//		bool VKILL,   //(025, NAK, Ctrl-U, or Ctrl-X, or also @) Kill character. This erases the input since the last EOF or beginning-of-line. Recognized when ICANON is set, and then not passed as input.
//		bool VEOF,    //(004, EOT, Ctrl-D) End-of-file character. More precisely: this character causes the pending tty buffer to be sent to the waiting user program without waiting for end-of-line. If it is the first character of the line, the read() in the user program returns 0, which signifies end-of-file. Recognized when ICANON is set, and then not passed as input.
//		int  VMIN,    //Minimum number of characters for non-canonical read.
//		bool VEOL,    //(0, NUL) Additional end-of-line character. Recognized when ICANON is set.
//		int  VTIME,   //Timeout in deciseconds for non-canonical read.
//		bool VEOL2,   //(not in POSIX; 0, NUL) Yet another end-of-line character. Recognized when ICANON is set.
//		bool VSWTCH,  //(not in POSIX; not supported under Linux; 0, NUL) Switch character. (Used by shl only.)
//		bool VSTART,  //(021, DC1, Ctrl-Q) Start character. Restarts output stopped by the Stop character. Recognized when IXON is set, and then not passed as input.
//		bool VSTOP,   //(023, DC3, Ctrl-S) Stop character. Stop output until Start character typed. Recognized when IXON is set, and then not passed as input.
//		bool VSUSP,   //(032, SUB, Ctrl-Z) Suspend character. Send SIGTSTP signal. Recognized when ISIG is set, and then not passed as input.
//		bool VDSUSP,  //(not in POSIX; not supported under Linux; 031, EM, Ctrl-Y) Delayed suspend character: send SIGTSTP signal when the character is read by the user program. Recognized when IEXTEN and ISIG are set, and the system supports job control, and then not passed as input.
//		bool VLNEXT,  //(not in POSIX; 026, SYN, Ctrl-V) Literal next. Quotes the next input character, depriving it of a possible special meaning. Recognized when IEXTEN is set, and then not passed as input.
//		bool VWERASE, //(not in POSIX; 027, ETB, Ctrl-W) Word erase. Recognized when ICANON and IEXTEN are set, and then not passed as input.
//		bool VREPRINT,//(not in POSIX; 022, DC2, Ctrl-R) Reprint unread characters. Recognized when ICANON and IEXTEN are set, and then not passed as input.
//		bool VDISCARD,//(not in POSIX; not supported under Linux; 017, SI, Ctrl-O) Toggle: start/stop discarding pending output. Recognized when IEXTEN is set, and then not passed as input.
//		bool VSTATUS  //(not in POSIX; not supported under Linux; status request: 024, DC4, Ctrl-T).
//	);
