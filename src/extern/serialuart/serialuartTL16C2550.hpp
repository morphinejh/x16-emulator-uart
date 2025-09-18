/*!
\file     serialuartTL16C2550.hpp
\brief    Header file of the class serialuartTL16C2550. A serial UART for Commander X16 Emulator
\author   Jason Hill
\version  0.1
\date     September 18th of 2025, by Jason Hill
\modified none

This Serial library is used for communication of a physical serial device on the X16 emulator. Simulating
some aspects of the TL16C2550 for use on personal computers.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE X CONSORTIUM BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

This is a licence-free software, it can be used by anyone who try to build a better world.
*/

#pragma once

#include <string>
#include <iostream>
#include <cstdint>
#include "serialib/serialib.h"

extern unsigned char scratchReg;
extern bool has_uart1;
extern bool has_uart2;

class serialuartTL16C2550{
	public:
	
		 //serialuartTL16C2550();
		~serialuartTL16C2550();
		int init(char*);
		int addrwrite(unsigned char *value, int address);
		int addrread(unsigned char* value, int address);
		
	private:
		//UART Registers
		unsigned char IIR, IER,
			FCR, MCR, LCR, LSR, MSR,
			scratchReg,
			DLSB, DMSB,
			loopvalue;
		uint16_t requestedDivisor;
		serialib serialPort;

		//Crystal Oscilator Speed on Texelec Serial/Wifi Card
		static const int OSC = 14745600;
		std::string path;		
		void reconfigureSerial();
		int write(unsigned char*);
		int read( unsigned char*);
		int dataAvailable();
		int baudCalculator(uint16_t);
		
		//For easy descriptions on errors
		std::string descParity(int);
		std::string descStopbits(int);
	
};
