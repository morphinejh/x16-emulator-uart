/*!
\file     serialuartTL16C2550.cpp
\brief    Source file of the class serialuartTL16C2550. A serial UART for Commander X16 Emulator
\author   Jason Hill
\version  0.1
\date     September 18th of 2025, by Jason Hill
\modified none

This Serial library is used for communication of a physical serial device on the X16 emulator. Simulating
some aspects of the TL16C2550 for use on personal computers.

Description: This file will simulate many functions of the Texas Instruments TL16C2550 UART. Some functions
have been omtited due to arhcitectrual differences between hardware and OS control. Some FIFO functionas
are 'faked' for ease of use.

This file will emulate one serial port by attaching it to a physical port on the computer. Adjusting port
configurations as necessary based on register settings as they are programmed.It is not a complete
implementation, but will run most programs.

On hardware, UART1 is connected to an ESP32 on the X16 Serial/Wifi Card. An ESP8266 or ESP32 will
need to be attached to a COM port, with proper firmware, to get the same functionality.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE X CONSORTIUM BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

This is a licence-free software, it can be used by anyone who try to build a better world.
*/

//#define DEBUGUART
//#define VERBOSE

#include "serialuartTL16C2550.hpp"

serialuartTL16C2550::~serialuartTL16C2550(){
		this->serialPort.closeDevice();
}

int serialuartTL16C2550::addrwrite(unsigned char* value, int address){
	int retVal = 0;

#ifdef DEBUGUART	//Debug
		if(address != 0){
			printf("WriteAddress: $%02x\tValue: $%02x", address,(int)(*value));
			printf("\t-----\tIER: $%02x   IIR: $%02x   MCR: $%02x   LSR: $%02x   MSR: $%02x   Scratch: $%02x   LCR: $%02x   DLSB: $%02x   DMSB: $%02x", (int)IER, (int)IIR, (int)MCR, (int)LSR, (int)MSR, (int)scratchReg, (int)LCR, (int)DLSB, (int)DMSB);
			printf("   Divisor: %d\tBaud: %d\n", requestedDivisor, baudCalculator(requestedDivisor) );
		}//*/
#endif
	bool LCRconfigDirty=false;	//This triggers a closing and opening of the serial port.
	
	switch(address & 0x07){
		case 0: //Transmitter Holding Register
				if( LCR & 0x80 /*DLAB=1*/){
					DLSB = *value;
					retVal = DLSB;
				}
				else{				
					if( MCR & 0x10 /*Loop back mode*/ ){
						loopvalue=*value;
						retVal=1;
					} else {
						retVal=this->write(value);
					}
				}
				
			break;
			
		case 1: //Interrupt Enable Register
				if( LCR & 0x80 /*DLAB=1*/){
					DMSB = *value;
					retVal = DMSB;
					
				} else {
					IER = (0x0F & (*value) );
				}
			break;
			
		case 2: //FIFO Control Register
				FCR = 0xCF & (*value);
				retVal = FCR;
			break;
			
		case 3: //Line Control Register
				//Handle SerialPort reconfigurations
				if(LCR & 0x80){ //We are in Divisor Latch Mode currently
					if( ( (*value) & 0x80 ) == 0 ){
								//We are exiting Divisor Latch Mode; update baud rate
						requestedDivisor = DLSB;
						requestedDivisor |= ((uint16_t)(DMSB<<8));
						LCRconfigDirty=true;
					}
				}
				
				if ( (LCR&0x7F) != ( (*value)&0x7f ) ){
					LCRconfigDirty=true;
				}

				LCR=*value;
								
				if(LCRconfigDirty){
					reconfigureSerial();
				}
				retVal = LCR;
			break;
			
		case 4: //Modem Control Register
				if(*value != MCR){
					MCR =*value;
					reconfigureSerial();
				}
				retVal=MCR;
			break;
			
		case 5: //Line Status Register
			//LSR is read only
			retVal=-5;
			break;
			
		case 6: //Modem Status Register
			//MSR is read only
			retVal=-6;
			break;
			
		case 7: //Scratch Register
				scratchReg = *value;
				retVal=0;
			break;
			
		default:
			break;
	}
	return retVal;
}

int serialuartTL16C2550::addrread(unsigned char* value, int address){
	int retVal = -1;
	
#ifdef DEBUGUART	//Debug
		if(address != 0){
			printf("ReadAddress: $%02x", address);
			printf("\tFCR:$%02x\t\tIER: $%02x   IIR: $%02x   MCR: $%02x   LSR: $%02x   MSR: $%02x   Scratch: $%02x   LCR: $%02x   DLSB: $%02x   DMSB: $%02x", (int)FCR, (int)IER, (int)IIR, (int)MCR, (int)LSR, (int)MSR, (int)scratchReg, (int)LCR, (int)DLSB, (int)DMSB);
			printf("\n");
		}//*/
#endif
	int availCount = this->dataAvailable();
	// Pollings the modem status register [CTS/DSR] too quickly on some FTD232R or CH340 chips
	// cause a stall due to throughput limit. Some X16 programs hammer this to detect connection.
	// Given most Host OSes have HUGE buffers, let's assume we are always ready to send or recieve
	// data if the Port was opened succesfully.
	bool hasCTS = availCount < 14 ? true: false; //this->serialPort.isCTS();
	bool hasDSR = true; //this->serialPort.isDSR();
	switch(address & 0x07){
		case 0:
			if ( (LCR & 0x80) /*DLAB=1*/) {
				*value = DLSB;
				retVal = 1;
			}
			else {				
				if( MCR & 0x10  ){
					*value=loopvalue;
					retVal=loopvalue;
				} else {
					if( availCount > 0){
						retVal=this->read(value);
					}
					else {
						retVal = -1;
						*value = 0;
					}
				}
			}

			break;
		case 1:
			if( LCR & 0x80 /*DLAB=1*/){
				*value = DMSB;
				retVal = 1;
			}
			else{
				*value=IER;
				retVal=1;
			}
			break;
		case 2:
			if (FCR & 0x01){
				IIR |= 0xC0;
			}
			else{
				IIR &= 0x3F;
			}
				
			*value=IIR;
			retVal=1;
			break;
		case 3:
			*value=LCR;
			retVal=1;
			break;
		case 4:
			//Modem Control Register
			*value=MCR;
			retVal=1;
			break;
		case 5:
			//Line Status Register
			if(availCount > 0){
				LSR |= 0x01; //Data Ready
			}
			else {
				LSR &= 0xFE;
			}
			*value=LSR;
			retVal=1;
			break;
		case 6:
			//Modem Status Register - Update on read
			//if( (MCR & 0x01) && this->serialPort.isCTS() ) {//CTS
			if( hasCTS ) {
				MSR |=	0b00100000;
			}
			else {
				MSR &=	0b11011111;
			}

			//if( (MCR & 0x02) && this->serialPort.isDSR() ){//DSR
			if( hasDSR ){
				MSR |=	0b00010000;
			}
			else {
				MSR &=	0b11101111;
			}
			
			*value=MSR;
			retVal = MSR;
			break;
		case 7:
			//Scratch Register
			retVal = 1;
			*value = scratchReg;
			break;
		default:
			retVal=-2;
			break;
	}
	
	return retVal;
}

void serialuartTL16C2550::reconfigureSerial(){

	//This may get called a lot depending on the calling program and how it was written.
	//Every change to LCR or MCR constitutes a reconfiguration of the hardware settings.
	//if(!has_uart1)
		//return;

	//Serial Port Configuration enumerators
	SerialDataBits Databits;
	SerialStopBits Stopbits;
	SerialParity Parity;

	int wordLength=0;

	if(this->serialPort.isDeviceOpen()){
	    // Close the serial device in reset or re-initilization
		this->serialPort.closeDevice();
	}

	//Handle Databits
	wordLength = (LCR&0x03);
    switch( wordLength ){
		case 0:
				Databits = SERIAL_DATABITS_5;
			break;
		case 1:
				Databits = SERIAL_DATABITS_6;
			break;
		case 2:
				Databits = SERIAL_DATABITS_7;
			break;
		case 3:
				Databits = SERIAL_DATABITS_8;
			break;
		default:
				Databits = SERIAL_DATABITS_8;
			break;
	}
	
	//Handle Stop Bits
	if (LCR & 0x04){
		if(wordLength==5){
			Stopbits = SERIAL_STOPBITS_1_5;
		} else {
			Stopbits = SERIAL_STOPBITS_2;
		}
	} else {
		Stopbits = SERIAL_STOPBITS_1;
	}
    
    //Handle Parity
    if( LCR & 0x08 ){ //Bit 3 Enabled
		if( LCR & 0x20){ //Bit 5 Enabled
			if( LCR & 0x10 ){	//Bit 4 Enabled
				Parity = SERIAL_PARITY_SPACE;
			}
			else {				//Bit 4 Disabled	
				Parity = SERIAL_PARITY_MARK;
			}
		}
		else { //Bit 5 disabled
			if( LCR & 0x10 ){
				Parity = SERIAL_PARITY_EVEN;
			} else {
				Parity = SERIAL_PARITY_ODD;
			}			
		}
	}
	else {
		Parity = SERIAL_PARITY_NONE;
	}
    
    //Connect to real serial port
	//int errorOpening = serialPort.openDevice(path.c_str(), baudCalculator(requestedDivisor));

	int errorOpening = this->serialPort.openDevice(path.c_str(), baudCalculator(requestedDivisor),
		Databits,
		Parity,
		Stopbits);

	//DTR is alreasy ready when port is open
	this->serialPort.DTR(MCR & 0x01);
	//RTS is controlled from AFE bitof MCR (always ready if flow control is enabled
	this->serialPort.RTS( MCR & 0x20 );

    // If connection fails, return the error code and display a success message
    if (errorOpening!=1){
		 std::cerr<<errorOpening<<": Error opening: "<<path<<std::endl;
		 switch(errorOpening){
			case -1:
				std::cerr<<"device not found\n";
				break;
			case -2:
				std::cerr<<"error while opening the device\n";
				break;
			case -3:
				std::cerr<<"error while getting port parameters\n";
				break;
			case -4:
				std::cerr<<"Speed (Bauds) not recognized\n";
				break;
			case -5:
				std::cerr<<"error while writing port parameters\n";
				break;
			case -6:
				std::cerr<<"error while writing timeout parameters\n";
				break;
			case -7:
				std::cerr<<"Databits not recognized\n";
				break;
			case -8:
				std::cerr<<"Stopbits not recognized\n";
				break;
			case -9:
				std::cerr<<"Parity not recognized\n";
				break;
			default:
				std::cerr<<errorOpening<<std::endl;
				break;
		 }
		std::cerr<<"LCR: $"<<std::hex;
		std::cerr<<(int)LCR<<std::dec;
		std::cerr<<"\t     Baud: "<<baudCalculator(requestedDivisor);
		std::cerr<<"\tData bits: "<<5+Databits;
		std::cerr<<"\tParity: "<<descParity(static_cast<int>(Parity));
		std::cerr<<"\tStopbits: "<<descStopbits(static_cast<int>(Stopbits));
		std::cerr<<"\tDivisor: "<<requestedDivisor;
		std::cerr<<std::endl;
	 }
#ifdef VERBOSE
	else{
		std::cout<<"Reconfiguring: "<<this->path<<std::endl;
		std::cout<<"\t      Baud:\t"<<baudCalculator(requestedDivisor)<<std::endl;
		std::cout<<"\tData bits:\t"<<5+Databits<<std::endl;
		std::cout<<"\t   Parity:\t"<<descParity(static_cast<int>(Parity))<<std::endl;
		std::cout<<"\t Stopbits:\t"<<descStopbits(static_cast<int>(Stopbits))<<std::endl;
		std::cout<<"\t  Divisor:\t"<<requestedDivisor<<std::endl;
		std::cout<<std::endl;
	}
#endif
	return;
}

int serialuartTL16C2550::init(char* port) {
	
	//if(!has_uart1)
		//return -2;
	/*
	 * ROMTERM.PRG v1.27 reads three addresses at start-up
	 * $9fe1 == 0 [IER]
	 * $9fe3 == 0 [LCR]
	 * $9fe4 == 0 [MCR]
	 * $9fe7 //Scratch register writable check
	 * These must be zero or it doesn't think there is a card installed
	 */
	IER = 0;
	IIR = 0;
	FCR = 0;
	MCR = 0;
	LCR = 0; //1-stop, no parity, 8-bit word at start-up
	LSR = 0;
	MSR = 0;
	DLSB= 8; //Default 115200 at start-up
	DMSB= 0;
	requestedDivisor= 8;

    // Close the serial device in reset or initilization
	if(this->serialPort.isDeviceOpen()){
	    // Close the serial device in reset or initilization
		this->serialPort.closeDevice();
	}

    
    // Connection to serial port
#if defined (_WIN32) || defined( _WIN64)
	//Windows can't open COM ports greater than COM9 without this.
    path ="\\\\.\\\\";
#else
    path ="";
#endif

    path += (char*)port;
	std::cout<<"Attempting: "<<path<<", ";
    int errorOpening = this->serialPort.openDevice(path.c_str(), 115200);

    // If connection fails, return the error code and display a success message
    if (errorOpening!=1){
		 std::cout<<(int)errorOpening<<": ";
		 switch(errorOpening){
			case -1:
				std::cerr<<"device not found\n";
				break;
			case -2:
				std::cerr<<"error while opening the device\n";
				break;
			case -3:
				std::cerr<<"error while getting port parameters\n";
				break;
			case -4:
				std::cerr<<"Speed (Bauds) not recognized\n";
				break;
			case -5:
				std::cerr<<"error while writing port parameters\n";
				break;
			case -6:
				std::cerr<<"error while writing timeout parameters\n";
				break;
			case -7:
				std::cerr<<"Databits not recognized\n";
				break;
			case -8:
				std::cerr<<"Stopbits not recognized\n";
				break;
			case -9:
				std::cerr<<"Parity not recognized\n";
				break;
			default:
				std::cout<<": Error opening: "<<path<<std::endl;
				break;
		 }
		 return -1;
	 }
	 
    printf ("Successful connection to %s\n",path.c_str());

	this->serialPort.DTR(false);
	this->serialPort.RTS(false);
	
	return 0;
}

int serialuartTL16C2550::write(unsigned char* val){
	//Write one byte at a time.
	//FIFO possible, but wouldn't match speed of hardware.
	int retVal= this->serialPort.writeBytes(val,1);
	return retVal;
}

int serialuartTL16C2550::read(unsigned char* readVal){
	//Read one byte at a time... Only way to mimic hardware.
	return this->serialPort.readBytes(readVal,1,1000,1000);
}

int  serialuartTL16C2550::dataAvailable(){
	//Bytes available from computer UART
	return this->serialPort.available();
}

int serialuartTL16C2550::baudCalculator(uint16_t divisor){
	if(divisor > 0)
		return ( (OSC/divisor) / 16 );
	else
		return -1;
}

std::string serialuartTL16C2550::descParity(int parity){
  
    std::string results;
    
    switch(parity){
		case 0:
			results="No Parity";
			break;
		case 1:
			results="Even Parity";
			break;
		case 2:
			results="Odd Parity";
			break;
		case 3:
			results="Mark Parity";
			break;
		case 4:
			results="Space Parity";
			break;
		default:
			results="Error";
			break;
	}
	return results;
}

std::string serialuartTL16C2550::descStopbits(int stopbits){
    std::string results;
    
    switch(stopbits){
		case 0:
			results="1 bit";
			break;
		case 1:
			results="1.5 bits";
			break;
		case 2:
			results="2 bits";
			break;
		default:
			results="Error";
			break;
	}
    return results;
}
