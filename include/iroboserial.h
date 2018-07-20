//
// Sample for beego(motor driver for MDR8)
// Serial Library(MDR8 Only)
//
// Copyright (C) 2018 Mobile Robot Research Co. LTD. Allright Reserved.
//
//
#pragma onece

#include <stdio.h>
#include <stdbool.h>
#include <fcntl.h>
#include <linux/serial.h>


typedef int					HANDLE;

// Functions
HANDLE SerialOpen(char *port, int baud);								// Open Serial Port
bool SerialClose(HANDLE hComm);											// Close Serial Port
bool SerialFlash(HANDLE hComm);											// Flash Serial Port

int SerialRead(HANDLE hComm, char *buf, int readlen, int timeout);		// Read Data from Serial Port (MDR8 ONly)
int SerialSend(HANDLE hComm, char *pdata, int length);					// Send Data to Serial Port (MDR8 Only)


