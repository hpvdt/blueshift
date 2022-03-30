#include <wiringSerial.h>
#include <stdio.h>

int openLine(char * line) {
	// Open serial line
	return serialOpen(line, 38400);
}

void sendData(int line, char type, char* data){
	if (line == -1) return; // Exit if no line set up
	unsigned char length = strlen(data) + 31;
	
	serialPrintf(line, "%c%c%s", type, length, data);
}

void requestData(int line, char type, char* data) {
	// Requests data oer serial, stores it to a char array that is passed
	// into the function and refernced.
	
	if (line == -1) return; // Return on no line
	
	// Send out request
	serialPutchar(line, type);
	
	// Get back length of incoming data
	int rxLength = (int) serialGetchar(line) - 31;
	
	// Read in data to the buffer passed into the function
	for (int i = 0; i < rxLength; i++) {
		data[i] = serialGetchar(line);
	}
	data[rxLength] = '\0'; // Terminate the string
}

void closeLine (int line) {
	if (line == -1) return;
	serialClose(line);
}
