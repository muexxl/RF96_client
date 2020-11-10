#ifndef MXSUPPORT_H
#define MXSUPPORT_H

#include <stdint.h>

void printObject(void * , int );

void printHex(void *, int);
bool isSerialAnswerY();
void setupTimer1(uint32_t);

/**
 * copy byte wise from source to target
 * 
 * @code
 * bytecopy(&source, &target, &sizeof(source))
 * 
 */
void copyBytewise(const void *source, void *target, int len);

/**
 * 
 * write chars to Serial 
 * 
 * */
void writeToSerial(void *, int );

#endif