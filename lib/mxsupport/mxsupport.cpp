#include <mxsupport.h>

#ifdef ARDUINO
#include <Arduino.h>

void printObject(void *obj, int size)
{
    uint8_t *currentByte = reinterpret_cast<uint8_t *>(obj);
    int counter{0};
    while (size--)
    {
        //cout << std::dec << counter++ << ":";

        if (counter % 0x10 == 0)
        {
            if (counter)
            {
                Serial.print(F("\n"));
            }

            if (counter < 0x10)
            {
                Serial.print(0);
            }
            Serial.print(counter, HEX);
            Serial.print(F("  |  "));
            
        }

        if (int(*currentByte) < 0x10)
        {
            Serial.print(0);
        }
        Serial.print(int(*currentByte++), HEX);
        Serial.print(" ");
        counter++;
    }
    Serial.print("\n");
}

void writeToSerial(void *data, int size)
{
    char * chardata = reinterpret_cast<char*>(data);
    Serial.write(chardata,size);
}

void printHex(void *obj, int size)
{
    uint8_t *currentByte = reinterpret_cast<uint8_t *>(obj);
    while (size--)
    {
        if (int(*currentByte) < 0x10)
        {
            Serial.print(0);
        }
        Serial.print(int(*currentByte++), HEX);
        Serial.print(" ");
    }
}

bool isSerialAnswerY()
{
    while (!Serial.available())
    {
        ;
    }
    char answer = Serial.read();
    return (answer == 'y');
}
void setupTimer1(uint32_t milliseconds)
{
    cli();
    TCCR1A = 0; // set entire TCCR0A register to 0
    TCCR1B = 0; // same for TCCR0B
    TIMSK1 = 0;
    TCNT1 = 0; //initialize counter value to 0

    OCR1A = min(milliseconds*15625/1000, 0xffff);
    // turn on CTC mode
    TCCR1B |= (1 << WGM12);

    TCCR1B |= (1 << CS10) | (1 << CS12); // set prescaler at 1024

    // enable timer compare interrupt A
    TIMSK1 = (1 << OCIE1A);
    sei();
}
#else

#include <stdint.h>
#include <iostream>
using namespace std;

void printObject(void *obj, int size)
{
    uint8_t *currentByte = reinterpret_cast<uint8_t *>(obj);
    int counter{0};
    while (size--)
    {
        //cout << std::dec << counter++ << ":";

        if (counter % 0x10 == 0)
        {

            printf("\n%04X | ", counter);
        }
        printf("%02X ", int(*currentByte++));
        counter++;
    }
    printf("\n");
}

#endif

void copyBytewise(const void *source, void *target, int len)
{
    const char *sourceByte {reinterpret_cast<const char *>(source)};
    char *targetByte {reinterpret_cast<char *>(target)};
    while (len--)
    {
        *targetByte++ = *sourceByte++;
    }
}

