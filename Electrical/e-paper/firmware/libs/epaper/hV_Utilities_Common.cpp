//
// hV_Utilities_Common.cpp
// Library C++ code
// ----------------------------------
//
// Project Pervasive Displays Library Suite
// Based on highView technology
//
// Created by Rei Vilo, 01 Jun 2013
//
// Copyright (c) Rei Vilo, 2010-2025
// Licence Creative Commons Attribution-ShareAlike 4.0 International (CC BY-SA 4.0)
// For exclusive use with Pervasive Displays screens
//
// See hV_Utilities_Common.h for references
//
// Release 700: Refactored screen and board functions
// Release 803: Added types for string and frame-buffer
//

// Library header
#include "hV_Utilities_Common.h"
#include "stdarg.h"
#include "arduToPico.h"




void delay_ms(uint32_t ms) {
    sleep_ms(ms);
}

char bufferIn[128];
char bufferOut[128];

// Code
// Utilities

STRING_TYPE formatString(const char * format, ...)
{
    memset(&bufferOut, 0x00, sizeof(bufferOut));
    va_list args;
    va_start(args, format);
    vsnprintf(bufferOut, 127, format, args);
    va_end(args);

    return STRING_TYPE(bufferOut);
}

STRING_TYPE trimString(STRING_TYPE text)
{
    STRING_TYPE work = "";
    bool flag = true;
    char c;

    uint8_t index;
    uint8_t start, end;

    // Upwards from start
    index = 0;
    flag = true;
    while ((index < text.length()) and flag)
    {
        if ((text.at(index) != '\n') and (text.at(index) != '\r') and (text.at(index) != ' ') and (text.at(index) != '\t'))
        {
            flag = false;
            start = index;
        }
        index++;
    }

    // Downwards from end
    index = text.length();
    flag = true;
    while ((index > 0) and flag)
    {
        if ((text.at(index) != '\n') and (text.at(index) != '\r') and (text.at(index) != ' ') and (text.at(index) != '\t'))
        {
            flag = false;
            end = index - 1;
        }
        index--;
    }

    return text.substr(start, end - start);
}

long map(long x, long in_min, long in_max, long out_min, long out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int32_t cos32x100(int32_t degreesX100)
{
    int32_t i = 1;

    if (degreesX100 < 0)
    {
        i = -i;
        degreesX100 = -degreesX100;
    }

    degreesX100 %= 36000;

    if (degreesX100 > 9000)
    {
        if (degreesX100 < 18000)
        {
            i = -i;
            degreesX100 = 18000 - degreesX100;
        }
        else if (degreesX100 < 27000)
        {
            i = -i;
            degreesX100 = degreesX100 - 18000;
        }
        else
        {
            degreesX100 = 36000 - degreesX100;
        }
    }

    if (degreesX100 < 1000)
    {
        return i * map(degreesX100,    0, 1000, 100, 98);
    }
    else if (degreesX100 < 2000)
    {
        return i * map(degreesX100, 1000, 2000,  98, 93);
    }
    else if (degreesX100 < 3000)
    {
        return i * map(degreesX100, 2000, 3000,  93, 86);
    }
    else if (degreesX100 < 4000)
    {
        return i * map(degreesX100, 3000, 4000,  86, 76);
    }
    else if (degreesX100 < 5000)
    {
        return i * map(degreesX100, 4000, 5000,  76, 64);
    }
    else if (degreesX100 < 6000)
    {
        return i * map(degreesX100, 5000, 6000,  64, 50);
    }
    else if (degreesX100 < 7000)
    {
        return i * map(degreesX100, 6000, 7000,  50, 34);
    }
    else if (degreesX100 < 8000)
    {
        return i * map(degreesX100, 7000, 8000,  34, 17);
    }
    else
    {
        return i * map(degreesX100, 8000, 9000,  17,  0);
    }
}

int32_t sin32x100(int32_t degreesX100)
{
    return cos32x100(degreesX100 + 27000);
}

std::string utf2iso(const std::string& s)
{
    const size_t maxLen = s.length() + 1;
    char* bufferIn = new char[maxLen];
    std::strcpy(bufferIn, s.c_str());

    char bufferOut[maxLen] = {0};  // zero-initialize output

    uint8_t c;
    size_t outIndex = 0;

    for (size_t i = 0; i < std::strlen(bufferIn); ++i)
    {
        c = static_cast<uint8_t>(bufferIn[i]);

        if (c < 0x80)
        {
            bufferOut[outIndex++] = c;
        }
        else if (c == 0xC3)
        {
            bufferOut[outIndex++] = static_cast<uint8_t>(bufferIn[++i]) + 64;
        }
        else if (c == 0xC2)
        {
            bufferOut[outIndex++] = bufferIn[++i];
        }
        else if (c == 0xE2)
        {
            if ((static_cast<uint8_t>(bufferIn[i + 1]) == 0x82) &&
                (static_cast<uint8_t>(bufferIn[i + 2]) == 0xAC))
            {
                bufferOut[outIndex++] = 0x80;  // Euro sign (€)
                i += 2;
            }
        }
    }

    std::string result(bufferOut);
    delete[] bufferIn;
    return result;
}

uint16_t checkRange(uint16_t value, uint16_t valueMin, uint16_t valueMax)
{
    uint16_t localMin = std::min(valueMin, valueMax);
    uint16_t localMax = std::max(valueMin, valueMax);

    return std::min(std::max(localMin, value), localMax);
}

void setMinMax(uint16_t value, uint16_t & valueMin, uint16_t & valueMax)
{
    if (value < valueMin)
    {
        valueMin = value;
    }
    if (value > valueMax)
    {
        valueMax = value;
    }
}

uint32_t roundUp(uint32_t value, uint16_t modulo)
{
    uint32_t result = value / modulo;
    if ((value % modulo) > 0)
    {
        result++;
    }
    return result;
}
