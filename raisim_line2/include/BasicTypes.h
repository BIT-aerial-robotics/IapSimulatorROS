// BasicTypes.h
// converts basic data to encoded strings
// Jianrui Du	2023.1.7

#pragma once

#include <string>

namespace DataProtocol
{

int int2str(long long value, char *s, int radix);

// converts 32-bit unsigned int number to a binary string in a big endian form
// @param _unsignedInt  unsigned number to convert
// @return the result string
std::string convertUnsignedInt(unsigned int _unsignedInt);

// restores an unsigned integer from a string representing an unsigned integer
// @param _unsignedIntString  the string representing an unsigned integer
// @return the result unsigned integer
unsigned int deconvertUnsignedInt(const std::string& _unsignedIntString);

// converts 32-bit signed int number to a binary string in a big endian form using ZigZag representation                
// ZigZag - Positive integers n are encoded as 2 * n (the even numbers), while negative integers -n are encoded as       
// 2 * n + 1 (the odd numbers)
// @param _signedInt  signed number to convert
// @return the result string
std::string convertSignedInt(int _signedInt);

// restores an integer from a string representing a signed integer
// @param _signedIntString  the string representing a signed integer
// @return the result signed integer
int deconvertSignedInt(const std::string& _signedIntString);

// adds the most significant bit to an integer string (unsigned or signed), adds 1 for not last payload, 0 for finale   
// adds a 0 or 1 every 7 bits, fills with 0s before the highest bit if not 7's multiplier
// @param _origin  origin string without MSB
// @return the result string
std::string addMsbInt(const std::string& _origin);

// removes the most significant bit to restore its origin integer string, see in "addMsbInt",                           
// returns an empty string if the input string is not 8's multiplier
// @param _msbIntString the integer string with MSB
// @return the origin string without MSB
std::string removeMsbInt(const std::string& _msbIntString);

// converts a float number to a string with 4 bytes
// @param _float  float number to convert
// @return the result string
std::string convertFloat(float _float);

// restores a float number from a string with 4 bytes
// @param _floatString  the string representing a float number
// @return the result float number
float deconvertFloat(const std::string& _floatString);

// converts a double number to a string with 8 bytes
// @param _double  double number to convert
// @return the result string
std::string convertDouble(double _double);

// restores a double number from a string with 8 bytes
// @param _doubleString  the string representing a double number
// @return the result double number
double deconvertDouble(const std::string& _doubleString);

// maps a group of 4-bit binary to a bit hexadecimal, e.g. 0000->0, 0101->5, 1011->b, 1111->f
// @param _binaryString  the 4-bit binary start char pointer using big endian
// @return the result hexadecimal char (0~f)
char mapBinToHex(const char* _binaryString);

// maps a 1-bit hexadecimal to a 4-bit binary, e.g. 0->0000, 5->0101, b->1011, f->1111
// @param _hexBit  the 1-bit hexadecimal
// @return the result binary string (0000~1111)
std::string mapHexToBin(const char _hexBit);

// shortens an integer string from binary to hexadecimal, returns an empty string if input doesn't meet the requirements
// @param _binaryIntString  the binary string to convert, whose length is a multiple of 8
// @return the result hexadecimal string
std::string convertBinToHexInt(const std::string& _binaryIntString);

// converts an integer string from hexadecimal to binary
// @param _hexadecimalIntString  the hexadecimal string to convert
// @return the result binary string
std::string convertHexToBinInt(const std::string& _hexadecimalIntString);

// shortens a binary integer string to bytes with each bit of a byte contains a binary bit, returns an empty string if  
// input doesn't meet the requirements
// @param _binaryIntString  the binary string to convert
// @return the result byte string in big endian
std::string convertBinToByte(const std::string& _binaryIntString);

// converts a byte integer string to a binary integer string
// @param _byteIntString  the byte string to convert
// @return the result binary string in big endian
std::string convertByteToBin(const std::string& _byteIntString);
}
