// BasicTypes.cpp
// converts basic data to encoded strings
// Jianrui Du	2023.1.7

#include "BasicTypes.h"

#include <cmath>
#include <cstring>

namespace DataProtocol
{

int int2str(long long value, char *s, int radix)
{
    char *p, aux;
    unsigned long long v;
    size_t l;
 
    /* Generate the string representation, this method produces
     * an reversed string. */
    v = (value < 0) ? -value : value;
    p = s;
    do {
        *p++ = '0' + (v % radix); // 2 
        v /= radix; // 2
    } while (v);
    if (value < 0) *p++ = '-';
 
    /* Compute length and add null term. */
    l = p - s;
    *p = '\0';
 
    /* Reverse the string. */
    p--;
    while (s < p) {
        aux = *s;
        *s = *p;
        *p = aux;
        s++;
        p--;
    }
    return l;
}

std::string convertUnsignedInt(unsigned int _unsignedInt)
{
	std::string result;
	char cstring[65]{};  // 65 is enough for 64-bit systems
	int2str(_unsignedInt, cstring, 2);  // integer to binarty string
	result = cstring;

	return result;
}

unsigned int deconvertUnsignedInt(const std::string& _unsignedIntString)
{
	unsigned result{};

	unsigned binaryDigits = _unsignedIntString.length();
	for (unsigned i = 0; i < binaryDigits; ++i)
	{
		if (_unsignedIntString[i] == '1')
		{
			result += pow(2u, (binaryDigits - i - 1u));
		}
	}

	return result;
}

std::string convertSignedInt(int _signedInt)
{
	unsigned encodedInt;
	if (_signedInt >= 0)
	{
		encodedInt = 2 * _signedInt;
	}
	else
	{
		encodedInt = -2 * _signedInt - 1;
	}

	return convertUnsignedInt(encodedInt);
}

int deconvertSignedInt(const std::string& _signedIntString)
{
	int result{};

	unsigned zigZagResult = deconvertUnsignedInt(_signedIntString);
	if (zigZagResult % 2 == 0)
	{
		result = zigZagResult / 2;
	}
	else
	{
		result = -int(zigZagResult / 2) - 1;
	}

	return result;
}

std::string addMsbInt(const std::string& _origin)
{
	std::string result = _origin;

	unsigned digitLength = _origin.length();
	if (digitLength == 0)  // returns an empty string if the origin string is empty
	{
		result.clear();
		return result;
	}

	unsigned remainingLength = digitLength % 7;
	unsigned divisor = digitLength / 7;

	// stores the number of payloads in the final result
	unsigned payloadNumber;
	if (remainingLength != 0)
	{
		// fills with 0s before the highest bit if not 7's multiplier
		result.insert(0, 7 - remainingLength, '0');
		payloadNumber = divisor + 1;
	}
	else
	{
		// no needs to fill with 0s
		payloadNumber = divisor;
	}
	// adds MSB as 1 for the payloads before the last one
	for (int i = 0; i < payloadNumber - 1; ++i)
	{
		result.insert(8 * i, 1, '1');
	}
	// adds MSB as 0 for the last payload
	result.insert(8 * (payloadNumber - 1), 1, '0');

	return result;
}

std::string removeMsbInt(const std::string& _msbIntString)
{
	std::string result = _msbIntString;

	unsigned digitLength = _msbIntString.length();
	if (digitLength == 0 || digitLength % 8 != 0)
	{
		result.clear();
		return result;
	}

	unsigned payloadNumber = digitLength / 8;
	// replaces MSB 1s to 0s
	for (int i = 0; i < payloadNumber - 1; ++i)
	{
		result.replace(i * 8, 1, std::string("0"));
	}
	// deletes extra 0s produced by MSB
	for (int i = 0; i < payloadNumber; ++i)
	{
		result.erase(i * 7, 1);
	}
	// delete extra 0s in the higher bits if any
	while (result[0] == '0')
	{
		result.erase(0, 1);
	}
	if (result.empty())
	{
		result = "0";
	}

	return result;
}

std::string convertFloat(float _float)
{
	unsigned char ch[4]{};
	float number = _float;
	memcpy(ch, &number, 4);
	std::string result((char*)ch, 4);

	return result;
}

float deconvertFloat(const std::string& _floatString)
{
	float number{};
	memcpy(&number, _floatString.c_str(), 4);

	return number;
}

std::string convertDouble(double _double)
{
	unsigned char ch[8]{};
	double number = _double;
	memcpy(ch, &number, 8);
	std::string result((char*)ch, 8);

	return result;
}

double deconvertDouble(const std::string& _doubleString)
{
	double number{};
	memcpy(&number, _doubleString.c_str(), 8);

	return number;
}

char mapBinToHex(const char* _binaryString)
{
	switch (*_binaryString)
	{
	case '1': // 1xxx - 8~f
		switch (*(_binaryString + 1))
		{
		case '1': // 11xx - c~f
			switch (*(_binaryString + 2))
			{
			case '1': // 111x - e,f
				switch (*(_binaryString + 3))
				{
				case '1': // 1111 - f
					return 'f';
					break;
				case '0': // 1110 - e
					return 'e';
					break;
				default:
					break;
				}
				break;
			case '0': // 110x - c,d
				switch (*(_binaryString + 3))
				{
				case '1': // 1101 - d
					return 'd';
					break;
				case '0': // 1100 - c
					return 'c';
					break;
				default:
					break;
				}
				break;
			default:
				break;
			}
			break;
		case '0': // 10xx - 8~b
			switch (*(_binaryString + 2))
			{
			case '1': // 101x - a,b
				switch (*(_binaryString + 3))
				{
				case '1': // 1011 - b
					return 'b';
					break;
				case '0': // 1010 - a
					return 'a';
					break;
				default:
					break;
				}
				break;
			case '0': // 100x - 8,9
				switch (*(_binaryString + 3))
				{
				case '1': // 1001 - 9
					return '9';
					break;
				case '0': // 1000 - 8
					return '8';
					break;
				default:
					break;
				}
				break;
			default:
				break;
			}
			break;
		default:
			break;
		}
		break;
	case '0': // 0xxx - 0~7
		switch (*(_binaryString + 1))
		{
		case '1': // 01xx - 4~7
			switch (*(_binaryString + 2))
			{
			case '1': // 011x - 6,7
				switch (*(_binaryString + 3))
				{
				case '1': // 0111 - 7
					return '7';
					break;
				case '0': // 0110 - 6
					return '6';
					break;
				default:
					break;
				}
				break;
			case '0': // 010x - 4,5
				switch (*(_binaryString + 3))
				{
				case '1': // 0101 - 5
					return '5';
					break;
				case '0': // 0100 - 4
					return '4';
					break;
				default:
					break;
				}
				break;
			default:
				break;
			}
			break;
		case '0': // 00xx - 0~3
			switch (*(_binaryString + 2))
			{
			case '1': // 001x - 2,3
				switch (*(_binaryString + 3))
				{
				case '1': // 0011 - 3
					return '3';
					break;
				case '0': // 0010 - 2
					return '2';
					break;
				default:
					break;
				}
				break;
			case '0': // 000x - 0,1
				switch (*(_binaryString + 3))
				{
				case '1': // 0001 - 1
					return '1';
					break;
				case '0': // 0000 - 0
					return '0';
					break;
				default:
					break;
				}
				break;
			default:
				break;
			}
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
}

std::string mapHexToBin(const char _hexBit)
{
	switch (_hexBit)
	{
	case '0':
		return std::string("0000");
		break;
	case '1':
		return std::string("0001");
		break;
	case '2':
		return std::string("0010");
		break;
	case '3':
		return std::string("0011");
		break;
	case '4':
		return std::string("0100");
		break;
	case '5':
		return std::string("0101");
		break;
	case '6':
		return std::string("0110");
		break;
	case '7':
		return std::string("0111");
		break;
	case '8':
		return std::string("1000");
		break;
	case '9':
		return std::string("1001");
		break;
	case 'a':
		return std::string("1010");
		break;
	case 'b':
		return std::string("1011");
		break;
	case 'c':
		return std::string("1100");
		break;
	case 'd':
		return std::string("1101");
		break;
	case 'e':
		return std::string("1110");
		break;
	case 'f':
		return std::string("1111");
		break;
	default:
		break;
	}
}

std::string convertBinToHexInt(const std::string& _binaryIntString)
{
	unsigned binaryLength = _binaryIntString.length();
	if (binaryLength % 8 != 0)
	{
		std::string result;
		result.clear();
		return result;
	}
	unsigned divisor4 = binaryLength / 4;
	char* cResult = new char[divisor4 + 1]; // stores c string result temporarily
	cResult[divisor4] = '\0'; // needs a terminator to get a string with normal length and without garbled code
	const char* cString = _binaryIntString.c_str();
	// converts 4-bit binary string to a 1-bit hexadecimal a time
	for (int i = 0; i < divisor4; ++i)
	{
		cResult[i] = mapBinToHex(cString + i * 4);
	}
	std::string result(cResult, divisor4);

	delete[] cResult;
	return result;
}

std::string convertHexToBinInt(const std::string& _hexadecimalIntString)
{
	std::string result("");

	unsigned hexLength = _hexadecimalIntString.length();
	for (int i = 0; i < hexLength; ++i)
	{
		result += mapHexToBin(_hexadecimalIntString[i]);
	}

	return result;
}

std::string convertBinToByte(const std::string& _binaryIntString)
{
	unsigned binLength = _binaryIntString.length();
	if (binLength % 8 != 0)
	{
		std::string result;
		result.clear();
		return result;
	}

	auto* binArray = _binaryIntString.c_str();

	unsigned payloadNumber = binLength / 8;
	char* cResult = new char[payloadNumber + 1];
	for (int i = 0; i < payloadNumber; ++i)
	{
		cResult[i] = 0x00;
	}
	cResult[payloadNumber] = '\0';
	for (unsigned i = 0; i < payloadNumber; ++i)
	{
		if (binArray[i * 8] == '1') cResult[i] |= 0x80;
		if (binArray[i * 8 + 1] == '1') cResult[i] |= 0x40;
		if (binArray[i * 8 + 2] == '1') cResult[i] |= 0x20;
		if (binArray[i * 8 + 3] == '1') cResult[i] |= 0x10;
		if (binArray[i * 8 + 4] == '1') cResult[i] |= 0x08;
		if (binArray[i * 8 + 5] == '1') cResult[i] |= 0x04;
		if (binArray[i * 8 + 6] == '1') cResult[i] |= 0x02;
		if (binArray[i * 8 + 7] == '1') cResult[i] |= 0x01;
	}

	std::string result(cResult, payloadNumber);

	delete[] cResult;
	return result;
}

std::string convertByteToBin(const std::string& _byteIntString)
{
	unsigned byteLength = _byteIntString.length();

	char* cResult = new char[8 * byteLength + 1];
	cResult[byteLength] = '\0';
	for (unsigned i = 0; i < byteLength; ++i)
	{
		cResult[8 * i] = char((_byteIntString[i] & 0x80) >> 7) + '0';
		cResult[8 * i + 1] = char((_byteIntString[i] & 0x40) >> 6) + '0';
		cResult[8 * i + 2] = char((_byteIntString[i] & 0x20) >> 5) + '0';
		cResult[8 * i + 3] = char((_byteIntString[i] & 0x10) >> 4) + '0';
		cResult[8 * i + 4] = char((_byteIntString[i] & 0x08) >> 3) + '0';
		cResult[8 * i + 5] = char((_byteIntString[i] & 0x04) >> 2) + '0';
		cResult[8 * i + 6] = char((_byteIntString[i] & 0x02) >> 1) + '0';
		cResult[8 * i + 7] = char(_byteIntString[i] & 0x01) + '0';
	}

	std::string result(cResult, byteLength * 8);

	delete[] cResult;
	return result;
}

}
