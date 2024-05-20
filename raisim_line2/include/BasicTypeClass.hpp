// BasicTypeClass.hpp
// a universal class for basic class packing operation
// Jianrui Du	2023.7.14

#pragma once

#include "BasicTypes.h"

#include <iostream>

namespace DataProtocol
{

enum class DataType
{
	Unsigned,    // 1
	Signed,      // 2
	Float,       // 3
	Double,      // 4
	Length,      // 5
	Submessage,  // 6
	NotDefined   // 0
};

// TODO: notes
class Data
{
public:
	Data(unsigned int _unsignedInteger, int _id)
		: type_(DataType::Unsigned), unsigned_(_unsignedInteger), id_(_id)
	{
		encodeUnsignedInteger();
	}
	Data(int _integer, int _id)
		: type_(DataType::Signed), signed_(_integer), id_(_id)
	{
		encodeSignedInteger();
	}
	Data(float _float, int _id)
		: type_(DataType::Float), float_(_float), id_(_id)
	{
		encodeFloat();
	}
	Data(double _double, int _id)
		: type_(DataType::Double), double_(_double), id_(_id)
	{
		encodeDouble();
	}
	Data(const std::string& _string, int _id)
		: type_(DataType::Length), string_(_string), id_(_id)
	{
		encodeString();
	}
	Data(const std::string& _encodedData)
		: type_(DataType::NotDefined), id_(-1), encodedData_(_encodedData)
	{
		decodeIdAndType();
		decodeData();
	}
	Data()
		: type_(DataType::NotDefined), id_(-1)
	{}

	virtual ~Data()
	{

	}

public:
	DataType getType() const { return type_; }
	int getId() const { return id_; }
	unsigned int getUnsignedInt() const { return unsigned_; }
	int getSignedInt() const { return signed_; }
	float getFloat() const { return float_; }
	double getDouble() const { return double_; }
	const std::string& getString() const { return string_; }

	const std::string& getEncodedDataString() const { return encodedData_; }

public: // friends
	// output the encoded data
	friend std::ostream& operator<<(std::ostream& os, const Data& _data)
	{
		std::string outputString = convertBinToHexInt(convertByteToBin(_data.encodedData_));
		os << outputString;

		return os;
	}

private:
	void encodeUnsignedInteger()
	{
		std::string data = addMsbInt(convertUnsignedInt(unsigned_));
		unsigned int key = 1 + 8 * id_;
		encodedData_ = convertBinToByte(addMsbInt(convertUnsignedInt(key)) + data);
	}

	void encodeSignedInteger()
	{
		std::string data = addMsbInt(convertSignedInt(signed_));
		unsigned int key = 2 + 8 * id_;
		encodedData_ = convertBinToByte(addMsbInt(convertUnsignedInt(key)) + data);
	}

	void encodeFloat()
	{
		std::string data = convertFloat(float_);
		unsigned int key = 3 + 8 * id_;
		encodedData_ = convertBinToByte(addMsbInt(convertUnsignedInt(key))) + data;
	}

	void encodeDouble()
	{
		std::string data = convertDouble(double_);
		unsigned int key = 4 + 8 * id_;
		encodedData_ = convertBinToByte(addMsbInt(convertUnsignedInt(key))) + data;
	}

	void encodeString()
	{
		std::string data = convertBinToByte(addMsbInt(convertUnsignedInt(string_.length()))) + string_;
		unsigned int key = 5 + 8 * id_;
		encodedData_ = convertBinToByte(addMsbInt(convertUnsignedInt(key))) + data;
	}

	void decodeIdAndType()
	{
		// gets the key bytes
		int keyLength{ 1 }; // the length of the key bytes
		for (int i = 0; ; ++i)
		{
			char counter = char((encodedData_[i] & 0x80) >> 7);
			if (counter == 0)
			{
				break;
			}
			++keyLength;
		}
		std::string keyBytes = encodedData_.substr(0, keyLength);
		
		unsigned int key = deconvertUnsignedInt(removeMsbInt(convertByteToBin(keyBytes)));
		id_ = key / 8;
		switch (key % 8)
		{
		case 1:
			type_ = DataType::Unsigned;
			break;
		case 2:
			type_ = DataType::Signed;
			break;
		case 3:
			type_ = DataType::Float;
			break;
		case 4:
			type_ = DataType::Double;
			break;
		case 5:
			type_ = DataType::Length;
			break;
		default:
			type_ = DataType::NotDefined;
			id_ = -1;
			break;
		}
	}
	
	void decodeData()
	{
		// gets the key bytes length
		int keyLength{ 1 }; // the length of the key bytes
		for (int i = 0; ; ++i)
		{
			char counter = char((encodedData_[i] & 0x80) >> 7);
			if (counter == 0)
			{
				break;
			}
			++keyLength;
		}
		
		std::string data = encodedData_.substr(keyLength);
		switch (type_)
		{
		case DataType::Unsigned:
			unsigned_ = deconvertUnsignedInt(removeMsbInt(convertByteToBin(data)));
			break;
		case DataType::Signed:
			signed_ = deconvertSignedInt(removeMsbInt(convertByteToBin(data)));
			break;
		case DataType::Float:
			float_ = deconvertFloat(data);
			break;
		case DataType::Double:
			double_ = deconvertDouble(data);
			break;
		case DataType::Length:
		{
			// gets string length first
			int lenLength{ 1 }; // the number of the len bytes
			for (int i = 0; i < data.length(); ++i)
			{
				char counter = char((data[i] & 0x80) >> 7);
				if (counter == 0)
				{
					break;
				}
				++lenLength;
			}
			std::string lengthByte = data.substr(0, lenLength);
			int contentLength = deconvertUnsignedInt(removeMsbInt(convertByteToBin(lengthByte)));
			string_ = data.substr(lenLength, contentLength);
			break;
		}
		default:
			break;
		}
	}

private:
	DataType type_;
	int id_;

	unsigned int unsigned_ = 0u;
	int signed_ = 0;
	float float_ = 0.0f;
	double double_ = 0.0;
	std::string string_ = "";

	std::string encodedData_;
};

}
