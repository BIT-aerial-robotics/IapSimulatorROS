// DataPacket.hpp
// a universal class for packing and unpacking data
// Jianrui Du	2023.1.15

#pragma once

#include "BasicTypeClass.hpp"

#include <vector>

namespace DataProtocol
{

class DataPacket
{
public:
	DataPacket() {}
	DataPacket(const std::string& _encodedData)
		: encodedData_(_encodedData)
	{
		decode();
	}

	void addData(const Data& _data)
	{
		metaData_.push_back(_data);
		encodedData_ += _data.getEncodedDataString();
	}
	void addData(const DataPacket& _subDataPacket, int _id)
	{
		subDataPacket_.push_back(_subDataPacket);
		std::string encodedData = _subDataPacket.getEncodedDataString();
		std::string data = convertBinToByte(addMsbInt(convertUnsignedInt(encodedData.length()))) + encodedData;
		unsigned int key = 6 + 8 * _id;
		encodedData_ += convertBinToByte(addMsbInt(convertUnsignedInt(key))) + data;
	}
	void addData(const std::vector<Data>& _dataArray)
	{
		for (auto& data : _dataArray)
		{
			addData(data);
		}
	}
	//void addData(const std::vector<DataPacket>& _subDataPacketArray)
	//{
	//	subDataPacket_.insert(subDataPacket_.end(), _subDataPacketArray.begin(), _subDataPacketArray.end());
	//	for (auto& subDataPacket : _subDataPacketArray)
	//	{
	//		encodedData_ += subDataPacket.getEncodedDataString();
	//	}
	//}

	const std::string& getEncodedDataString() const { return encodedData_; }

	virtual ~DataPacket()
	{
		std::vector<Data>().swap(metaData_);
		std::vector<DataPacket>().swap(subDataPacket_);
	}

public:
	const std::vector<Data>& getAllMetaData() const { return metaData_; }
	const std::vector<DataPacket>& getAllSubDataPacket() const { return subDataPacket_; }

	size_t getMetaDataNumber() const { return metaData_.size(); }
	size_t getSubPacketNumber() const { return subDataPacket_.size(); }
	int getId() const { return id_; }

public: // friends
	// output the encoded data
	friend std::ostream& operator<<(std::ostream& os, const DataPacket& _data)
	{
		std::string outputString = convertBinToHexInt(convertByteToBin(_data.encodedData_));
		os << outputString;

		return os;
	}

private:
	void decode()
	{
		int counter{};
		unsigned dataLength = encodedData_.length();
		std::string remain(encodedData_);
		while (counter < dataLength)
		{
			int keyLength{ 1 }; // the length of the key bytes
			for (int i = 0; ; ++i)
			{
				char keyCounter = char((remain[i] & 0x80) >> 7);
				if (keyCounter == 0)
				{
					break;
				}
				++keyLength;
			}
			std::string keyByte = remain.substr(0, keyLength);
			remain = remain.substr(keyLength);
			counter += keyLength;
			unsigned int key = deconvertUnsignedInt(removeMsbInt(convertByteToBin(keyByte)));
			int id = key / 8;
			id_ = id;
			switch (key % 8)
			{
			case 1: // an unsigned integer
			{
				// gets the integer length first
				int intLength{ 1 }; // the number of the integer bytes
				for (int i = 0; i < remain.length(); ++i)
				{
					char msb = char((remain[i] & 0x80) >> 7);
					if (msb == 0)
					{
						break;
					}
					++intLength;
				}
				// decodes the integer and adds it to meta data vector
				std::string unsignedIntString = remain.substr(0, intLength);
				unsigned int unsignedInt = deconvertUnsignedInt(removeMsbInt(convertByteToBin(unsignedIntString)));
				addData(Data(unsignedInt, id));

				counter += intLength;
				remain = remain.substr(intLength);
			}
			    break;
			case 2: // a signed integer
			{
				// gets the integer length first
				int intLength{ 1 }; // the number of the integer bytes
				for (int i = 0; i < remain.length(); ++i)
				{
					char msb = char((remain[i] & 0x80) >> 7);
					if (msb == 0)
					{
						break;
					}
					++intLength;
				}
				// decodes the integer and adds it to meta data vector
				std::string signedIntString = remain.substr(0, intLength);
				int signedInt = deconvertSignedInt(removeMsbInt(convertByteToBin(signedIntString)));
				addData(Data(signedInt, id));

				counter += intLength;
				remain = remain.substr(intLength);
			}
				break;
			case 3: // a float
			{
				int floatLength = 4;
				std::string floatString = remain.substr(0, floatLength);
				float floatNumber = deconvertFloat(floatString);
				addData(Data(floatNumber, id));

				counter += floatLength;
				remain = remain.substr(floatLength);
			}
				break;
			case 4: // a double
			{
				int doubleLength = 8;
				std::string doubleString = remain.substr(0, doubleLength);
				double doubleNumber = deconvertDouble(doubleString);
				addData(Data(doubleNumber, id));

				counter += doubleLength;
				remain = remain.substr(doubleLength);
			}
				break;
			case 5: // a string
			{
				// gets string length first
				int lenLength{ 1 }; // the number of the len bytes
				for (int i = 0; i < remain.length(); ++i)
				{
					char msb = char((remain[i] & 0x80) >> 7);
					if (msb == 0)
					{
						break;
					}
					++lenLength;
				}
				// decodes the string and adds it to meta data vector
				std::string lengthByte = remain.substr(0, lenLength);
				int contentLength = deconvertUnsignedInt(removeMsbInt(convertByteToBin(lengthByte)));
				std::string content = remain.substr(lenLength, contentLength);
				addData(Data(content, id));

				counter += lenLength;
				counter += contentLength;
				remain = remain.substr(lenLength);
				remain = remain.substr(contentLength);
			}
				break;
			case 6: // a subDataPacket
			{
				// gets subDataPacket length first
				int subPacketLength{ 1 }; // the number of the subDataPacket bytes
				for (int i = 0; i < remain.length(); ++i)
				{
					char msb = char((remain[i] & 0x80) >> 7);
					if (msb == 0)
					{
						break;
					}
					++subPacketLength;
				}
				// decodes subPacketLength
				std::string lengthByte = remain.substr(0, subPacketLength);
				int contentLength = deconvertUnsignedInt(removeMsbInt(convertByteToBin(lengthByte)));
				std::string content = remain.substr(subPacketLength, contentLength);
				addData(DataPacket(content), id);

				counter += subPacketLength;
				counter += contentLength;
				remain = remain.substr(subPacketLength);
				remain = remain.substr(contentLength);
			}
			    break;
			default:
				break;
			}
		}
	}

private:
	std::vector<Data> metaData_;
	std::vector<DataPacket> subDataPacket_;

	std::string encodedData_;

	int id_ = 0;
};

}
