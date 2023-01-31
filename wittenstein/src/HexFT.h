/*
 * HexFT.h
 *
 *  Created on: 23.11.2020
 *  Author: Simon Haas
 */

#include <string>

/*
 * \brief sensor data class
 *
 * \note represents data format of HexFT
 */
class SensorData {
public:
	float fX;
	float fY;
	float fZ;
	float mX;
	float mY;
	float mZ;
	float temperature;
};

/*
 * \brief class for HexFT sensor
 */
class HexFT {
public:
	//Constructor
    HexFT(std::string serialPort);
	//Destructor
	~HexFT();

	SensorData ReadSensorData();
private:
	int _serialPort;
};

