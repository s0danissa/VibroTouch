/*
 * HexFT.cpp
 *
 *  Created on: 23.11.2020
 *  Author: Simon Haas
 */

#include "HexFT.h"

#include <errno.h>
#include <fcntl.h>
#include <iostream>
#include <string.h>
#include <unistd.h>

using namespace std;

HexFT::HexFT(string serialPort) {
	_serialPort = open(serialPort.c_str(), O_RDWR);

	if (_serialPort < 0) {
		// Here you can implement a custom error handling.
		cout << "Error " << errno << " from open: " << strerror(errno) << endl;
	}
}

HexFT::~HexFT() {
	close(_serialPort);
}

/*
 * \brief Reads one data package from HexFT sensor
 *
 * \return sensor data
 */
SensorData HexFT::ReadSensorData() {
	SensorData sensorData;

	read(_serialPort, reinterpret_cast<uint8_t*>(&sensorData),
			sizeof(sensorData));

	return sensorData;
}
