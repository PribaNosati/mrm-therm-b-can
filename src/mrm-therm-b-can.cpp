#include "mrm-therm-b-can.h"
#include <mrm-robot.h>

/** Constructor
@param robot - robot containing this board
@param esp32CANBusSingleton - a single instance of CAN Bus common library for all CAN Bus peripherals.
@param hardwareSerial - Serial, Serial1, Serial2,... - an optional serial port, for example for Bluetooth communication
@param maxNumberOfBoards - maximum number of boards
*/
Mrm_therm_b_can::Mrm_therm_b_can(uint8_t maxNumberOfBoards) : 
	SensorBoard(1, "Thermo", maxNumberOfBoards, ID_MRM_THERM_B_CAN, 1) {
	readings = new std::vector<int16_t>(maxNumberOfBoards);
}

Mrm_therm_b_can::~Mrm_therm_b_can()
{
}

/** Add a mrm-therm-b-can sensor
@param deviceName - device's name
*/
void Mrm_therm_b_can::add(char * deviceName)
{
	uint16_t canIn, canOut;
	switch (nextFree) {
	case 0:
		canIn = CAN_ID_THERM_B_CAN0_IN;
		canOut = CAN_ID_THERM_B_CAN0_OUT;
		break;
	case 1:
		canIn = CAN_ID_THERM_B_CAN1_IN;
		canOut = CAN_ID_THERM_B_CAN1_OUT;
		break;
	case 2:
		canIn = CAN_ID_THERM_B_CAN2_IN;
		canOut = CAN_ID_THERM_B_CAN2_OUT;
		break;
	case 3:
		canIn = CAN_ID_THERM_B_CAN3_IN;
		canOut = CAN_ID_THERM_B_CAN3_OUT;
		break;
	case 4:
		canIn = CAN_ID_THERM_B_CAN4_IN;
		canOut = CAN_ID_THERM_B_CAN4_OUT;
		break;
	case 5:
		canIn = CAN_ID_THERM_B_CAN5_IN;
		canOut = CAN_ID_THERM_B_CAN5_OUT;
		break;
	case 6:
		canIn = CAN_ID_THERM_B_CAN6_IN;
		canOut = CAN_ID_THERM_B_CAN6_OUT;
		break;
	case 7:
		canIn = CAN_ID_THERM_B_CAN7_IN;
		canOut = CAN_ID_THERM_B_CAN7_OUT;
		break;
	default:
		sprintf(errorMessage, "Too many %s: %i.", _boardsName.c_str(), nextFree);
		return;
	}
	SensorBoard::add(deviceName, canIn, canOut);
}

/** Read CAN Bus message into local variables
@param canId - CAN Bus id
@param data - 8 bytes from CAN Bus message.
@param length - number of data bytes
@return - true if canId for this class
*/
bool Mrm_therm_b_can::messageDecode(CANMessage& message) {
	for (Device& device : devices)
		if (isForMe(message.id, device)) {
			if (!messageDecodeCommon(message, device)) {
				switch (message.data[0]) {
				case COMMAND_SENSORS_MEASURE_SENDING: {
					int16_t temp = (message.data[2] << 8) | message.data[1];
					(*readings)[device.number] = temp;
					device.lastReadingsMs = millis();
				}
				break;
				default:
					errorAdd(message, ERROR_COMMAND_UNKNOWN, false, true);
				}
			}
			return true;
		}
	return false;
}


/** Analog readings
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@return - analog value
*/
int16_t Mrm_therm_b_can::reading(uint8_t deviceNumber){
	if (deviceNumber >= nextFree) {
		sprintf(errorMessage, "%s %i doesn't exist.", _boardsName.c_str(), deviceNumber);
		return 0;
	}
	else
		if (started(deviceNumber))
			return (*readings)[deviceNumber];
		else
			return 0;
}

/** Print all readings in a line
*/
void Mrm_therm_b_can::readingsPrint() {
	print("Therm:");
	for (Device& device: devices)
		if (devices[device.number].alive)
			print(" %i", reading(device.number));
}


/** If sensor not started, start it and wait for 1. message
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@return - started or not
*/
bool Mrm_therm_b_can::started(uint8_t deviceNumber) {
	if (millis() - devices[deviceNumber].lastReadingsMs > MRM_THERM_B_CAN_INACTIVITY_ALLOWED_MS || devices[deviceNumber].lastReadingsMs == 0) {
		// print("Start mrm-therm-b-can-b%i \n\r", deviceNumber);
		for (uint8_t i = 0; i < 8; i++) { // 8 tries
			start(&devices[deviceNumber], 0);
			// Wait for 1. message.
			uint32_t startMs = millis();
			while (millis() - startMs < 50) {
				if (millis() - devices[deviceNumber].lastReadingsMs < 100) {
					// print("Thermo confirmed\n\r"); 
					return true;
				}
				delayMs(1);
			}
		}
		sprintf(errorMessage, "%s %i dead.", _boardsName.c_str(), deviceNumber);
		return false;
	}
	else
		return true;
}

/**Test
*/
void Mrm_therm_b_can::test()
{
	static uint64_t lastMs = 0;

	if (millis() - lastMs > 300) {
		uint8_t pass = 0;
		for (Device& device: devices){
			if (devices[device.number].alive) {
				if (pass++)
					print(" ");
				print("%i ", reading(device.number));
			}
		}
		lastMs = millis();
		if (pass)
			print("\n\r");
	}

	//stop();
}

