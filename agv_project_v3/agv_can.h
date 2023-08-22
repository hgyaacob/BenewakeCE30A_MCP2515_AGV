/* 

Filename : agv_can.h
Author : Muhammad Yaacob bin Hasan Ganny
Date created : 26-10-2018

Dependencies
------------
Arduino serial library
MCP2515 CANbus Shield Library


*/


#ifndef _AGV_CAN_H
#define _AGV_CAN_H

#include "stdint.h"
#include "params.h"



class  AGV_CAN
{
	public:
	//CANBus Functions
	void can_init(void);
	void can_startrange(void); //start range and change depth and width on the fly (untuk lidar shj);
	void can_stoprange(void);
  void can_ccf_changemode(void);
  void can_ccf_changebaud(void);
  void can_lidarchangeconfig(void);
	
	//Read Functions
	int getDensity(uint16_t c);
	int8_t getPosition(void);
	
	//Public variables
	int8_t obstacleAngle; //obstacle angle if detected
	uint16_t obstacleDistance; //obstacle distance
	boolean obstacleDetected; //obstacle detected status 1 or 0
	boolean obstacleError; //lidar sensor status 1 error, 0 normal
  boolean lidarReady; //lidar ready status, on heartbeat

  boolean tapeDetect; //boolean if agv is detected by the sensor
	int8_t barPositionValue;//linefollowing value
	
	
/*___________________________________________ */
	private:
	void can_receive(void);
	
	
	//Math function
	int MultiplicationCombine(uint16_t x_high, uint16_t x_low);
	
	//Private variables
	uint16_t barRawValues;
	
	uint8_t len;
	uint8_t buf[8];
	uint8_t rxId;

};


#endif //_AGV_CAN_H
