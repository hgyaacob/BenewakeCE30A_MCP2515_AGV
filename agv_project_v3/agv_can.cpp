#include <mcp_can.h>
#include <mcp_can_dfs.h>

#include <can.h>
#include <mcp2515.h>


/* 

Filename : agv_can.cpp
Author : Muhammad Yaacob bin Hasan Ganny
Date created : 26-10-2018

Dependencies
------------
Arduino serial library
MCP2515 CANbus Shield Library


*/

#include "mcp_can.h"
#include "agv_can.h"
#include "params.h"


/* Frame format for CE30-A */
uint8_t lidarstart[5] = {0xc1, 0x00, 0x19, 0x11, 0x00}; /* start ranging ,3rd byte - width in cm, 4th byte - depth in dm */ 
uint8_t lidarstop [5] = {0xc0, 0x00, 0x00, 0x00, 0x00}; /* stop ranging */
uint8_t lidarconfig [8] = {0x81, 0x95, 0x58, 0x06, 0x76, 0x60, 0xE8, 0x03}; /* change data frame ID to a higher priority in canbus */

/* -------------------------- */

uint8_t ccf_changebaud [8] = {0x40, 0x20, 0x10, 0x00, 0x05, 0x00, 0x00, 0x00}; /* fifth byte 0x03 250kbps, 0x04 500kbps*, 0x05 1M*/ 
uint8_t ccf_outputmode [8] = {0x40, 0x30, 0x10, 0x00, 0x02, 0x00, 0x00, 0x00}; /* fifth byte 0x02 continous output, 0x01 change output mode */

/* -------------------------- */


MCP_CAN CAN(MCP_CS_PIN);

void AGV_CAN::can_init(void){
	while (CAN_OK != CAN.begin(CAN_1000KBPS, MCP_8MHz)){
		Serial.println("CAN BUS Shield Init fail");
		Serial.println("Init CAN BUS Shield again");
	}
	Serial.println("CAN BUS Shield init ok!");
}
	
void AGV_CAN::can_startrange(void){
	CAN.sendMsgBuf(CE30A_ID, 0, 5, lidarstart);
}

void AGV_CAN::can_stoprange(void){
	CAN.sendMsgBuf(CE30A_ID, 0, 5, lidarstop);
}

void AGV_CAN::can_receive(void){
	if(CAN_MSGAVAIL == CAN.checkReceive())
  {
    CAN.readMsgBuf(&len, buf);
    unsigned long canId = CAN.getCanId();
    int i = 0;

/* Enable to check all incoming messages */
      /*Serial.print("0x");
      Serial.print(canId,HEX);
      Serial.print("\t"); 
      for(i = 0; i < len; i++)     {
       Serial.print("0x");
      Serial.print(buf[i],BIN);
      Serial.print("\t");
      
      }
      Serial.println();*/
/* **************************************** */

/* Filter datas from CE30-A */
	if(canId == CE30A_ID_HEARTBEAT){
    lidarReady = 1;    
		if((buf[1] >> 5) & 0x01 == 0){
		  obstacleError = 0; //0 means normal
		} else {obstacleError = 1;}
	} else {lidarReady = 0; }

    if(canId == CE30A_ID_DATA){
		obstacleDistance = MultiplicationCombine(buf[1],buf[0]);
		obstacleAngle = buf[3];

		if(buf[7] == 1){
		  obstacleDetected = 1;
		}
		else{ obstacleDetected = 0; }
    }
/* **************************************** */
/* Filter only raw position line from line follower sensor */
    if(canId == LINE_FOLLOWER_RECEIVE){
      barRawValues = MultiplicationCombine(buf[1],buf[0]); 
      /* Serial.print(barRawValues, BIN);
        Serial.println();*/
      getDensity(barRawValues);
    }

    
  }
              
/* **************************************** */  
}

void AGV_CAN::can_ccf_changemode(void){
	CAN.sendMsgBuf(LINE_FOLLOWER_TRANSMIT, 0, 8, ccf_outputmode);
}

void AGV_CAN::can_ccf_changebaud(void){
  CAN.sendMsgBuf(LINE_FOLLOWER_TRANSMIT, 0, 8, ccf_changebaud);
}

void AGV_CAN::can_lidarchangeconfig(void){
	CAN.sendMsgBuf(CE30A_ID_CONFIG, 0, 8, lidarconfig);
}

int AGV_CAN::getDensity(uint16_t c){
	size_t bits_Counted = 0;
  
	for(size_t i = 0; i < 8 * sizeof c; ++i){
		if ((c & (1 << i)) == 0){
		  ++bits_Counted;
		}
	}

 if(bits_Counted > 0){
  tapeDetect = true;
 } else {tapeDetect = false;}
	return bits_Counted;
}

int8_t AGV_CAN::getPosition(void){
	can_receive();
	int16_t accumulator = 0;
	int16_t i = 0;

  for( i = 15; i > 7; i--){
    if ( ((~barRawValues >> i) & 0x01) == 1 ){
		accumulator += ((16 * (i - 7)) - 1);
    //accumulator += ((1 * (i - 7)) - 1);
	}
  }

  for ( i = 0; i < 8; i++ ){ //iterate positive side bits
    if ( ((~barRawValues >> i) & 0x01) == 1 ){
      accumulator += ((-16 * (8 - i)) + 1);
      //accumulator += ((-1* (8 - i)) + 1);
    }
  }

 

  if( getDensity(barRawValues) > 0){
    barPositionValue = accumulator / getDensity(barRawValues);
  }
  else{
    barPositionValue = 0;
  }

  return barPositionValue;
}
	
int AGV_CAN::MultiplicationCombine(uint16_t x_high, uint16_t x_low){
  int combined;
  combined = x_high;
  combined = combined*256; 
  combined |= x_low;
  return combined;
}
	
