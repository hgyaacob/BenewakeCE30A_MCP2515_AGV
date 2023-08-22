
/* 

Filename : params.h
Author : Muhammad Yaacob bin Hasan Ganny
Date created : 15-12-2018

Description
------------
Main parameters define to separate from main coding files.
Change parameters value in here.
*/

#ifndef _PARAMS_H
#define _PARAMS_H

#define CE30A_ID 0x606      //CAN ID for LiDAR sensor CE30A
#define CE30A_ID_DATA 0x581
#define CE30A_ID_HEARTBEAT 0x589
#define CE30A_ID_CONFIG 0x607

#define LINE_FOLLOWER_TRANSMIT 0x605 //Line follower id to change node id
#define LINE_FOLLOWER_RECEIVE 0x585 //Line follower reply id

#define MCP_CS_PIN 2       // Chip Select pin for CAN Bus Module
#define DIGIPOT_CS_PIN 3   // Select pin for digitalPotentiometer
#define DATA_LENGTH 8       // Data length to receive/transmit can bus 

/* ---------------------------- */
/* Parameters for Line Following PID */

#define Kp  1.5 // experiment to determine this, start by something small that just makes your bot follow the line at a slow speed
#define Kd  0.2 // experiment to determine this, slowly increase the speeds and adjust this value. ( Note: Kp < Kd) 
#define rightMaxSpeed 127 // max speed
#define rightBaseSpeed 61  //70
#define leftMaxSpeed 0 // max speed
#define leftBaseSpeed 61 //52
//#define moveMotorSpeed 230 //forward speed

#define SERIAL_ENABLE 0 //enable serial COM


//#define moveMotorPin 10 //analog pins for right motor
//#define turnMotorPin 9 //analog pins for left motor

//#define RELAY_PIN 22 //pin for relay

#endif //_PARAMS_H
