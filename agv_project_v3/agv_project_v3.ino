#include <SPI.h>
#include "agv_can.h"
#include <Bounce2.h> //Include the debounce library to handle the buttons.


/* Variable declaration */
int8_t agvPosition = 0;
int error = 0;
int lastError = 0;

int turnMotorSpeed;
int moveMotorSpeed;

bool moveState; //AGV is in move (true) or stop (false)


/* -SPI Communication- */
const byte forwardPotAdress = B00000000;
const byte turnPotAdress = B00010000;

const byte rdForwardPot = B00001100;
const byte rdTurnPot = B00011100; 

/*Create an instance of AGV_CAN class */ 
AGV_CAN agv;


void setup()
{
  /* -SPI Com Initialization */
  pinMode(DIGIPOT_CS_PIN, OUTPUT);
  pinMode(MCP_CS_PIN, OUTPUT);

  /*Turn on MCP2515 module on SPI BUS (low means enable)*/
  digitalWrite(DIGIPOT_CS_PIN, HIGH);
  digitalWrite(MCP_CS_PIN, LOW);

  //pinMode(RELAY_PIN, OUTPUT);
  
  delay(100);  // delay to allow time to open the serial monitor window to check all is well

  #if SERIAL_ENABLE == 1
    Serial.begin(9600);
  #else
    //Serial.begin(9600);
  #endif

  /*Initialize MCP2515 module*/
  agv.can_init(); //SPI.begin already inside class

  /* Start ranging */ 
  agv.can_startrange();
}

void loop(){

  /* Get current position of the AGV */
   
  agvPosition = agv.getPosition();
  stopagv();
  
  /*Start move the AGV*/
    if (agv.tapeDetect == true && agv.obstacleDetected == false)
    {
        moveState = true;
        
        while(moveState != false)
        {
          agvPosition = agv.getPosition();
          switchSPI(MCP_CS_PIN, 1); //off CANbus
          switchSPI(DIGIPOT_CS_PIN, 0); //on digipot
          
          moveagv();
          
          switchSPI(DIGIPOT_CS_PIN, 1); //off digipot
          switchSPI(MCP_CS_PIN, 0); //on CANbus

          if(agv.obstacleDetected == true || agv.tapeDetect == false){
            moveState = false;
          }
        }   
    } else{
        stopagv();
      }

  }

//Moves the agv function
void moveagv(){
  if (moveState == false) {
    moveState = true;
  }
  
  setMotorSpeed(agvPosition, 1);

}

//Move the agv while there's no obstacle and not arrive yet at marker

// *************************************
void stopagv(){
  if (moveState == true) {
    moveState = false;

  }
 //Serial.print("Stop AGV\n");
  
  switchSPI(MCP_CS_PIN, 1); //off CANbus
  switchSPI(DIGIPOT_CS_PIN, 0); //on digipot
  setMotorSpeed(agvPosition, false);
  switchSPI(DIGIPOT_CS_PIN, 1); //off digipot
  switchSPI(MCP_CS_PIN, 0); //on CANbus
      

}

void emergency_Handler(){
  //Handles the state when emergency button is pressed
}

void printSerial(){
  Serial.print(agv.tapeDetect);
  Serial.print("\t");
  Serial.print(error);
  Serial.print("\t");
  Serial.print(moveMotorSpeed);
  Serial.print("\t"); 
  Serial.print(turnMotorSpeed);
  Serial.print("\t");
  Serial.print(agv.obstacleDetected);
  Serial.print("\t");
  Serial.print(moveState);
  Serial.println();  
}

void switchSPI(int pin, bool state){
  digitalWrite(pin, state);
}

int writeSpeed(int value, int oldValue, int potNum){
 /* int i;
  
  if(value > oldValue){
    for(i = oldValue;  i <= value; i++){
      SPI.transfer(potNum);
      SPI.transfer(i);
    }
  } 
  else if(value < oldValue){
   for(i = oldValue;  i >= value; i--){
    SPI.transfer(potNum);
    SPI.transfer(i);
   }
  }*/

  SPI.transfer(potNum);
  SPI.transfer(value);
}

int readSpeed(int rdPot){
  int rdValue;
  SPI.transfer(rdPot);

  rdValue = SPI.transfer(0);

  return rdValue;
}



void setMotorSpeed(int8_t linePosition, bool startMotor)
{
  
  if(startMotor == true)
  {
    error = (linePosition/8);
    int BaseSpeed;
    moveMotorSpeed = 105;
    moveMotorSpeed = moveMotorSpeed - (0.3*abs(error));
    
    //Slow down when turning

    BaseSpeed = analogRead(A0)/8;
    
    //PD control
    turnMotorSpeed = Kp * error + Kd * (error - lastError) + 43;
    lastError = error;
    
    //Maintain if reached maximum speed allowed;
    if(turnMotorSpeed > rightMaxSpeed)
    {
      turnMotorSpeed  = rightMaxSpeed;
    } else if (turnMotorSpeed < leftMaxSpeed)
    {
      turnMotorSpeed = leftMaxSpeed;
    }

    
    //Control digital potentiometer
    writeSpeed(turnMotorSpeed, readSpeed(rdTurnPot), turnPotAdress);  
    writeSpeed(moveMotorSpeed, readSpeed(rdForwardPot), forwardPotAdress);
    
   
  }
  
  else if(startMotor == false)
  {
   lastError = 0;
   moveMotorSpeed = 61;
   turnMotorSpeed = 61;
   writeSpeed(moveMotorSpeed, readSpeed(rdForwardPot), forwardPotAdress);
   writeSpeed(turnMotorSpeed, readSpeed(rdTurnPot), turnPotAdress);  
  }

  #if SERIAL_ENABLE == 1
    printSerial();  
  #else
    //printSerial();
  #endif
  
}
