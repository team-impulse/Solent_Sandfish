//21.2.15 -- Solant Sandfish, v0.1
//Full implementation - CanSat firmware
//To be run on mission hardware as specified in interim reports (PCB Rev.2 )

/* Includes:
 * SPI--LoRa modules.
 * I2C--sensors.
 * TinyGPS++ - GPS data processing
 * SensLib--custom library to read sensor data
 * RFMLib--custom library to perform interface with radio modules
 * Servo--parachute release servos
 */
#include <TinyGPS++.h>
#include <SPI.h>
#include <Servo.h>
#include <Wire.h>
#include <sensor_library.h>
#include <RFM98W_library.h>


/* Constants:
 * LoRa pins; as the name implies
 * Motor control pins--motorA and motorB are the two sides of the H-bridge driver.
 * Sensor pins; as implied.
 * Serial baud rate for the GPS
 * Serial baud rate for computer serial comms.
 * Verbosity - 0=no debug, 1=normal
 * Boolean sign for latitude (this software assumes longitude will always be positive)
 */
//LoRa pins:
#define nss 20
#define dio0 7
#define dio5 16
#define rfm_rst 21

//motor pins
#define m_a1 3 //motor A
#define m_a2 4
#define m_b1 14 //motor B
#define m_b2 15

//serial baud rates
#define gps_serial_baud_rate 9600
#define computer_serial_baud_rate 38400

#define latitude_sign_positive false //Latitude sign

#define serial_debug_verbosity 1 //verbosity for Serial debug
#define current_software_version "0.01.01"//Major revision; minor revision; build


/*
 * Global variables:
 * TinyGPSPlus object--receives and decodes GPS data.
 * SensLib - responsible for communication with sensors and decoding of their data
 * RFMLib - responsible for communication with the radio module. Abstraction layer.
 * Next waypoints--capacity for four waypoints
 */
TinyGPSPlus gps;//GPS object
SensLib sns;//sensor object
RFMLib radio = RFMLib(nss,dio0,dio5,rfm_rst);//radio object
uint32_t future_waypoints[10];//Long, lat; long,lat;long,lat...

void setup(){
  #if serial_debug_verbosity != 0
  Serial.begin(computer_serial_baud_rate);
  Serial.print("Team Impulse CanSat software version ");
  Serial.print(current_software_version);
  Serial.print("  Operating with verbosity: ");
  Serial.println(serial_debug_verbosity);
  #endif
  Serial1.begin(gps_serial_baud_rate);
  SPI.begin();//Join the SPI bus
  const byte my_config[5] = {0x44,0x84,0x88,0xAC,0xCD};//radio settings
  radio.configure(my_config);//Radio configuration
  
  Wire.begin();//join the I2C bus
  sns.initialise();//initialise the sensors connected over I2C
  
  //Motor initialisation
   pinMode(m_a1,OUTPUT);//set motor control pins to output
   pinMode(m_a2,OUTPUT);
   pinMode(m_b1,OUTPUT);
   pinMode(m_b2,OUTPUT);
   digitalWrite(m_a1,LOW);//set initial mode - stationary.
   digitalWrite(m_a2,LOW);
   digitalWrite(m_b1,LOW);
   digitalWrite(m_b2,LOW);   
}

void loop(){}

void RFMISR(){
 radio.rfm_done = true; 
}
