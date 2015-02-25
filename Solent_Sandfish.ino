//21.2.15 -- Solent Sandfish
//Full implementation - CanSat firmware
//To be run on mission hardware as specified in interim reports (PCB Rev.2 )

/* Includes:
 * SPI--LoRa modules.
 * I2C--sensors.
 * TinyGPS++ - GPS data processing
 * SensLib--custom library to read sensor data
 * RFMLib--custom library to perform interface with radio modules
 * Servo--parachute release servo and strut release servo
 * LSM303--magnetometer library
 */
#include <TinyGPS++.h>
#include <SPI.h>
#include <Servo.h>
#include <Wire.h>
#include <sensor_library.h>
#include <RFM98W_library.h>
#include <LSM303.h>

/* Constants:
 * LoRa pins; as the name implies
 * The number of bytes implied by each command byte
 * Para release servo pin
 * Strut release servo pin
 * Boolean - parachute released
 * Boolean - parachute armed
 * Linear servo--smallest safe
 * Linear servo--largest safe
 * Motor control pins--motorA and motorB are the two sides of the H-bridge driver.
 * Sensor pins; as implied.
 * Serial baud rate for the GPS
 * Serial baud rate for computer serial comms.
 * Verbosity - 0=no debug, 1=normal, 2=very verbose
 * Boolean sign for latitude (this software assumes longitude will always be positive)
 * Radius of location accuracy required at waypoints (m)
 * Tolerance on heading (governed by induced magnetometer inaccuracy)
 * Period between transmissions (during which the radio will be receiving)
 * Period between sensor readings
 */
//LoRa pins:
#define nss 20
#define dio0 7
#define dio5 16
#define rfm_rst 21

//radio command lengths
const byte cmd_lengths[8] = {0,8,2,1,1,2,2,2};
//servo pins
#define para_release_pin 23
#define strut_release_pin 5 //----TBC

//servo limits
#define servo_max_angle 150
#define servo_min_angle 20

//Parachute state
boolean para_released = false;
boolean para_armed = false;

//motor pins
const byte motor_pins[] = {3,4,14,15};//a1,a2,b1,b2

//serial baud rates
#define gps_serial_baud_rate 9600
#define computer_serial_baud_rate 38400

#define verbosity 1 //verbosity for Serial debug
#define current_software_version "0.01.14"//Major revision; minor revision; build
#define heading_tolerance 5

#define latitude_sign_positive false //Latitude sign
#define waypoint_acc_radius 5



  

//timers
#define radio_transmit_period 700 //time in milliseconds
#define sensor_reading_period 100

/*
 * Global variables:
 * TinyGPSPlus object--receives and decodes GPS data.
 * SensLib - responsible for communication with sensors and decoding of their data
 * RFMLib - responsible for communication with the radio module. Abstraction layer.
 * LSM303 - magnetometer
 * Servo - parachute release
 * Servo - strut deployment
 * Next waypoints--capacity for five waypoints
 * Length of next waypoints array
 * Radio transmission timer.
 * Sensor reading timer
 * Packet incremental counter
 * Packet received boolean
 * Manual motor control status
 */
TinyGPSPlus gps;//GPS object
SensLib sns;//sensor object
RFMLib radio = RFMLib(nss,dio0,dio5,rfm_rst);//radio object
LSM303 magnetometer;
Servo para_release;
Servo strut_release;
double future_waypoints[10];//Long, lat; long,lat;long,lat...
byte future_waypoints_len;
uint32_t radio_transmit_timer;
uint32_t sensor_read_timer;
byte pkt_inc=0;
boolean pkt_rx = false;
byte manual[] = {1,1};//assign to 255 to disable override, otherwise setting as normal.

/* Misc declarations/definitions
 * Prototype for assemblePacket statement--references apparently confuse the Arduino/Processing compiler, which is peculiar.
  (This problem has been reported by other users with different code.
 */
 
 void assemblePacket(RFMLib::Packet &pkt);
 void decodePacket(RFMLib::Packet pkt);
 
 
 void setup(){
   //initialise servos--this is a matter of urgency.
  para_release.attach(para_release_pin);
  para_release.write(servo_min_angle);
  strut_release.attach(strut_release_pin);
  strut_release.write(servo_min_angle);//-----TBC
  
  #if verbosity != 0
  Serial.begin(computer_serial_baud_rate);
  Serial.print("Team Impulse CanSat firmware v");
  Serial.print(current_software_version);
  Serial.print("  ('Solent Sandfish'). Operating with verbosity: ");
  Serial.println(verbosity);
  #endif
  Serial1.begin(gps_serial_baud_rate);
  SPI.begin();//Join the SPI bus
  byte my_config[5] = {0x44,0x84,0x88,0xAC,0xCD};//radio settings
  radio.configure(my_config);//Radio configuration
  
  Wire.begin();//join the I2C bus
  sns.initialise();//initialise the sensors connected over I2C
  magnetometer.init();//start the magnetometer
  magnetometer.enableDefault();
  magnetometer.m_min = (LSM303::vector<int16_t>){-32767, -32767, -32767};
  magnetometer.m_max = (LSM303::vector<int16_t>){+32767, +32767, +32767};//set compass calibration values
  
  //Motor initialisation
  for(byte i = 0;i<5;i++){
    pinMode(motor_pins[i],OUTPUT);
    digitalWrite(motor_pins[i],LOW);
  }   
   radio_transmit_timer = millis();
}

void loop(){;
  if(radio.rfm_done) finishRFM();
  if(para_armed && para_released){
    para_release.write(servo_max_angle); 
  }
  while(Serial1.available())gps.encode(Serial1.read());//Read in NMEA GPS data
  
  if((millis()-radio_transmit_timer) > radio_transmit_period && radio.rfm_status != 1){
      sns.pollMS5637();
      sns.pollHYT271();
      magnetometer.read();//read everything

     //We're due to transmit--stop anything else
    if(radio.rfm_status == 2){
     #if verbosity != 0
     Serial.println("Aborting reception - transmit time.");
     #endif
     RFMLib::Packet rx;//throwaway reception packet--we do nothing with this data
     radio.endRX(rx);
     #if verbosity != 0
     Serial.println("Aborted reception.");
     #endif
    }
    RFMLib::Packet pkt;
    assemblePacket(pkt);  

    #if verbosity > 1
    Serial.println("About to transmit:");
    for(int i = 0;i<pkt.len;i++){
      Serial.print(pkt.data[i]);
      Serial.print("-");
    }
    #endif   
    radio.beginTX(pkt);
    #if verbosity != 0
    Serial.println("Began transmit.");
    #endif
    attachInterrupt(7,RFMISR,RISING);
    radio_transmit_timer = millis();
  }
  if(future_waypoints_len==0 && manual[0]==255){//If there are no more waypoints to visit
    byte motor_control[] = {1,1};
    writeMotors(motor_control);//stop.
  }
  else if(manual[0]==255){//assuming we have not visited every waypoint
    double dist_to_next = TinyGPSPlus::distanceBetween(gps.location.lat(),gps.location.lng(),future_waypoints[0],future_waypoints[1]);//distance to the next waypoint in metres
    if(dist_to_next <= waypoint_acc_radius){//if we've arrived within 5 metres
      leftShiftWpt();//get rid of that waypoint so we can proceed to the next
    }
    else{//nav code here
      double course_to_next = TinyGPSPlus::courseTo(gps.location.lat(),gps.location.lng(),future_waypoints[0],future_waypoints[1]);
      int16_t crs_change = detChange((uint16_t)magnetometer.heading(),course_to_next);
      byte m[2] = {2,2};
      if(crs_change > heading_tolerance){//if the heading error is outside tolerance
        m[0] = 1;
      }
      else if(crs_change < -heading_tolerance){
        m[1] =1;
      }
      else byte m[] = {2,2};
      writeMotors(m);
    }
  }
  else{
    writeMotors(manual);
    #if verbosity > 1
    Serial.println("Manual motor control. Writing:");
    Serial.print(manual[0]);
    Serial.print(" -- ");
    Serial.println(manual[1]);
    Serial.println();
    #endif
  }
}

int16_t detChange(uint16_t old, uint16_t changed){//course change mechanism
    changed += 360;
    int16_t diff = changed - old;
    if(diff > 180){
        diff = -1 * (360-diff);
    }
    return diff;
}

void leftShiftWpt(){
 for(int i = 2;i<(future_waypoints_len*2);i++){
    future_waypoints[i-2] = future_waypoints[i]; 
 }
 future_waypoints_len -=1;
}

void writeMotors(byte cont[2]){
  byte cnt_base = 0;
  for(byte i = 0;i<2;i++){
   switch(cont[i]){
     case 0://backwards
      digitalWrite(motor_pins[cnt_base],LOW);
      digitalWrite(motor_pins[cnt_base+1],HIGH);
      break;    
     case 1://stop
      digitalWrite(motor_pins[cnt_base],LOW);
      digitalWrite(motor_pins[cnt_base+1],LOW);
      break;
     case 2://forwards
      digitalWrite(motor_pins[cnt_base],HIGH);
      digitalWrite(motor_pins[cnt_base+1],LOW);
      break;
   }
  cnt_base+=2; 
  }
}

void finishRFM(){
  switch(radio.rfm_status){
   case 1:
     #if verbosity != 0
     Serial.println("Ending transmission.");
     #endif
     radio.endTX();
     #if verbosity != 0
     Serial.println("Beginning reception.");
     #endif
     radio.beginRX();
     break;
   case 2:
     #if verbosity != 0
     Serial.println("Ending reception.");
     #endif
     rxDone();
     break;
  } 
}

void rxDone(){
  #if verbosity != 0
  Serial.println("RX done function. Hooray.");
  #endif
  RFMLib::Packet rx;
  radio.endRX(rx);
  decodePacket(rx);
}


void RFMISR(){
 radio.rfm_done = true; 
}

void decodePacket(RFMLib::Packet pkt){
  byte i = 0;
  #if verbosity != 0
  Serial.println("Packet received: ");
  #endif
  #if verbosity > 2
  for(byte a = 0;a<pkt.len;a++){
   Serial.println(pkt.data[a]); 
  }
  #endif
  while(i<pkt.len){
     switch(pkt.data[i]){
       case 0:
         #if verbosity != 0
         Serial.print("  OK  ");
         #endif
       break;
       case 1:
         #if verbosity != 0
         Serial.print("  ADD waypoint  ");
         #endif
         byte coords[cmd_lengths[1]];
         i++;
         for(int z = 0;z<cmd_lengths[1];z++){
           coords[z] = pkt.data[i];
           i++;
         }
         addWpt(coords);
         break;
       case 2:
         i++;
         manual[0] = pkt.data[i];
         i++;
         manual[1] = pkt.data[i];
         i++;
       break;
       case 3:
         //arm parachute       
           i+=2;
           para_armed = true;
       break;
       case 4://arm strut
       
       break;
       case 5://release parachute
         i++;
         if(pkt.data[i] == pkt.data[i+1] && pkt.data[i]==255)
           para_released = true;
       break;
       case 6://release strut
       
       break;
       case 7://delete all waypoints and stop
         while(future_waypoints_len>0)
           leftShiftWpt();
         manual[0] = 1;
         manual[1] = 1;
       break;
     } 
  }
}

void addWpt(byte coords [8]){
  uint32_t coord_raw = (coords[0]<<24) | (coords[1]<<16) | (coords[2]<<8) | coords[3];
  double longitude = coord_raw / 1000000;
  coord_raw = (coords[4]<<24) | (coords[5]<<16) | (coords[6]<<8) | coords[7];
  double latitude = coord_raw / 1000000;
  if(!latitude_sign_positive)
    latitude *= -1;
  byte lat_index = future_waypoints_len * 2;
  future_waypoints[lat_index] = latitude;
  future_waypoints[lat_index+1] = longitude;
  future_waypoints_len++;
}

void assemblePacket(RFMLib::Packet &pkt){
  //round the pressure and shave a decimal place off
  sns.pressure = (sns.pressure / 10) + ((sns.pressure % 10)>4) ? 1 : 0 ;//this makes sure that the pressure fits into a 16 bit int
  //saving two bytes of valuable bandwidth
  pkt.data[0] = (byte)(sns.pressure >> 8);//pressure
  pkt.data[1] = sns.pressure & 255;
  
  //HYT271 temp
  pkt.data[2] = (byte)(sns.external_temperature>>8);
  pkt.data[3] = sns.external_temperature & 255;
  
  //MS5637 temp
  pkt.data[4] = (byte)(sns.internal_temperature>>8);
  pkt.data[5] = sns.internal_temperature & 255;
  
  //humidity
  pkt.data[6] = (byte)(sns.humidity >> 8);
  pkt.data[7] = sns.humidity & 255;
  
  //GPS longitude
  uint32_t raw_pos = gps.location.rawLat().billionths;
  pkt.data[8] = (byte)(raw_pos >> 24);
  pkt.data[9] = (byte)(raw_pos >> 16);
  pkt.data[10] = (byte)(raw_pos >> 8);
  pkt.data[11] = raw_pos & 255;
  
  //and longitude
  raw_pos = gps.location.rawLng().billionths;
  pkt.data[12] = (byte)(raw_pos >> 24);
  pkt.data[13] = (byte)(raw_pos >> 16);
  pkt.data[14] = (byte)(raw_pos >> 8);
  pkt.data[15] = raw_pos & 255;
  
  //heading
  raw_pos = (magnetometer.heading() * 100);
  pkt.data[16] = (byte)(raw_pos >> 24);
  pkt.data[17] = (byte)(raw_pos >> 16);
  pkt.data[18] = (byte)(raw_pos >> 8);
  pkt.data[19] = raw_pos & 255;
  
  pkt.data[20] = gps.hdop.value();
  #if verbosity != 0
  Serial.print("GPS accuracy: ");
  Serial.println(gps.hdop.value());
  #endif
  //incremental counter
  pkt.data[21] = pkt_inc;
  pkt_inc++;
  
  //set length
  pkt.len = 20;
}
