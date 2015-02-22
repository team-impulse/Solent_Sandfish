//21.2.15 -- Solent Sandfish
//Full implementation - CanSat firmware
//To be run on mission hardware as specified in interim reports (PCB Rev.2 )

/* Includes:
 * SPI--LoRa modules.
 * I2C--sensors.
 * TinyGPS++ - GPS data processing
 * SensLib--custom library to read sensor data
 * RFMLib--custom library to perform interface with radio modules
 * Servo--parachute release servos
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
 * Motor control pins--motorA and motorB are the two sides of the H-bridge driver.
 * Sensor pins; as implied.
 * Serial baud rate for the GPS
 * Serial baud rate for computer serial comms.
 * Verbosity - 0=no debug, 1=normal, 2=very verbose
 * Boolean sign for latitude (this software assumes longitude will always be positive)
 * Radius of location accuracy required at waypoints (m)
 * Period between transmissions (during which the radio will be receiving)
 * Period between sensor readings
 */
//LoRa pins:
#define nss 20
#define dio0 7
#define dio5 16
#define rfm_rst 21

//motor pins
const byte motor_pins[] = {3,4,14,15};//a1,a2,b1,b2

//serial baud rates
#define gps_serial_baud_rate 9600
#define computer_serial_baud_rate 38400

#define serial_debug_verbosity 1 //verbosity for Serial debug
#define current_software_version "0.01.02"//Major revision; minor revision; build

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
 * Next waypoints--capacity for five waypoints
 * Length of next waypoints array
 * Radio transmission timer.
 * Sensor reading timer
 * Packet incremental counter
 * Packet received boolean
 */
TinyGPSPlus gps;//GPS object
SensLib sns;//sensor object
RFMLib radio = RFMLib(nss,dio0,dio5,rfm_rst);//radio object
LSM303 magnetometer;
double future_waypoints[10];//Long, lat; long,lat;long,lat...
byte future_waypoints_len;
uint32_t radio_transmit_timer;
uint32_t sensor_read_timer;
byte pkt_inc=0;
boolean pkt_rx = false;

/* Misc declarations/definitions
 * Prototype for assemblePacket statement--references apparently confuse the Arduino/Processing compiler, which is peculiar.
  (This problem has been reported by other users with different code.
 */
 
 void assemblePacket(RFMLib::Packet &pkt);
 
 
 void setup(){
  #if serial_debug_verbosity != 0
  Serial.begin(computer_serial_baud_rate);
  Serial.print("Team Impulse CanSat firmware v");
  Serial.print(current_software_version);
  Serial.print("  ('Solent Sandfish'). Operating with verbosity: ");
  Serial.println(serial_debug_verbosity);
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
  while(Serial1.available())gps.encode(Serial1.read());//Read in NMEA GPS data
  sns.pollMS5637();
  sns.pollHYT271();
  magnetometer.read();//read everything
  
  if((millis()-radio_transmit_timer) > radio_transmit_period && radio.rfm_status != 1){
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
  if(future_waypoints_len==0){//If there are no more waypoints to visit
    byte motor_control[] = {1,1};
    writeMotors(motor_control);//stop.
  }
  else{//assuming we have not visited every waypoint
    double dist_to_next = TinyGPSPlus::distanceBetween(gps.location.lat(),gps.location.lng(),future_waypoints[0],future_waypoints[1]);//distance to the next waypoint in metres
    if(dist_to_next <= waypoint_acc_radius){//if we've arrived within 5 metres
      leftShiftWpt();//get rid of that waypoint so we can proceed to the next
    }
    else{//nav code here
      
      
      
    }
  }
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
}


void RFMISR(){
 radio.rfm_done = true; 
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
  Serial.print("GPS verbosity: ");
  Serial.println(gps.hdop.value());
  #endif
  //incremental counter
  pkt.data[21] = pkt_inc;
  pkt_inc++;
  
  //set length
  pkt.len = 20;
}
