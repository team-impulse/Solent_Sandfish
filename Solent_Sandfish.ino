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
 * Motors armed?
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
#define para_release_pin 10
#define strut_release_pin 5 //----TBC

//servo limits
#define servo_max_angle 150
#define servo_min_angle 30

//Parachute state
boolean para_armed = false;

//motor pins
const byte motor_pins[] = {14,15,3,4};//a1,a2,b1,b2
boolean motors_armed = true;

//serial baud rates
#define gps_serial_baud_rate 9600
#define computer_serial_baud_rate 38400

#define verbosity 1 //verbosity for Serial debug
#define current_software_version "1.4.2"//Major revision; minor revision; build
#define heading_tolerance 5

#define latitude_sign_positive false //Latitude sign
#define waypoint_acc_radius 5


//timers
#define radio_transmit_period 900 //time in milliseconds
#define sensor_reading_period radio_transmit_period-180

/*
 * Global variables:
 * TinyGPSPlus object--receives and decodes GPS data.
 * SensLib - responsible for communication with sensors and decoding of their data
 * RFMLib - responsible for communication with the radio module. Abstraction layer.
 * LSM303 - magnetometer
 * Servo - parachute release
 * Next waypoints--capacity for five waypoints
 * Length of next waypoints array
 * Radio transmission timer.
 * Sensor reading timer
 * Packet received boolean
 * Manual motor control status
 * MS5637 read already?
 * Time to read?
 */
TinyGPSPlus gps;//GPS object
SensLib sns;//sensor object
RFMLib radio = RFMLib(nss,dio0,dio5,rfm_rst);//radio object
LSM303 magnetometer;
Servo para_release;
double future_waypoints[10];//Long, lat; long,lat;long,lat...
byte future_waypoints_len;
uint32_t radio_transmit_timer;
uint32_t sensor_read_timer;
boolean pkt_rx = false;
byte manual[] = {255,255};//assign to 255 to disable override, otherwise setting as normal.

boolean ms5637_read = false;
boolean read_sens = false;

/* Misc declarations/definitions
 * Prototype for assemblePacket statement--references apparently confuse the Arduino/Processing compiler, which is peculiar.
  (This problem has been reported by other users with different code.
 */
 
 void assemblePacket(RFMLib::Packet &pkt);
 void decodePacket(RFMLib::Packet pkt);
 
 
 void setup(){
   pinMode(3,OUTPUT);
   pinMode(4,OUTPUT);
   //initialise servos--this is a matter of urgency.
  para_release.attach(para_release_pin);
  para_release.write(servo_min_angle);
  
  #if verbosity != 0
  Serial.begin(computer_serial_baud_rate);
  Serial.print("Team Impulse CanSat firmware v");
  Serial.print(current_software_version);
  Serial.print("  ('Solent Sandfish'). Operating with verbosity: ");
  Serial.println(verbosity);
  #endif
  Serial1.begin(gps_serial_baud_rate);
  SPI.begin();//Join the SPI bus
  byte my_config[5] = {0x64,0x74,0xFA,0xAC,0xCD};//radio settings
  radio.configure(my_config);//Radio configuration
  
  Wire.begin();//join the I2C bus
  sns.initialise();//initialise the sensors connected over I2C
  magnetometer.init();//start the magnetometer
  magnetometer.enableDefault();
  magnetometer.m_min = (LSM303::vector<int16_t>){ -1296,     +0,   -775};
  magnetometer.m_max = (LSM303::vector<int16_t>){    +0,   +452,     +0};//set compass calibration values
  
  //Motor initialisation
  for(byte i = 0;i<5;i++){
    pinMode(motor_pins[i],OUTPUT);
    digitalWrite(motor_pins[i],LOW);
  }   

   radio_transmit_timer = millis();
}

void loop(){
  #if verbosity > 3
  delay(5);//need this delay if printing everything to avoid crashing the serial monitor
  #endif
  magnetometer.read();
  if(millis()-sensor_read_timer >= sensor_reading_period){
    read_sens = true; 
  }
  if(read_sens){
   if(ms5637_read){
    sns.pollHYT271();
    Serial.println("Read hygro");
    Serial.println(sns.humidity);
    read_sens = false;
    sensor_read_timer = millis(); 
    ms5637_read = false;
   }
   else{
    sns.pollMS5637();
    ms5637_read = true; 
   }
  }
  if(radio.rfm_done) finishRFM();
  while(Serial1.available())gps.encode(Serial1.read());//Read in NMEA GPS data
  Serial.println("--=-=-=-=--");
  Serial.println((millis()-radio_transmit_timer));
  Serial.println(radio.rfm_status);
  if((millis()-radio_transmit_timer) > radio_transmit_period && radio.rfm_status != 1)
    transmitTime();
  if(future_waypoints_len==0 && manual[0]==255){//If there are no more waypoints to visit
    byte motor_control[] = {1,1};
    writeMotors(motor_control);//stop.
  }
  else if(manual[0]==255 && motors_armed){//assuming we have not visited every waypoint
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
  else if(motors_armed){
    writeMotors(manual);
    #if verbosity > 5
    Serial.println("Manual motor control. Writing:");
    Serial.print(manual[0]);
    Serial.print(" -- ");
    Serial.println(manual[1]);
    Serial.println();
    #endif
  }
  else{//motors not armed
   byte motor_cnt[] = {1,1};
   writeMotors(motor_cnt); 
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

void transmitTime(){
      Serial.println("time to tx");
      if(radio.rfm_status==2){
      RFMLib::Packet p;
     radio.endRX( p);
    }
    RFMLib::Packet p;
    assemblePacket(p);
    radio.beginTX(p); 
    attachInterrupt(7,RFMISR,RISING);
    radio_transmit_timer = millis();
    sensor_read_timer = millis();
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
     radio.rfm_done = false;
         attachInterrupt(7,RFMISR,RISING);
     break;
     case 2:
     #if verbosity != 0
     Serial.println("Ending reception.");
     #endif 
     RFMLib::Packet rx;
     radio.endRX(rx);
     decodePacket(rx);
     break;
   }

}


void RFMISR(){
 radio.rfm_done = true; 
}

void decodePacket(RFMLib::Packet pkt){
  byte i = 0;
  #if verbosity != 0
  Serial.println("Packet to be decoded: ");
  Serial.print("len = ");Serial.println(pkt.len);
       for(int k = 0;k<pkt.len;k++)Serial.println(pkt.data[k]);
  #endif

//  if(pkt.crc){
  while(i < pkt.len){

   switch(pkt.data[i]){
    case 0://general status OK - 1 byte
    #if verbosity > 0
     Serial.println("OK");
     #endif
     break;
    case 1://add waypoint-8 bytes
    #if verbosity > 0
    Serial.println("Waypoint add request:");
    #endif
      if(future_waypoints_len <5){
        byte base_index = future_waypoints_len *2;
        future_waypoints_len++;
        future_waypoints[base_index] = ((pkt.data[i+1]<<24)|(pkt.data[i+2]<<16)| (pkt.data[i+3]<<8)|pkt.data[i+4])/1000000;
        if(!latitude_sign_positive)
          future_waypoints[base_index] *= -1; 
        
        future_waypoints[base_index+1] = ((pkt.data[i+5]<<24)|(pkt.data[i+6]<<16)| (pkt.data[i+7]<<8)|pkt.data[i+8])/1000000;
        #if verbosity > 0
        Serial.println("Waypoint added.");
        Serial.print(future_waypoints[base_index]);
        Serial.print(", ");
        Serial.println(future_waypoints[base_index+1]);
        #endif
      }
      #if verbosity > 0
      else Serial.println("No more waypoint slots available now. Request ignored.");
      #endif
      i+=8;
    break;
    case 2://manual motor control-two bytes
      manual[0]=pkt.data[i+1];
      manual[1]=pkt.data[i+2];
      #if verbosity  > 0
      Serial.println("Manual motors:");
      Serial.print(manual[0]);
      Serial.print(" ");
      Serial.println(manual[1]);
      #endif
      i+=2;
    break;
    case 3:
      para_armed = true;
    break;
    case 4://arm the motors - 2 bytes
      if (pkt.data[i+1]==255)
      motors_armed =true;
      else motors_armed = false;
      i++;
      #if verbosity > 0
      Serial.println("Motors armed.");
      #endif
    break;
    case 5://arm parachute - 1 byte
      para_armed = true;
      #if verbosity > 0
      Serial.println("Armed parachute");
      #endif
    break;
    case 6://Drop all waypoints--2 bytes
      if (pkt.data[i+1]==255){
         future_waypoints_len=0;//delete all waypoints
         #if verbosity > 0     
         Serial.println("All waypoints deleted.");
        #endif
      }
    break;  
    case 7:
    if(pkt.data[i+1]==255 && pkt.data[i+2]==255){
      #if verbosity > 0
      Serial.println("Release:");
      #endif
      para_release.write(servo_max_angle);
    }
          i+=2;
    break;
   } 
   i++;
 // }
  }
  #if verbosity > 0
  /*  else
      Serial.println("CRC failed.");*/
  #endif
  
  
}

void assemblePacket(RFMLib::Packet &pkt){
  //round the pressure and shave a decimal place off to fit it into 16 bits
  //saving two bytes of valuable bandwidth
  int32_t pr_calc = sns.pressure;
  Serial.println(sns.humidity);
  byte round_byte = ((pr_calc % 10)>4)?1:0;
  pr_calc /= 10;
  pr_calc += (int16_t) round_byte;
    #if verbosity > 0
  Serial.println((int16_t)pr_calc);
  #endif
  uint16_t small_pressure = (uint16_t) pr_calc;
  pkt.data[0] = (byte)(small_pressure >> 8);//pressure
  pkt.data[1] = small_pressure & 255;
  
  //HYT271 temp
  pkt.data[2] = (byte)(sns.external_temperature>>8);
  pkt.data[3] = sns.external_temperature & 255;
  
  //MS5637 temp
  pkt.data[4] = (byte)(sns.internal_temperature>>8);
  pkt.data[5] = sns.internal_temperature & 255;
  
  //humidity
  pkt.data[6] = (byte)(sns.humidity >> 8);
  pkt.data[7] = sns.humidity & 255;
  
  //GPS latitude
  uint32_t raw_pos = (uint32_t)(gps.location.lat()*1000000);
  pkt.data[8] = (byte)(raw_pos >> 24);
  pkt.data[9] = (byte)(raw_pos >> 16);
  pkt.data[10] = (byte)(raw_pos >> 8);
  pkt.data[11] = raw_pos & 255;
  
  //and longitude
  raw_pos = (uint32_t)(gps.location.lng()*1000000);
  pkt.data[12] = (byte)(raw_pos >> 24);
  pkt.data[13] = (byte)(raw_pos >> 16);
  pkt.data[14] = (byte)(raw_pos >> 8);
  pkt.data[15] = raw_pos & 255;
  //nb lng and lat have fixed sign agreed beforehand.
  //heading
  raw_pos = (magnetometer.heading() * 100);
  pkt.data[16] = (byte)(raw_pos >> 24);
  pkt.data[17] = (byte)(raw_pos >> 16);
  pkt.data[18] = (byte)(raw_pos >> 8);
  pkt.data[19] = raw_pos & 255;
  
  pkt.data[20] = gps.hdop.value()/10;
  pkt.data[21] = future_waypoints_len;
  #if verbosity != 0
  Serial.print("GPS accuracy: ");
  Serial.println(gps.hdop.value());
  Serial.print("Lat");
  Serial.println(gps.location.lat(),6);
  Serial.print("Lng");
  Serial.println(gps.location.lng(),6);
  #endif
  //incremental counter


  //set length
  pkt.len = 22;
  
  //ir data append here
}
