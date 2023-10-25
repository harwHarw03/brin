#pragma once
#include <Arduino.h>
#include "Wire.h"
#include <Scheduler/Scheduler.h>
#include <Scheduler/Task.h>

#include "Adafruit_Sensor.h"
#include "MPU6050.h"
#include "tinySHT2x.h"
#include "TinyGPS.h"

#include <SoftwareSerial.h>
#include "EBYTE.h"
#include <JSON.h>

/*
WARNING: IF USING AN ESP32
DO NOT USE THE PIN NUMBERS PRINTED ON THE BOARD
YOU MUST USE THE ACTUAL GPIO NUMBER
*/

// =============== Pin Definitions

#define PIN_RX 16   // Serial2 RX (connect this to the EBYTE Tx pin)
#define PIN_TX 17   // Serial2 TX pin (connect this to the EBYTE Rx pin)

#define PIN_M0 4    // D4 on the board (possibly pin 24)
#define PIN_M1 22   // D2 on the board (possibly called pin 22)
#define PIN_AX 21   // D15 on the board (possibly called pin 21)

#define PIN_SOLENOID 26
#define PIN_POMPA 27
const int sensorPin = 25; // Pin analog sensor MPX
#define ANALOG_IN_PIN 12 // Battery   
static const int RXPin = 5, TXPin = 4; //gps

// ===============

#define BNO055_SENSOR true
#define SHT20_SENSOR false
#define Soil_SENSOR false
#define GPS_SENSOR false
#define MPX_SENSOR true
#define BATTERY false
#define EBYTE_COMMUNICATION false

#define window_size 20
int window[window_size] = {0};

#define window_size_B 10
int window_B[window_size_B] = {0};

// =============== Constant

// Battery Measurement
// Floats for resistor values in divider (in ohms) 
const float R1 = 5100.0;  // Resistor R1 (10K)
const float R2 = 1200.0;  // Resistor R2 (330 ohm)   
// Float for Reference Voltage 
float ref_voltage = 3.124;   
//Sensor Soil
const int AirValue = 570;   //you need to change this value that you had recorded in the air
const int WaterValue = 0;  //you need to change this value that you had recorded in the water
//MPX Sensor
const float ADC_mV = 4.8828125;       // convesion multiplier from Arduino ADC value to voltage in mV
const float SensorOffset = 336.0;     // in mV taken from datasheet
const float sensitivity = 4.413;      // in mV/mmH2O taken from datasheet
const float mmh2O_cmH2O = 10;         // divide by this figure to convert mmH2O to cmH2O
const float mmh2O_kpa = 0.00981;      // convesion multiplier from mmH2O to kPa
const float referenceVoltage = 5.0; // Tegangan referensi pada Arduino (dalam volt)
const float sensorSensitivity = 0.2; // Sensitivitas sensor (dalam volt/kPa)
const float waterDensity = 1000; // Kerapatan air (dalam kg/m^3)
const float gravity = 9.81; // Percepatan gravitasi (dalam m/s^2)
const float fluidHeight = 0.5; // Ketinggian fluida pada dasar selang (dalam meter)
const float calibrationOffset = 146.17;
const float hoseLength = 0.97;

static const uint32_t GPSBaud = 9600;

// =============== Data

//, Battery
float adc_voltage = 0.0; 
float in_voltage = 0.0;   
// Integer for ADC value
int adc_value = 0; 

//soil moisture
int intervals = (AirValue - WaterValue)/3;
int soilMoistureValue = 0;
char categori_soil;
//water level
float ketinggian_sebenarnya;
float h;
//gps
float flat = 0.0000000; 
float flon = 0.0000000;
unsigned long age;

bool newData = false;
unsigned long chars;
unsigned short sentences, failed;

int curr_sample = 0;

// =============== Sensors

tinySHT2x sht;
TinyGPS gps;
SoftwareSerial gpsSerial(RXPin, TXPin);
HardwareSerial SerialAT(2);

#define HEADER 250
#define TAIL 253 
StaticJsonDocument<300> json_data;
String newstr;
String serializedData;

struct DATA {
    String nodeUUID;
    String masterUUID;
    float Acc_X;// acx: [],
    float Acc_Y;//   acy: [],
    float Acc_Z;//   acz: [],
    float Gyro_X;//   gx: [],
    float Gyro_Y;//   gy: [],
    float Gyro_Z;//   gz: [],
    float latitude;//   lat: [],
    float longitude;//   lon: [],
    float roll;//   roll: [],
    float pitch;//   pitch: [],
    float humidity;//   hum: [],
    int soil_moisture;//   soil: [],
    float temperature;//   temp: [],
    float Battery;//   bat: [],
    float waterLevel;
};

DATA FinalData;

// =============== Functions

void push(int val);
int take_avg();
void adc_baterai_Lion(int val);
int baterai_avg();
void battery_update();
void gps_update();
void water_update();
String get_serialized(){return serializedData;}
void node_update();

// ---=--- Implementations

//schduelibg
class IMUPDATE : public TimingTask
{
    public :
        IMUPDATE(uint32_t _rate):rate(_rate){updateTime(millis());}
        virtual void run(uint32_t now){
            MPU6050GetData();
            UpdateQuaternion();
            Now = micros();
            sampleFreq = (1000000.0f / (Now - lastUpdate));
            lastUpdate = Now;
            MahonyAHRSupdateIMU(gxrs, gyrs, gzrs, axg, ayg, azg);
            GetRollPitchYaw();
            FinalData.Acc_X = AcX;
            FinalData.Acc_Y = AcY;
            FinalData.Acc_Z = AcZ;
            FinalData.Gyro_X = GyX;
            FinalData.Gyro_Y = GyY;
            FinalData.Gyro_Z = GyZ;
            FinalData.roll = roll;
            FinalData.pitch = pitch;
            tick(rate);
        }
    private:
        uint32_t rate;
};

class Blinker : public TimingTask
{
    public:
        Blinker(uint32_t _rate):rate(_rate){updateTime(millis());}
        bool led_on = true;
        virtual void run(uint32_t now){
            if (led_on){digitalWrite(13, LOW); led_on = false;}
            else{digitalWrite(13, HIGH); led_on = true;}
            tick(rate);
        } 
    private:
        uint32_t rate;
};

class WATERLEVEL : public TimingTask
{
    public:
        WATERLEVEL(uint32_t _rate):rate(_rate){updateTime(millis());}

        virtual void run(uint32_t now){
            water_update();
            tick(rate);
        } 
    private:
        uint32_t rate;
};

class SUPPORT : public TimingTask
{
    public:
        SUPPORT(uint32_t _rate):rate(_rate){updateTime(millis());}
        bool led_on = true;
        virtual void run(uint32_t now){
            FinalData.humidity = sht.getHumidity();
            FinalData.temperature = sht.getTemperature();
            FinalData.soil_moisture = analogRead(33);
            // anemo_update();
            // water_level_update();
            tick(rate);
        } 
    private:
        uint32_t rate;
};

class SUPPORT2 : public TimingTask
{
    public:
        SUPPORT2(uint32_t _rate):rate(_rate){updateTime(millis());}

        virtual void run(uint32_t now){
            gps_update();
            tick(rate);
        } 
    private:
        uint32_t rate;
};

class Logger : public TimingTask
{
    public:
        Logger(uint32_t _rate):rate(_rate){updateTime(millis());}

        virtual void run(uint32_t now){
            node_update();
            Serial.println(serializedData);
            tick(rate);
        } 
    private:
        uint32_t rate;
};

void node_update()
{
/*JSON*/
    json_data["acx"] = FinalData.Acc_X;
    json_data["acy"] = FinalData.Acc_Y;
    json_data["acz"] = FinalData.Acc_Z;
    json_data["gx"] = FinalData.Gyro_X;
    json_data["gy"] = FinalData.Gyro_Y;
    json_data["gz"] = FinalData.Gyro_Z;
    json_data["lat"] = FinalData.latitude;
    json_data["lon"] = FinalData.longitude;
    json_data["roll"] = FinalData.roll;
    json_data["pitch"] = FinalData.pitch;
    json_data["hum"] = FinalData.humidity;
    json_data["soil"] = FinalData.soil_moisture;
    json_data["temp"] = FinalData.temperature;
    json_data["bat"] = FinalData.Battery;

    String messg;
    serializeJson(json_data, serializedData);
    // messg += '@';
    // Serial.println(messg);
}

void sensor_init() 
{
    json_data["idmaster"] = "a40ba80e-7228-11ee-b962-0242ac120002";
    json_data["idnode"] = "a40ba80e-7228-11ee-b962-0242ac120002";
    // Init
    // Serial.begin(9600);
    Serial.println("Start");
    imu_init();
    pinMode(13, OUTPUT);

    // Blinker blinking(750);
    // IMUPDATE imu_update(50);
    // WATERLEVEL water_level(150);
    // SUPPORT support_sensor(100);
    // SUPPORT2 support_sensor2(500);
    // Logger logger(100);
    // // Task *tasks[] = {&imu_update, &support_sensor, &support_sensor2, &water_level, &logger};
    // Task *tasks[] = {&imu_update, &logger};
    // Scheduler scheduler(tasks, NUM_TASKS(tasks));
    // while(1){scheduler.runTasks();}
}

void push(int val){
  int i=0;
  for (i=0;i<window_size;i++){
    window[i-1]=window[i];
  }
  window[window_size-1]=val;
}

int take_avg(){
  long sum=0;
  int i=0;
  for (i=0;i<window_size;i++){
    sum+=window[i];
  }
  return sum/window_size;
}

void adc_baterai_Lion(int val){
  int j=0;
  for (j=0;j<window_size_B;j++){
    window_B[j-1]=window_B[j];
  }
  window_B[window_size_B-1]=val;
}

int baterai_avg(){
  long jumlah=0;
  int i=0;
  for (i=0;i<window_size_B;i++){
    jumlah+=window_B[i];
  }
  return jumlah/window_size_B;
}

void battery_update()
{
    //Battery======================================================================================================
    adc_value = analogRead(ANALOG_IN_PIN);  
    adc_baterai_Lion(adc_value);
    // Determine voltage at ADC input    
    adc_voltage  = (baterai_avg() * ref_voltage) / 4025;         
    // Calculate voltage at divider input    
    FinalData.Battery = adc_voltage / (R2/(R1+R2)); 
}

void gps_update()
{
    while (gpsSerial.available())
    {
      char c = gpsSerial.read();
      // Serial.write(c); // uncomment this line if you want to see the GPS data flowing
      if (gps.encode(c)) // Did a new valid sentence come in?
        newData = true;
    }
    if (newData)
    {
        gps.f_get_position(&FinalData.latitude, &FinalData.latitude, &age);
    }
}

void water_update()
{
    curr_sample = analogRead(25);
    DelayMicros(300);
    push(curr_sample);
    h = ((take_avg()-48.0403278832)/7.6673168807);
    float sensorValue = ((take_avg()- SensorOffset) * ADC_mV) / sensitivity / mmh2O_cmH2O; 
    
    float voltage = take_avg() * (referenceVoltage / 1023.0); // Konversi nilai analog menjadi tegangan (volt)
    float pressure = (voltage - (referenceVoltage / 2)) / sensorSensitivity; // Konversi tegangan menjadi tekanan (kPa)
    float waterLevel = (pressure * 1000) / (waterDensity * gravity) + fluidHeight - (calibrationOffset / 100.0) - hoseLength; // Konversi tekanan menjadi tinggi air (meter) dengan offset kalibrasi dan panjang selang
    float tinggiair = waterLevel*100 + 197.03;
    // float ketinggian = (0.0317*take_avg())-0.9412; //penggaris
    float ketinggian = (0.2829*take_avg())-9.9141; //hasil 8/6/2023 
    // float ketinggian = (0.25*take_avg())-9.25;
    FinalData.waterLevel = ketinggian + 1;
}


void detection(){
    // _rainfall_rate = 5;
    // if (_rainfall_rate > rainfall_rate_danger || _soil_moisture > soil_moisture_danger || _humidity > humidity_danger) flood_category = 3;
    // else if ((_humidity > humidity_warning) || (_soil_moisture > soil_moisture_warning)) flood_category = 2; 
    // else flood_category = 1;
    
    // acceleration_magnitude = sqrt(pow(_accel_x, 2) + pow(_accel_y, 2) + pow(_accel_z, 2));
    // // if (acceleration_magnitude > acceleration_danger && _soil_moisture > soil_moisture_warning) {
    // //     landslide_category = 3; 
    // // } else if ((acceleration_magnitude > acceleration_warning && acceleration_magnitude <= acceleration_danger) || (_soil_moisture > soil_moisture_safe && _soil_moisture <= soil_moisture_warning)) {
    // //     landslide_category = 2; 
    // // } else{
    // //     landslide_category = 1; 
    // // }
    // if (abs(_accel_x) > 1500 || abs(_accel_y) > 1500 && _soil_moisture > soil_moisture_danger) {
    //     landslide_category = 3; 
    // } else if (abs(_accel_x) > 800 || abs(_accel_y) > 800 || _soil_moisture > soil_moisture_warning) {
    //     landslide_category = 2; 
    // } else{
    //     landslide_category = 1; 
    // }
}
