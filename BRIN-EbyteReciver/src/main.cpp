/**
 * Created by K. Suwatchai (Mobizt)
 *
 * Email: k_suwatchai@hotmail.com
 *
 * Github: https://github.com/mobizt
 *
 * Copyright (c) 2021 mobizt
 *
 */
// https://github.com/arduino-libraries/ArduinoMqttClient
#include "soc/soc.h"
#include "soc/rtc.h"

#include <ArduinoMqttClient.h>
#if defined(ARDUINO_SAMD_MKRWIFI1010) || defined(ARDUINO_SAMD_NANO_33_IOT) || defined(ARDUINO_AVR_UNO_WIFI_REV2)
#include <WiFiNINA.h>
#elif defined(ARDUINO_SAMD_MKR1000)
#include <WiFi101.h>
#elif defined(ESP8266)
#include <ESP8266WiFi.h>
#elif defined(ESP32)
#include <WiFi.h>
#endif
// Enable LW MQTT library after include the library and before include the FirebaseJson.
#include <FirebaseJson.h>
#include <Arduino.h>
#include "EBYTE.h"
#include "time.h"

HardwareSerial SerialAT(2);

bool parsing = false;
String sData, data[15];
/*
WARNING: IF USING AN ESP32
DO NOT USE THE PIN NUMBERS PRINTED ON THE BOARD
YOU MUST USE THE ACTUAL GPIO NUMBER
*/
#define PIN_RX 16   // Serial2 RX (connect this to the EBYTE Tx pin)
#define PIN_TX 17   // Serial2 TX pin (connect this to the EBYTE Rx pin)
#define PIN_M0 4    // D4 on the board (possibly pin 24)
#define PIN_M1 22   // D2 on the board (possibly called pin 22)
#define PIN_AX 21   // D15 on the board (possibly called pin 21)
// i recommend putting this code in a .h file and including it
// from both the receiver and sender modules
struct DATA {
  int sensor_soil;
  // int soil_info;
  float sensor_SHT20_temp;
  float sensor_SHT20_hum;
  float latitude;
  float longitud;
  float sensor_MPX;
  float Acc_X;
  float Acc_Y;
  float Acc_Z;
  float Gyro_X;
  float Gyro_Y;
  float Gyro_Z;
  float Orientasi_X;
  float Orientasi_Y;
  // float Orientasi_Z;
  float Battery;
};
// these are just dummy variables, replace with your own
int Chan;
DATA MyData;
unsigned long Last;
// create the transceiver object, passing in the serial and pins
EBYTE Transceiver(&Serial2, PIN_M0, PIN_M1, PIN_AX);
//=========================================================================================================================================================================
// /* Define the WiFi credentials */
#define WIFI_SSID "Subhanallah"
#define WIFI_PASSWORD "muhammadnabiyullah"
WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);
unsigned long lastMillis = 0;
int count = 0;
const char broker[] = "test.mosquitto.org";
int port = 1883;
const char topic[] = "tessensorews";
const long interval = 1000;
unsigned long previousMillis = 0;
bool mqttReady = false;
int MQTT_Soil = 0;
float MQTT_SHT20_Temp = 0;
float MQTT_SHT20_Hum = 0;
float MQTT_Lat = 0.0000000;
float MQTT_Long = 0.0000000;
float MQTT_MPX = 0;

//TIME//
const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 25200;
const int   daylightOffset_sec = 0;
int JAM, MENIT;

void printLocalTime()
{
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    Serial.println("Failed to obtain time");
    return;
  }
  Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
  JAM = timeinfo.tm_hour;
  MENIT = timeinfo.tm_min;
}

void send_data(float data1, float data2){
  String string_data;
  string_data = '#';
  string_data += data1;
  string_data += '*';
  string_data += data2;
  string_data += '*';
  string_data += '@';
  SerialAT.println(string_data);
}

void setup() {
  Serial.begin(9600);
  SerialAT.begin(9600,SERIAL_8N1,16,17);
  Serial.println("Starting Reader");

  // this init will set the pinModes for you
  Transceiver.init();
  // all these calls are optional but shown to give examples of what you can do
  // Serial.println(Transceiver.GetAirDataRate());
  // Serial.println(Transceiver.GetChannel());
  // Transceiver.SetAddressH(1);
  // Transceiver.SetAddressL(1);
  // Chan = 15;
  // Transceiver.SetChannel(Chan);
  // save the parameters to the unit,
  // Transceiver.SetPullupMode(1);
  // Transceiver.SaveParameters(PERMANENT);
  // you can print all parameters and is good for debugging
  // if your units will not communicate, print the parameters
  // for both sender and receiver and make sure air rates, channel
  // and address is the same
  Transceiver.PrintParameters();
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(300);
  }
  Serial.println();
  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP());
  Serial.println();

  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  printLocalTime();

  Serial.print("Attempting to connect to the MQTT broker: ");
  Serial.println(broker);
  if (!mqttClient.connect(broker, port))
  {
    Serial.print("MQTT connection failed! Error code = ");
    Serial.println(mqttClient.connectError());
    return;
  }
  mqttReady = true;
  Serial.println("You're connected to the MQTT broker!");
  Serial.println();

}

void loop() {
  if (SerialAT.available()) {
    sData = SerialAT.readStringUntil('@');
    Serial.println(sData);
    int q = 0;
    String tempData;
    bool parsing = false; // Menandakan bahwa proses parsing sedang berlangsung
    for (int i = 0; i < sData.length(); i++) {
      if (sData[i] == '#') {
        parsing = true;
        tempData = "";
      } else if (sData[i] == '*') {
        if (parsing) {
          data[q] = tempData;
          // Serial.print("ini data yang masuk:   ");
          // Serial.println(data[q]);
          tempData = "";
          q++;
        }
      } else {
        if (parsing) {
          tempData += sData[i];
        }
      }
    }

    if (data[13] != "") { // Memastikan data telah di-parse dengan lengkap
      MyData.sensor_soil = data[0].toFloat();
      MyData.sensor_SHT20_temp = data[1].toFloat();
      MyData.sensor_SHT20_hum = data[2].toFloat();
      MyData.latitude = data[3].toFloat();
      MyData.longitud = data[4].toFloat();
      MyData.sensor_MPX = data[5].toFloat();
      MyData.Acc_X = data[6].toFloat();
      MyData.Acc_Y = data[7].toFloat();
      MyData.Acc_Z = data[8].toFloat();
      MyData.Gyro_X = data[9].toFloat();
      MyData.Gyro_Y = data[10].toFloat();
      MyData.Gyro_Z = data[11].toFloat();
      MyData.Orientasi_X = data[12].toFloat();
      MyData.Orientasi_Y = data[13].toFloat();
      MyData.Battery = data[14].toFloat();

      Serial.print("Sensor Soil value: "); Serial.println(MyData.sensor_soil);
      Serial.print("Temperature SHT20: "); Serial.println(MyData.sensor_SHT20_temp);
      Serial.print("Humidity SHT20: "); Serial.println(MyData.sensor_SHT20_hum);
      Serial.print("Latitude: "); Serial.println(MyData.latitude, 7);
      Serial.print("Longitude: "); Serial.println(MyData.longitud, 7);
      Serial.print("Water level: "); Serial.println(MyData.sensor_MPX);
      Serial.print("Acc X: "); Serial.println(MyData.Acc_X);
      Serial.print("Acc Y: "); Serial.println(MyData.Acc_Y);
      Serial.print("Acc Z: "); Serial.println(MyData.Acc_Z);
      Serial.print("Gyro X: "); Serial.println(MyData.Gyro_X);
      Serial.print("Gyro Y: "); Serial.println(MyData.Gyro_Y);
      Serial.print("Gyro Z: "); Serial.println(MyData.Gyro_Z);
      Serial.print("Ori X: "); Serial.println(MyData.Orientasi_X);
      Serial.print("Ori Y: "); Serial.println(MyData.Orientasi_Y);
      Serial.print("Battery: "); Serial.println(MyData.Battery);

      // Reset parsing dan data setelah memproses satu set data
      for (int i = 0; i < 14; i++) {
        data[i] = "";
      }
    }
  }

   if (!mqttReady)
        return;
  mqttClient.poll();
  if (millis() - lastMillis > 1000)
  {
    lastMillis = millis();
    Serial.print("Sending message to topic: ");
    Serial.println(topic);
 
    FirebaseJson json;
 
    json.add("A", MyData.sensor_soil);
    json.add("B", MyData.sensor_SHT20_temp);
    json.add("C", MyData.sensor_SHT20_hum);
    json.add("D", MyData.latitude);
    json.add("E", MyData.longitud);
    json.add("F", MyData.sensor_MPX);
    json.add("G", MyData.Acc_X);
    json.add("H", MyData.Acc_Y);
    json.add("I", MyData.Acc_Z);
    json.add("J", MyData.Gyro_X);
    json.add("K", MyData.Gyro_Y);
    json.add("L", MyData.Gyro_Z);
    json.add("M", MyData.Orientasi_X);
    json.add("N", MyData.Orientasi_Y);
    json.add("O", MyData.Battery);
 
    json.toString(Serial);
 
    // send message, the Print interface can be used to set the message contents
    mqttClient.beginMessage(topic);
    json.toString(mqttClient);
    mqttClient.endMessage();
    count++;
    }
  printLocalTime();
  send_data(JAM,MENIT);
  delay(5000);
}

// void loop() {
//   // if the transceiver serial is available, proces incoming data
//   // you can also use Transceiver.available()
//   if (SerialAT.available()) {
//     char inChar = SerialAT.read();
//     sData = SerialAT.read();
//     if (inChar == '#')
//     {
//       // Serial.print(2);
//       parsing = true;
//     }

//     // if (parsing)
//     // {
//     //   int q = 0;
//     //   for (int i = 0; i < sData.length(); i++)
//     //   {
//     //     if (sData [i] == '*')
//     //     {
//     //       data [q] = "";
//     //     } 
//     //     data[q] = sData[i];
//     //     q++;
        
//     //   }
//     // }

//     if (parsing) {
//     int q = 0;
//     String tempData;
//     for (int i = 0; i < sData.length(); i++) {
//       if (sData[i] == '*') {
//         data[q] = tempData;
//         tempData = "";
//         q++;
//       } else {
//         tempData += sData[i];
//       }
//     }
//     }
//   MyData.sensor_soil = data[0].toInt();
//   MyData.sensor_SHT20_temp = data[1].toFloat();
//   MyData.sensor_SHT20_hum = data[2].toFloat();
//   MyData.latitude = data[3].toFloat();
//   MyData.longitud = data[4].toFloat();
//   MyData.sensor_MPX = data[5].toFloat();
//   MyData.Acc_X = data[6].toFloat();
//   MyData.Acc_Y = data[7].toFloat();
//   MyData.Acc_Z = data[8].toFloat();
//   MyData.Gyro_X = data[9].toFloat();
//   MyData.Gyro_Y = data[10].toFloat();
//   MyData.Gyro_Z = data[11].toFloat();
//   MyData.Orientasi_X = data[12].toFloat();
//   MyData.Orientasi_Y = data[13].toFloat();
//     // i highly suggest you send data using structures and not
//     // a parsed data--i've always had a hard time getting reliable data using
//     // a parsing method
//     // Transceiver.GetStruct(&MyData, sizeof(MyData));
//     // note, you only really need this library to program these EBYTE units
//     // you can call readBytes directly on the EBYTE Serial object
//     // Serial2.readBytes((uint8_t*)& MyData, (uint8_t) sizeof(MyData));
//     // MQTT_Soil = MyData.sensor_soil;
//     // MQTT_SHT20_Temp = MyData.sensor_SHT20_temp;
//     // MQTT_SHT20_Hum = MyData.sensor_SHT20_hum;
//     // MQTT_Lat = MyData.latitude;
//     // MQTT_Long = MyData.longitud;
//     // MQTT_MPX = MyData.sensor_MPX;
//     // dump out what was just received
//     Serial.print("Sensor Soil value: "); Serial.println(MyData.sensor_soil);
//     // Serial.print("Humidity Soil: "); Serial.println(MyData.soil_info);
//     Serial.print("Temperature SHT20: "); Serial.println(MyData.sensor_SHT20_temp);
//     Serial.print("Humidity SHT20: "); Serial.println(MyData.sensor_SHT20_hum);
//     Serial.print("Latitude: "); Serial.println(MyData.latitude,7);
//     Serial.print("Longitud: "); Serial.println(MyData.longitud,7);
//     Serial.print("Water level: "); Serial.println(MyData.sensor_MPX);
//     Serial.print("Acc X: "); Serial.println(MyData.Acc_X);
//     Serial.print("Acc Y: "); Serial.println(MyData.Acc_Y);
//     Serial.print("Acc Z: "); Serial.println(MyData.Acc_Z);
//     Serial.print("Gyro X: "); Serial.println(MyData.Gyro_X);
//     Serial.print("Gyro Y: "); Serial.println(MyData.Gyro_Y);
//     Serial.print("Gyro Z: "); Serial.println(MyData.Gyro_Z);
//     Serial.print("Ori X: "); Serial.println(MyData.Orientasi_X);
//     Serial.print("Ori Y: "); Serial.println(MyData.Orientasi_Y);
//     // Serial.print("Orientation Z: "); Serial.println(MyData.Orientasi_Z);
//     // if you got data, update the checker
//     Last = millis();
//   }
//   // else {
//   //   // if the time checker is over some prescribed amount
//   //   // let the user know there is no incoming data
//   //   if ((millis() - Last) > 1000) {
//   //     Serial.println("Searching: ");
//   //     Last = millis();
//   //   }
//   // }
 
//   delay(5000);
// }
// If you want use RSSI uncomment
//#define ENABLE_RSSI true
// #include "Arduino.h"
// #include "LoRa_E220.h"
// // ---------- esp8266 pins --------------
// //LoRa_E220 e220ttl(RX, TX, AUX, M0, M1);  // Arduino RX <-- e220 TX, Arduino TX --> e220 RX
// // LoRa_E220 e220ttl(D3, D4, D5, D7, D6); // Arduino RX <-- e220 TX, Arduino TX --> e220 RX AUX M0 M1
// //LoRa_E220 e220ttl(D2, D3); // Config without connect AUX and M0 M1
// //#include <SoftwareSerial.h>
// //SoftwareSerial mySerial(D2, D3); // Arduino RX <-- e220 TX, Arduino TX --> e220 RX
// //LoRa_E220 e220ttl(&mySerial, D5, D7, D6); // AUX M0 M1
// // -------------------------------------
// // ---------- Arduino pins --------------
// //LoRa_E220 e220ttl(4, 5, 3, 7, 6); // Arduino RX <-- e220 TX, Arduino TX --> e220 RX AUX M0 M1
// //LoRa_E220 e220ttl(4, 5); // Config without connect AUX and M0 M1
// //#include <SoftwareSerial.h>
// //SoftwareSerial mySerial(4, 5); // Arduino RX <-- e220 TX, Arduino TX --> e220 RX
// //LoRa_E220 e220ttl(&mySerial, 3, 7, 6); // AUX M0 M1
// // -------------------------------------
// // ------------- Arduino Nano 33 IoT -------------
// // LoRa_E220 e220ttl(&Serial1, 2, 4, 6); //  RX AUX M0 M1
// // -------------------------------------------------
// // ------------- Arduino MKR WiFi 1010 -------------
// // LoRa_E220 e220ttl(&Serial1, 0, 2, 4); //  RX AUX M0 M1
// // -------------------------------------------------
// // ---------- esp32 pins --------------
// LoRa_E220 e220ttl(&Serial2, 19, 18, 5); //  RX AUX M0 M1
// //LoRa_E220 e220ttl(&Serial2, 22, 4, 18, 21, 19, UART_BPS_RATE_9600); //  esp32 RX <-- e220 TX, esp32 TX --> e220 RX AUX M0 M1
// // -------------------------------------
// // ---------- Raspberry PI Pico pins --------------
// // LoRa_E220 e220ttl(&Serial2, 2, 10, 11); //  RX AUX M0 M1
// // -------------------------------------
// // ---------------- STM32 --------------------
// //HardwareSerial Serial2(USART2);   // PA3  (RX)  PA2  (TX)
// //LoRa_E220 e220ttl(&Serial2, PA0, PB0, PB10); //  RX AUX M0 M1
// // -------------------------------------------------
// void setup() {
//   Serial.begin(9600);
//   delay(500);
//   // Startup all pins and UART
//   e220ttl.begin();
//   Serial.println("Start receiving!");
// }
// void loop() {
// 	// If something available
//   if (e220ttl.available()>1) {
// 	  Serial.println("Message received!");
// 	  // read the String message
// #ifdef ENABLE_RSSI
// 	ResponseContainer rc = e220ttl.receiveMessageRSSI();
// #else
// 	ResponseContainer rc = e220ttl.receiveMessage();
// #endif
// 	// Is something goes wrong print error
// 	if (rc.status.code!=1){
// 		Serial.println(rc.status.getResponseDescription());
// 	}else{
// 		// Print the data received
// 		Serial.println(rc.status.getResponseDescription());
// 		Serial.println(rc.data);
// #ifdef ENABLE_RSSI
// 		Serial.print("RSSI: "); Serial.println(rc.rssi, DEC);
// #endif
// 	}
//   }
// }

// int solenoidPin = 26; //This is the output pin on the Arduino we are using
// int pompaPin = 27; //This is the output pin on the Arduino we are using

// void setup() {
// // put your setup code here, to run once:
// pinMode(solenoidPin, OUTPUT); //Sets the pin as an output
// pinMode(pompaPin, OUTPUT); //Sets the pin as an output
// }

// void loop() {
// // put your main code here, to run repeatedly:
// digitalWrite(solenoidPin, HIGH); //Switch Solenoid ON
// digitalWrite(pompaPin, HIGH); //Switch Solenoid ON
// delay(1000); //Wait 1 Second
// digitalWrite(solenoidPin, LOW); //Switch Solenoid OFF
// digitalWrite(pompaPin, LOW); //Switch Solenoid OFF
// delay(1000); //Wait 1 Second
// }

// #include <Wire.h>

// // Pin sensor
// const int sensorPin = A0;

// // Konfigurasi sensor
// float sensorOffset = 0.0; // Offset kalibrasi
// float sensorScale = 0.045; // Skala kalibrasi (mm/mV)
// const float hoseDiameter = 4.5; // Diameter selang (mm)

// // Variabel global
// float waterLevel = 0.0;

// void setup() {
//   // Inisialisasi Serial Monitor
//   Serial.begin(9600);
//   while (!Serial) {
//     // Tunggu hingga Serial Monitor terhubung
//   }

//   // Kalibrasi nilai awal
//   Serial.println("Kalibrasi...");
//   delay(5000); // Tunggu 5 detik

//   // Baca nilai awal sensor sebagai offset
//   float sensorValue = analogRead(sensorPin);
//   sensorOffset = sensorValue * sensorScale;

//   // Tampilkan pesan selesai kalibrasi
//   Serial.println("Kalibrasi selesai.");
// }

// void loop() {
//   // Baca nilai sensor
//   float sensorValue = analogRead(sensorPin);

//   // Konversi nilai sensor menjadi level air dalam mm
//   waterLevel = (sensorValue * sensorScale) - sensorOffset;

//   // Tampilkan hasil level air
//   Serial.print("ADC : ");
//   Serial.print(sensorValue);
//   Serial.print(" Level Air: ");
//   Serial.print(waterLevel);
//   Serial.println(" mm");

//   // Tunda 1 detik
//   delay(1000);
// }




