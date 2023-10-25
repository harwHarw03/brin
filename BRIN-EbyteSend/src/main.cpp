#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include "Sensor.h"

// wifi 
const char* ssid = "test";
const char* password = "00000000";
WiFiClient espClient;
wl_status_t wifiStatus;

// mqtt 
const char* mqtt_server = "broker.hivemq.com";
const char* mqtt_topic = "test123l";
PubSubClient client(espClient);
String message = "";

void reconnect() 
{
  while (!client.connected()) 
  {
    Serial.println("Connecting to MQTT broker...");
    if (client.connect("ESP32Client")) 
    {
      Serial.println("Connected to MQTT broker");
    } else 
    {
      Serial.print("Failed to connect to MQTT broker, rc=");
      Serial.print(client.state());
      Serial.println(" retrying in 5 seconds");
      delay(3000);
    }
  }
}

class MQTT : public TimingTask
{
    public:
        MQTT(uint32_t _rate):rate(_rate){updateTime(millis());}

        virtual void run(uint32_t now){
            if (!client.connected()) 
            {
                reconnect();
            }
            Serial.println("Publish");
            client.publish(mqtt_topic, serializedData.c_str());
            tick(rate);
        } 
    private:
        uint32_t rate;
};

void setup() 
{
  Serial.begin(9600);  //usb serial
  // Serial1.begin(9600); //lora
  
  wifiStatus = WiFi.begin(ssid, password);
  // while (wifiStatus != WL_CONNECTED) 
  // {
    Serial.println("Connecting to WiFi...");
    delay(500);
  // }
  Serial.println("Connected to WiFi network");
  
  // connect to mqtt broker
  client.setServer(mqtt_server, 1883);  // port : 1883
  Serial.println("Waitt");
  delay(1000);
  if (!client.connected()) 
    {
        reconnect();
    }
    client.publish(mqtt_topic, "test");
    if (!client.connected()) 
    {
        reconnect();
    }
    client.publish(mqtt_topic, "Test1");
    if (!client.connected()) 
    {
        reconnect();
    }
    client.publish(mqtt_topic, "test3");
  sensor_init();

    Blinker blinking(750);
    IMUPDATE imu_update(50);
    WATERLEVEL water_level(150);
    SUPPORT support_sensor(100);
    SUPPORT2 support_sensor2(500);
    Logger logger(100);
    MQTT mqtt(5000);
    // Task *tasks[] = {&imu_update, &support_sensor, &support_sensor2, &water_level, &logger};
    Task *tasks[] = {&imu_update, &logger, &mqtt};
    Scheduler scheduler(tasks, NUM_TASKS(tasks));
    while(1){scheduler.runTasks();
    
    if (!client.connected()) 
            {
                reconnect();
            }
            Serial.println("Publish");
            client.publish(mqtt_topic, serializedData.c_str());
    }
}

void loop() 
{
    // #ifdef MASTER_BOARD
    // while (Serial1.available() > 0)
    // {
    //     message = Serial1.readString();
    // }
    if (!client.connected()) 
    {
        reconnect();
    }
    client.publish(mqtt_topic, serializedData.c_str());//message.c_str());  // send string as aarrayy of char
    // #endif
    
}