#include <CircularBuffer.h>
#include <MAX30100.h>
#include <MAX30100_BeatDetector.h>
#include <MAX30100_Filters.h>
#include <MAX30100_PulseOximeter.h>
#include <MAX30100_Registers.h>
#include <MAX30100_SpO2Calculator.h>
#include <Wire.h>
#include <ArduinoJson.h>
#include <WiFi.h>
#include "secrets.h"
#include <HTTPClient.h>

/****************************************
 * Define Constants
 ****************************************/


#define WIFISSID "Dark Exodus" // Put your WifiSSID here
#define PASSWORD "nathan06" // Put your wifi password here 
#define ECG A0
#define temp 34
#define REPORTING_PERIOD_MS 1000
 
/****************************************
 * Auxiliar Functions
 ****************************************/
float BPM, SpO2;
uint32_t tsLastReport = 0;
StaticJsonDocument<500> doc;
PulseOximeter pox;

void onBeatDetected()
{
    Serial.println("Beat Detected!");
}
 
/****************************************
 * Main Functions
 ****************************************/
void setup() {
  Serial.begin(115200);
  WiFi.begin(WIFISSID, PASSWORD);
  // Assign the pin as INPUT 
  pinMode(ECG, INPUT);
  pinMode(temp, INPUT);
  //Oximeter
  Serial.print("Initializing Pulse Oximeter..");
 
//    if (!pox.begin())
//    {
//         Serial.println("FAILED");
//         for(;;);
//    }
//    else
//    {
//         Serial.println("SUCCESS");
//         pox.setOnBeatDetectedCallback(onBeatDetected);
//    }
}
 
void loop() {
//  pox.update();
  float Ecg = analogRead(ECG);
  float Temp = analogRead(temp);
  BPM = pox.getHeartRate();
//  SpO2 = pox.getSpO2();
  SpO2 = 32.23;
  if (millis() - tsLastReport > REPORTING_PERIOD_MS)
    {
        Serial.print("Heart rate:");
        Serial.println(BPM);
        doc["sensors"]["Heartrate"] = BPM;
        Serial.print(" SpO2:");
        Serial.print(SpO2);
        doc["sensors"]["Oxygen"] = SpO2;
        Serial.println(" %");
        tsLastReport = millis();
        Serial.print("ECG value");
        doc["sensors"]["ECG"] = ECG;
        Serial.println(Ecg);
        Serial.print("Temperature");
        Serial.println(Temp);
        doc["sensors"]["Temperature"] = Temp;
//        POSTData();
    }
  
  delay(3000);
}


//
//void POSTData()
//{
//      
//      if(WiFi.status()== WL_CONNECTED){
//      HTTPClient http; http;
//
//      http.begin(serverName);
//      http.addHeader("Content-Type", "application/json");
//
//      String json;
//      serializeJson(doc, json);
//
//      Serial.println(json);
//      int httpResponseCode = http.POST(json);
//      Serial.println(httpResponseCode);
//      }
//}
