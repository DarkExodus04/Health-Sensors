#include <heartRate.h>
#include <MAX30105.h>
#include <spo2_algorithm.h>
#include <WiFi.h>
#include <CircularBuffer.h>
#include <Wire.h>
#include <ArduinoJson.h>
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
#define ADC_VREF_mV    3300.0 // in millivolt
#define ADC_RESOLUTION 4096.0

MAX30105 particleSensor;
#define MAX_BRIGHTNESS 255 
/****************************************
 * Auxiliar Functions
 ****************************************/
float BPM;
uint32_t tsLastReport = 0;
StaticJsonDocument<500> doc;
int32_t bufferLength; //data length
uint32_t irBuffer[100]; //infrared LED sensor data
uint32_t redBuffer[100];  //red LED sensor data
int32_t spo2; //SPO2 value
int8_t validSPO2; //indicator to show if the SPO2 calculation is valid
int32_t heartRate; //heart rate value
int8_t validHeartRate;
long unblockedValue;
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
  // Initialize sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println(F("MAX30105 was not found. Please check wiring/power."));
    while (1);
  }
  byte ledBrightness = 60; //Options: 0=Off to 255=50mA
  byte sampleAverage = 4; //Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  byte sampleRate = 100; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411; //Options: 69, 118, 215, 411
  int adcRange = 4096; //Options: 2048, 4096, 8192, 16384

  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings

  unblockedValue = 0;
  for (byte x = 0 ; x < 32 ; x++)
  {
    unblockedValue += particleSensor.getIR(); //Read the IR value
  }
  unblockedValue /= 32;
}

 
void loop() {

bufferLength = 100; //buffer length of 100 stores 4 seconds of samples running at 25sps

  //read the first 100 samples, and determine the signal range
  for (byte i = 0 ; i < bufferLength ; i++)
  {
    while (particleSensor.available() == false) //do we have new data?
      particleSensor.check(); //Check the sensor for new data

    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample(); //We're finished with this sample so move to next sample
  }

  //calculate heart rate and SpO2 after first 100 samples (first 4 seconds of samples)
  maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);

  //Continuously taking samples from MAX30102.  Heart rate and SpO2 are calculated every 1 second
  while (1)
  {
    //dumping the first 25 sets of samples in the memory and shift the last 75 sets of samples to the top
    for (byte i = 25; i < 100; i++)
    {
      redBuffer[i - 25] = redBuffer[i];
      irBuffer[i - 25] = irBuffer[i];
    }

    //take 25 sets of samples before calculating the heart rate.
    for (byte i = 75; i < 100; i++)
    {
      while (particleSensor.available() == false) //do we have new data?
        particleSensor.check(); //Check the sensor for new data
        
      redBuffer[i] = particleSensor.getRed();
      irBuffer[i] = particleSensor.getIR();
      particleSensor.nextSample();
    }

      // read the ADC value from the temperature sensor
    int adcVal = analogRead(temp);
    // convert the ADC value to voltage in millivolt
    float milliVolt = adcVal * (ADC_VREF_mV / ADC_RESOLUTION);

    // convert the voltage to the temperature in °C
    float tempC = milliVolt / 10;

    // convert the °C to °F
    float Ecg = analogRead(ECG);
    
//    long currentDelta = particleSensor.getIR() - unblockedValue;
//    if (currentDelta > (long)100)
//  {
//    heartRate = random(75,90);
//    spo2 = random(93,99);
//    tempC = random(26,32);
//  }
//  else
//  {
//    heartRate = 0;
//    spo2 = 0;
//    tempC = random(26,32);
//  }
    
    if (millis() - tsLastReport > REPORTING_PERIOD_MS)
      {
          Serial.print("Heart rate: ");
          Serial.println(heartRate);
          doc["sensors"]["Heartrate"] = heartRate;
          Serial.print(" SpO2: ");
          Serial.print(spo2);
          doc["sensors"]["Oxygen"] = spo2;
          Serial.println(" %");
          tsLastReport = millis();
          Serial.print("ECG value: ");
          doc["sensors"]["ECG"] = ECG;
          Serial.println(Ecg);
          Serial.print("Temperature: ");
          Serial.println(tempC);
          doc["sensors"]["Temperature"] = tempC;
  //        POSTData();
      }
//After gathering 25 new samples recalculate HR and SP02
    maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
  delay(2000);
  }

}

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
