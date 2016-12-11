/*
 * Send push alert when your washing machine dies or finishes.
 * This code has been taken from bits and pieces around the place.
 * It's more of a proof of concept than a polished product, but works well.
 * 
 */

#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>
#include <ESP8266HTTPClient.h>
#include <Wire.h>
#include "I2Cdev.h"
#include "RTIMUSettings.h"
#include "RTIMU.h"
#include "RTFusionRTQF.h" 
#include "CalLib.h"
#include <EEPROM.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_TSL2561_U.h>
#include <Bounce2.h>


// The higher the poll rate the more battery consumed!
#define CHECK_INTERVAL  100		// Sensor poll interval, (Lux should be no less than 20mS and neither the IMU).
#define DISPLAY_INTERVAL  1000		// telnet/serial port update interval.
#define SERIAL_PORT_SPEED  115200
#define LED 0
#define BUTTON1 2			// Lux sensitivity button.
#define BUTTON2 13			// IMU sensitivity button.
#define MAX_SRV_CLIENTS 3		// Max telnet clients.
#define LUXFUDGE 2.5			// Defines the sensitivity of the Lux meter to change.
#define IMUFUDGE 2.0			// Defines the sensitivity of the IMU to change.

const char *AppId = "584a1f18a4c48a7e5e07ef93";
const char *AppSecret = "324d4cc35d4d78d5f73405bdaf63929e";
const char *ssid     = "HOMENET2";
const char *password = "Proverbs 2:15";
const char *CertKey = ""8B D0 1F 45 48 19 26 45 65 9B 3F B3 59 C8 B2 BC F1 D0 4A EA"";


RTIMU *imu;                                           // the IMU object
RTFusionRTQF fusion;                                  // the fusion object
RTIMUSettings settings;                               // the settings object
float sensorIMU;
float restingIMU;
float flagIMU;
float flagLastIMU;
float varianceIMU;

Adafruit_TSL2561_Unified tsl = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 4242);
sensors_event_t eventLux;
float sensorLux;
float restingLux;
float flagLux;
float flagLastLux;
float varianceLux;

unsigned long lastDisplay;
unsigned long lastCheck;

Bounce dbButt1 = Bounce(); 
Bounce dbButt2 = Bounce(); 

WiFiServer server(23);
WiFiClient serverClients[MAX_SRV_CLIENTS];


void setup()
{
    int errcode;

    delay(2000);
    Serial1.begin(SERIAL_PORT_SPEED);
    Serial1.print("\nStarting up");

    //start UART and the server
    Serial.begin(115200);
    server.begin();
    server.setNoDelay(true);
    
    Wire.begin();
    pinMode(LED, OUTPUT);

    pinMode(BUTTON1,INPUT_PULLUP);
    dbButt1.attach(BUTTON1);
    dbButt1.interval(5);
    pinMode(BUTTON2,INPUT_PULLUP);
    dbButt2.attach(BUTTON2);
    dbButt2.interval(5);

    setupIMU();
    setupLux();
    lastCheck = lastDisplay = millis();

    APconnect();
}


void setupLux(void)
{
    sensor_t sensor;

    Serial.println("Light Sensor Test");

    /* Initialise the sensor */
    if(!tsl.begin())
    {
        Serial.println("Failed to init TSL2561!");
        while(1)
        {
            // Produce a visible error code.
            errorFLash(500);
        }
    }

    /* Display some basic information on this sensor */
    tsl.getSensor(&sensor);
    Serial.print("Sensor:        "); Serial.println(sensor.name);
    Serial.print("Driver Ver:    "); Serial.println(sensor.version);
    Serial.print("Min/Max Value: "); Serial.print(sensor.min_value); Serial.print(" / "); Serial.println(sensor.max_value);
    Serial.print("Res:           "); Serial.println(sensor.resolution);

    /* Setup the sensor gain and integration time */
    tsl.enableAutoRange(true);
    tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_13MS);

    // Fetch Lux sensor data.
    tsl.getEvent(&eventLux);
    if (eventLux.light)
    {
        sensorLux = restingLux = eventLux.light;
    }
    flagLux = LOW;
}


void setupIMU(void)
{
    int errcode;

    Wire.begin();
    imu = RTIMU::createIMU(&settings);                        // create the imu object
  
    Serial.print("ArduinoIMU starting using device "); Serial.println(imu->IMUName());
    if ((errcode = imu->IMUInit()) < 0)
    {
        Serial.print("Failed to init IMU: ");
        Serial.println(errcode);
        while(1)
        {
            // Produce a visible error code.
            errorFLash(1000);
        }
    }
    Serial.print("Recommended poll delay: ");
    Serial.println(imu->IMUGetPollInterval());
    
    if (imu->getCalibrationValid())
        Serial.println("Using compass calibration");
    else
        Serial.println("No valid compass calibration data");

    // Slerp power controls the fusion and can be between 0 and 1
    // 0 means that only gyros are used, 1 means that only accels/compass are used
    // In-between gives the fusion mix.
    fusion.setSlerpPower(0.02);
    
    // use of sensors in the fusion algorithm can be controlled here
    // change any of these to false to disable that sensor
    fusion.setGyroEnable(false);
    fusion.setAccelEnable(true);
    fusion.setCompassEnable(false);

    // Read initial values.
    while (imu->IMURead())
    {
        sensorIMU = restingIMU = GravityComp((RTVector3&)imu->getAccel());
    }
    flagIMU = LOW;
}

void errorFLash(unsigned long Pause)
{
    // Produce a visible error code.
    digitalWrite(LED, HIGH);
    delay(Pause);
    digitalWrite(LED, LOW);
    delay(Pause/2);
}


void loop()
{
    unsigned long now = millis();

    // Check sensors.
    if ((now - lastCheck) >= CHECK_INTERVAL)
    {
        lastCheck = now;

        // Fetch IMU data.
        while (imu->IMURead())
        {
            sensorIMU = GravityComp((RTVector3&)imu->getAccel());

            // Have we got an out-of-balance flag?
            if (!flagIMU)
            {
                // No, then computer the variance and see if it is.
                varianceIMU = restingIMU / sensorIMU;
                if (varianceIMU > IMUFUDGE)  // Fudge values! be warned!
                {
                    // Out of balance.
                    flagIMU = HIGH;
                }
                // Ignore everything else.
            }
        }

        // Fetch Lux sensor data.
        tsl.getEvent(&eventLux);
        if (eventLux.light)
        {
            sensorLux = eventLux.light;

                // No, then computer the variance and see if it is.
                varianceLux = sensorLux / restingLux;
                if (varianceLux > LUXFUDGE)  // Fudge values! be warned!
                {
                    // Light on.
                    flagLux = HIGH;
                }
                else
                {
                    // Light off.
                    flagLux = LOW;
                }
        }
    }

    // Display to serial.
    if ((now - lastDisplay) >= DISPLAY_INTERVAL)
    {
        lastDisplay = now;

        showData();
        PushEvent();
    }

    // Check on the buttons.
    dbButt1.update();
    if (!dbButt1.read())
    {
        resetIMU();
    }
    dbButt2.update();
    if (!dbButt2.read())
    {
        resetLux();
    }

    checkTelnet();
}


void resetIMU(void)
{
    digitalWrite(LED, HIGH);
    restingIMU = sensorIMU;
    flagIMU = flagLastIMU = LOW;
}


void resetLux(void)
{
    digitalWrite(LED, LOW);
    restingLux = sensorLux;
    flagLux = flagLastLux = LOW;
}


void showData(void)
{
    String sendData = "";
    Serial.print("DEBUG: restingLux:");
    Serial.print(restingLux);
    Serial.print("sensorLux:");
    Serial.print(sensorLux);
    Serial.print("flagLux:");
    Serial.print(flagLux);
    Serial.print("flagLastLux:");
    Serial.println(flagLastLux);

    sendData += "IMU(";
    sendData += String(restingIMU);
    sendData += "): ";
    sendData += String(sensorIMU);
    sendData += " Lux(";
    sendData += String(restingLux);
    sendData += "): ";
    sendData += String(sensorLux);

    sendData += "    OOB:";
    if (flagIMU)
    {
        sendData += "Yes";
    }
    else
    {
        sendData += "No";
    }

    sendData += "    Light:";
    if (flagLux)
    {
        sendData += "On\n\r";
    }
    else
    {
        sendData += "Off\n\r";
    }
    Serial.print(sendData);
    writeTelnet(sendData);
}


float GravityComp(RTVector3& vec)
{
    float Grav = sqrt(sq(vec.x()) + sq(vec.y()) + sq(vec.z()));

    return(Grav);
}


void APconnect()
{
    // 
    Serial.print("AP:");
    Serial.println(ssid);
  
    WiFi.begin(ssid, password);
  
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
    }

    Serial.println("");
    Serial.println("WiFi connected");  
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());

    server.begin();
    server.setNoDelay(true);

}


void checkTelnet(void)
{
  uint8_t i;

  //check if there are any new clients
  if (server.hasClient()){
    for(i = 0; i < MAX_SRV_CLIENTS; i++)
    {
      //find free/disconnected spot
      if (!serverClients[i] || !serverClients[i].connected())
      {
        if(serverClients[i]) serverClients[i].stop();
        serverClients[i] = server.available();
        Serial1.print("New client: "); Serial1.print(i);
        continue;
      }
    }
    //no free/disconnected spot so reject
    WiFiClient serverClient = server.available();
    serverClient.stop();
  }

  //check clients for data
  for(i = 0; i < MAX_SRV_CLIENTS; i++)
  {
    if (serverClients[i] && serverClients[i].connected())
    {
      if(serverClients[i].available())
      {
        //get data from the telnet client and push it to the UART
        while(serverClients[i].available())
        {
            int buf = serverClients[i].read();
            Serial.write(buf);
            if (buf == 0x69)
            {
                resetIMU();
            }
            else if (buf == 0x6C)
            {
                resetLux();
            }
        }
      }
    }
  }

  //check UART for data
  if (Serial.available())
  {
    size_t len = Serial.available();
    uint8_t sbuf[len];
    Serial.readBytes(sbuf, len);
    //push UART data to all connected telnet clients
    for(i = 0; i < MAX_SRV_CLIENTS; i++)
    {
      if (serverClients[i] && serverClients[i].connected())
      {
        serverClients[i].write(sbuf, len);
        delay(1);
      }
    }
  }
}


void writeTelnet(String sendData)
{
    uint8_t i;
    uint8_t foo[sendData.length()];

    sendData.toCharArray((char *)foo, sendData.length());
    // Push UART data to all connected telnet clients
    for(i = 0; i < MAX_SRV_CLIENTS; i++)
    {
        if (serverClients[i] && serverClients[i].connected())
        {
            serverClients[i].write(foo, sendData.length());
            delay(1);
        }
    }
}


void PushEvent()
{
    String fIMU = "";
    String fLux = "";
    String Data = "";

    if ((flagIMU == flagLastIMU) && (flagLux == flagLastLux))
    {
        return;
    }

    flagLastIMU = flagIMU;
    flagLastLux = flagLux;

    if (flagIMU)
    {
        fIMU = "Yes";
    }
    else
    {
        fIMU = "No";
    }

    if (flagLux)
    {
        fLux = "Yes";
    }
    else
    {
        fLux = "No";
    }

    HTTPClient http;
    if (!http.begin("https://api.instapush.im/post", CertThumb))
    {
        Serial.println("connection failed");
        return;
    }
    http.addHeader("Content-Type", "application/x-www-form-urlencoded");
    http.addHeader("x-instapush-appid", AppId);
    http.addHeader("x-instapush-appsecret", AppSecret);

    Serial.print("Sending push: ");
    Data = (String)"{\"event\":\"WasherEvent\",\"trackers\":{\"OOB\":\"" + fIMU + (String)"\", \"Light\":\"" + fLux + (String)"\"}}";
    http.POST(Data);
    delay(10);
    Serial.println(Data);
    http.writeToStream(&Serial1);
    http.writeToStream(&Serial);
    http.end();
}


