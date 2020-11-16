 
//Include################-------------------########################
#include "PMS.h"
#include <Wire.h>
#include <Adafruit_BME280.h>
#include <WiFi.h>
#include <FirebaseESP32.h>

//Define################--------------------########################

#define SEALEVELPRESSURE_HPA (1013.25)

// ESP32 --> Pantower PMS7003
// 25    --> RX
// 26    --> TX
// GND   --> GND
#define RXD2 25
#define TXD2 26

#define FIREBASE_HOST "(FirebaseDBname).firebaseio.com/" 
#define FIREBASE_AUTH "(Auth KEY)"


#define WIFI_SSID "SSID"
#define WIFI_PASSWORD "PASSWORD TO WIFI"

//Define FirebaseESP32 data object
FirebaseData firebaseData;
// Root Path
String path = "/ESP32_Device";
FirebaseJson json;
void printResult(FirebaseData &data);

HardwareSerial SerialPMS(1);
PMS pms(SerialPMS);
PMS::DATA data;
Adafruit_BME280 bme; // I2C

uint32_t delayTime = 20000;
uint32_t delayMS = 1500;

float prev_pms1;
float prev_pms2;
float prev_pms10;

float prev_temp;
float prev_humidity;
float prev_pressure;
float prev_approx;

void initWifi();
void printValuesBMEPMS();

void setup()
{
  Serial.begin(9600);
  initWifi();
  SerialPMS.begin(9600, SERIAL_8N1, RXD2, TXD2);
}

void loop()
{
  printValuesBMEPMS();
}

void updateDataPms1(float pmsData1)
{
  if (prev_pms1 != pmsData1)
  {
    String tempString = "";
    tempString += (int)pmsData1;
    tempString += "ug/m3";
    Serial.println("Sending -> " + tempString);
    prev_pms1 = pmsData1;

    Firebase.setDouble(firebaseData, path + "/PMS7003/pmsData1", pmsData1);
  }
}

void updateDataPms2(float pmsData2)
{
  if (prev_pms2 != pmsData2)
  {
    String tempString = "";
    tempString += (int)pmsData2;
    tempString += "ug/m3";
    Serial.println("Sending -> " + tempString);
    prev_pms2 = pmsData2;

    Firebase.setDouble(firebaseData, path + "/PMS7003/pmsData2", pmsData2);
  }
}

void updateDataPms10(float pmsData10)
{
  if (prev_pms10 != pmsData10)
  {
    String tempString = "";
    tempString += (int)pmsData10;
    tempString += "ug/m3";
    Serial.println("Sending -> " + tempString);
    prev_pms10 = pmsData10;

    Firebase.setDouble(firebaseData, path + "/PMS7003/pmsData10", pmsData10);
  }
}
void updateTemp(float tempV)
{
  if (prev_temp != tempV)
  {
    String tempString = "";
    tempString += (int)tempV;
    tempString += "C";
    Serial.println("Sending -> " + tempString);
    prev_temp = tempV;

    Firebase.setDouble(firebaseData, path + "/BME280/temperature", tempV);
  }
}

void updateHumidity(float humidity)
{
  if (prev_humidity != humidity)
  {
    String humidityString = "";
    humidityString += (int)humidity;
    humidityString += "%";
    Serial.println("Sending -> " + humidityString);
    prev_humidity = humidity;

    Firebase.setDouble(firebaseData, path + "/BME280/humidity", humidity);
  }
}
void updateApprox(float approx)
{
  if (prev_approx != approx)
  {
    String approxString = "";
    approxString += (int)approx;
    approxString += "m";
    Serial.println("Sending -> " + approxString);
    prev_approx = approx;

    Firebase.setDouble(firebaseData, path + "/BME280/approx", approx);
  }
}
void updatePressure(float pressure)
{
  if (prev_pressure != pressure)
  {
    String pressureString = "";
    pressureString += (int)pressure;
    pressureString += "hPa";
    Serial.println("Sending -> " + pressureString);
    prev_pressure = pressure;

    Firebase.setDouble(firebaseData, path + "/BME280/pressure", pressure);
  }
}

void printValuesBMEPMS()
{
  if (pms.read(data) && bme.begin(0x76))
  {
    Serial.println("--------------------------------------------------------");

    float pmsData1 = data.PM_AE_UG_1_0;
    Serial.print("PM 1.0 (ug/m3): ");
    Serial.println(pmsData1);
    Serial.print("ug/m3");
    updateDataPms1(pmsData1);
    Serial.println();

    float pmsData2 = data.PM_AE_UG_2_5;
    Serial.print("PM 2.5 (ug/m3): ");
    Serial.println(pmsData2);
    Serial.print("ug/m3");
    updateDataPms2(pmsData2);
    Serial.println();

    float pmsData10 = data.PM_AE_UG_10_0;
    Serial.print("PM 10.0 (ug/m3): ");
    Serial.println(pmsData10);
    Serial.println(" ug/m3");
    updateDataPms10(pmsData10);
    Serial.println();

    Serial.println("--------------------------------------------------------");

    float tempV = bme.readTemperature();
    Serial.print("Temperature = ");
    Serial.print(tempV);
    Serial.print(" *C ");
    updateTemp(tempV);
    Serial.println();

    float pressure = bme.readPressure() / 100.0F;
    Serial.print("Pressure = ");
    Serial.print(pressure);
    Serial.print(" hPa ");
    updatePressure(pressure);
    Serial.println();

    float approx = bme.readAltitude(SEALEVELPRESSURE_HPA);
    Serial.print("Approx. Altitude = ");
    Serial.print(approx);
    Serial.print(" m ");
    updateApprox(approx);
    Serial.println();

    float humidity = bme.readHumidity();
    Serial.print("Humidity = ");
    Serial.print(humidity);
    Serial.print(" % ");
    updateHumidity(humidity);

    Serial.println("--------------------------------------------------------");

    delay(delayTime);
  }
}
void initWifi()
{
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(delayMS);
  }
  Serial.println();
  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP());
  Serial.println();

  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
  Firebase.reconnectWiFi(true);

  //Set database read timeout to 1 minute (max 15 minutes)
  Firebase.setReadTimeout(firebaseData, 1000 * 60);
  //tiny, small, medium, large and unlimited.
  //Size and its write timeout e.g. tiny (1s), small (10s), medium (30s) and large (60s).
  Firebase.setwriteSizeLimit(firebaseData, "tiny");
}
