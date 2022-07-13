#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include "DHT.h"
#include "DHT_U.h"
#include <Adafruit_Sensor.h>
#include "esp_adc_cal.h"
#include <ArduinoJson.h>
#include <RTClib.h>
#include <WiFiManager.h> // https://github.com/tzapu/WiFiManager
#include <Preferences.h>

#define SOIL_MOISTURE_PIN 33 //Adc Pin
#define SYS_LED_PIN 2
#define DHT_PIN 4
#define LOW_WATER_PIN 16
#define LED_PIN 17
#define PUMP_PIN 5
#define FAN_PIN 18
#define LAMP_PIN 23

//#define DEBUG
#define DHTTYPE DHT22 // DHT22
#define CALENDAR_SIZE 400
#define DS1338_ADDR 0x68
#define FILTER_LEN 10
#define DEBOUNCE_DELAY 30
#define SW_VERSION "v1.0"
#define WATER 1
#define FAN 2
#define LED 3
#define LAMP 4

//structs
typedef struct
{
  uint16_t dayofmin;
  uint16_t action;
} calendar;

typedef struct
{
  uint16_t index;
  uint16_t length;
  uint8_t actionPin;
  uint8_t type;
  uint8_t status;
} calendarInfo;

//Function prototypes
void mqttCallback(char *topic, byte *message, unsigned int length);
void setup_wifi();
void reconnect();
uint32_t readADCCal(int ADC_Raw);
uint32_t calculateAvg(int sample);
void sort_calendar(calendar *cal, uint16_t length);
uint16_t jsonCalendarParse(const char *input, unsigned int inputLength, calendar *itemCalendar);
bool isDateEqualCalendar(const DateTime &dt, uint16_t calendarItem);
void calendarPerformAction(DateTime nowDate, calendarInfo *itemInfo, calendar *itemCalendar);
void publishStatus(calendarInfo *itemInfo);
bool isLowWater();
int openWaterPump();
void readSavedData();
void getDateString(char *dateBuffer, const DateTime &dt);
void getDeviceID(char *deviceID);

const char *ssid = "RedmiMk";
const char *password = "01011980";

//const char* mqtt_server = "34.211.84.46";
const char *mqtt_server = "broker.emqx.io"; //"broker.emqx.io";

DHT dht(DHT_PIN, DHTTYPE);
RTC_DS1307 rtc;
WiFiClient espClient;
WiFiManager wm;
PubSubClient client(espClient);
Preferences preferences;
DynamicJsonDocument doc(6144);
calendar waterCalendar[CALENDAR_SIZE], fanCalendar[CALENDAR_SIZE], ledCalendar[CALENDAR_SIZE], lampCalendar[CALENDAR_SIZE];
calendarInfo waterInfo, fanInfo, ledInfo, lampInfo;
DateTime nowDate, lastDate;

uint32_t adcBuffer[FILTER_LEN] = {0};
uint32_t soilMoistureRaw, soilMoisture;
int filterIndex = 0, loopCount = 0;
char dateBuffer[25], deviceID[20];
unsigned long lastMsg = 0, wlCheckTime = 0, portalTimeout = 0;
bool lowWaterCheck, mqttStatus;
String preStrCon, preStrMon;

float temperature = 0, humidity = 0;
uint16_t mOfWeek, lastMinOfWeek;

void setup()
{
  Serial.begin(115200);

  getDeviceID(deviceID);
  Serial.print("deviceID:");
  Serial.println(deviceID);

  preStrCon = String("doa/") + String(deviceID) + String("/control/");
  preStrMon = String("doa/") + String(deviceID) + String("/monitor/");

  pinMode(SYS_LED_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(PUMP_PIN, OUTPUT);
  pinMode(FAN_PIN, OUTPUT);

  pinMode(LOW_WATER_PIN, INPUT);
  pinMode(SOIL_MOISTURE_PIN, INPUT);

  digitalWrite(LED_PIN, HIGH);  //Active State:lOW
  digitalWrite(PUMP_PIN, HIGH); //Active State:lOW
  digitalWrite(FAN_PIN, HIGH);  //Active State:lOW

  //================================
  WiFi.mode(WIFI_STA); // explicitly set mode, esp defaults to STA+AP
  //reset settings - wipe credentials for testing
  //wm.resetSettings();

  wm.setConfigPortalBlocking(false);
  wm.setConnectTimeout(5);
  wm.setSaveConnectTimeout(5);
  //wm.setConfigPortalTimeout(180);

  //automatically connect using saved credentials if they exist
  //If connection fails it starts an access point with the specified name
  String apName = String("DOA_") + String(deviceID);
  if (wm.autoConnect(apName.c_str())) //wm.autoConnect("AutoConnectAP")
  {
    Serial.println("connected... :)");
  }
  else
  {
    Serial.println("Configportal running");
  }
  //====================================

  dht.begin();     //Initialize the DHT sensor
  readSavedData(); //Read saved calendar data

  //Start RTC
  Wire.begin();
  rtc.begin();
  lastDate = rtc.now();

  //setup_wifi();
  client.setBufferSize(4096);
  client.setServer(mqtt_server, 1883);
  client.setCallback(mqttCallback);
}

void loop()
{
  wm.process(); //Wifi manager

  if (millis() - wlCheckTime > 10000) // every 10 seconds
  {
    wlCheckTime = millis();
    if (WiFi.status() != WL_CONNECTED)
    {
      //if AP mode is open
      if (WiFi.getMode() != WIFI_STA)
      {
        if (millis() - portalTimeout > 180000) // every 180 seconds
        {
          portalTimeout = millis();
          WiFi.mode(WIFI_STA);
          Serial.println(F("Changing Wifi mode to WIFI_STA"));
        }
      }
      else //WiFi.getMode() == WIFI_STA
      {
        Serial.println(F("No Wifi, try reconnect"));
        WiFi.reconnect();
      }
    }
  }

  if (!client.connected())
  {
    reconnect();
  }
  //mqttStatus = client.connected();
  if (mqttStatus)
  {
    digitalWrite(SYS_LED_PIN, HIGH);
    client.loop();
  }
  else
  {
    digitalWrite(SYS_LED_PIN, LOW);
  }

  //Serial.print("Test ");
  long now = millis();
  if (now - lastMsg > 5000)
  {
    lastMsg = now;

    //If Low water level,  close the pump
    lowWaterCheck = isLowWater();
    if (lowWaterCheck)
    {
      digitalWrite(PUMP_PIN, HIGH); //Close the pump
      waterInfo.status = 0;
      publishStatus(&waterInfo);
    }

    if (loopCount % 6 == 0) //Her 30sn bir
    {
      humidity = dht.readHumidity();
      if (isnan(humidity))
      {
        humidity = dht.readHumidity();
      }
      temperature = dht.readTemperature();
      if (isnan(temperature))
      {
        temperature = dht.readTemperature();
      }

      Serial.println(F("#=== SENSOR BILGILERI ===="));
      Serial.print(F("Humidity = "));
      Serial.println(humidity);
      Serial.print(F("Temperature = "));
      Serial.println(temperature);

      Serial.print(F("lowWaterCheck = "));
      Serial.println(lowWaterCheck);

      for (int i = 0; i < FILTER_LEN; i++)
      {
        soilMoistureRaw = readADCCal(analogRead(SOIL_MOISTURE_PIN)); //Make calibration correction
        soilMoisture = calculateAvg(soilMoistureRaw);
      }
      Serial.print(F("Soil Moisture = "));
      Serial.println(soilMoisture);
    }

    if (loopCount % 2 == 0) //Her 10sn bir
    {
      nowDate = rtc.now(); //Get RTC Time
      getDateString(dateBuffer, nowDate);
      mOfWeek = 1440 * nowDate.dayOfTheWeek() + 60 * nowDate.hour() + nowDate.minute();
      lastMinOfWeek = 1440 * lastDate.dayOfTheWeek() + 60 * lastDate.hour() + lastDate.minute();

      //Eger tarih gecmis bir tarih olarak ayarlandi veya hafta yeniden baslamis ise
      if (lastDate > nowDate || lastMinOfWeek > mOfWeek)
      {
        waterInfo.index = 0;
        fanInfo.index = 0;
        ledInfo.index = 0;
        lampInfo.index = 0;
      }
      lastDate = nowDate;

      Serial.print(F("minuteOfWeek: "));
      Serial.println(mOfWeek);
      Serial.println(dateBuffer);

      Serial.println(F("#=== TAKVIM KONTROLLERI ===="));
      Serial.println(F(" =>Sulama takvimi:"));
      calendarPerformAction(nowDate, &waterInfo, waterCalendar); //Perform calendar actions

      Serial.println(F(" =>Fan takvimi:"));
      calendarPerformAction(nowDate, &fanInfo, fanCalendar);

      Serial.println(F(" =>Led takvimi:"));
      calendarPerformAction(nowDate, &ledInfo, ledCalendar);

      Serial.println(F(" =>UV Lamba takvimi:"));
      calendarPerformAction(nowDate, &lampInfo, lampCalendar);
    }

    if (mqttStatus && loopCount == 0) //Her 60sn de bir defa
    {
      // Publish messages
      client.publish((preStrMon + String("datetime")).c_str(), (const char *)dateBuffer);
      client.publish((preStrMon + String("version")).c_str(), SW_VERSION);
      client.publish((preStrMon + String("humidity")).c_str(), String(humidity).c_str());
      client.publish((preStrMon + String("temperature")).c_str(), String(temperature).c_str());
      client.publish((preStrMon + String("soilmoisture")).c_str(), String(soilMoisture).c_str());
      client.publish((preStrMon + String("waterlevel")).c_str(), String(!lowWaterCheck).c_str());
    }
    loopCount++;
    if (loopCount >= 12)
    {
      loopCount = 0;
    }
  }
}

void setup_wifi()
{
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print(F("Connecting to "));
  Serial.println(ssid);

  //WiFi.begin(ssid, password);
  WiFi.begin();

  if (WiFi.status() != WL_CONNECTED) //while
  {
    delay(500);
    Serial.println(F("Wifi not connected"));
  }
  else
  {
    Serial.println("");
    Serial.println(F("WiFi connected"));
    Serial.println(F("IP address: "));
    Serial.println(WiFi.localIP());
    //digitalWrite(BLUE_LED, HIGH);
  }
}

void mqttCallback(char *topic, byte *message, unsigned int length)
{
  Serial.print(F("Message arrived on topic: "));
  Serial.print(topic);
  Serial.print(F("  Message: "));
  String messageTemp;
  String topicString = String(topic);

  if (!(topicString == (preStrCon + String("water_calendar")) || topicString == (preStrCon + String("fan_calendar")) ||
        topicString == (preStrCon + String("led_calendar")) || topicString == (preStrCon + String("lamp_calendar"))))
  {
    for (int i = 0; i < length; i++)
    {
      Serial.print((char)message[i]);
      {
        messageTemp += (char)message[i];
      }
    }
  }

  Serial.println();
  if (topicString == (preStrCon + String("fan")))
  {
    Serial.print(F("Changing fan output: "));
    if (messageTemp == "on")
    {
      Serial.println("on");
      digitalWrite(FAN_PIN, LOW);
      //client.subscribe(preStrCon.c_str());
      fanInfo.status = 1;
      client.publish((preStrMon + String("fan")).c_str(), "on");
    }
    else if (messageTemp == "off")
    {
      Serial.println("off");
      digitalWrite(FAN_PIN, HIGH);
      fanInfo.status = 0;
      client.publish((preStrMon + String("fan")).c_str(), "off");
    }
  }
  else if (topicString == (preStrCon + String("led")))
  {
    Serial.print(F("Changing led output: "));
    if (messageTemp == "on")
    {
      Serial.println("on");
      digitalWrite(LED_PIN, LOW);
      ledInfo.status = 1;
      client.publish((preStrMon + String("led")).c_str(), "on");
    }
    else if (messageTemp == "off")
    {
      Serial.println("off");
      digitalWrite(LED_PIN, HIGH);
      ledInfo.status = 0;
      client.publish((preStrMon + String("led")).c_str(), "off");
    }
  }
  else if (topicString == (preStrCon + String("water")))
  {
    Serial.print(F("Changing water output: "));
    if (messageTemp == "on")
    {
      openWaterPump();
      if (lowWaterCheck)
      {
        client.publish((preStrMon + String("waterlevel")).c_str(), String(!lowWaterCheck).c_str());
        client.publish((preStrMon + String("water")).c_str(), "off");
        waterInfo.status = 0;
      }
      else
      {
        client.publish((preStrMon + String("water")).c_str(), "on");
        waterInfo.status = 1;
      }
    }
    else if (messageTemp == "off")
    {
      Serial.println("off");
      digitalWrite(PUMP_PIN, HIGH);
      client.publish((preStrMon + String("water")).c_str(), "off");
      waterInfo.status = 0;
    }
  }
  else if (topicString == (preStrCon + String("calendar")))
  {
    if (messageTemp == "reset")
    {
      Serial.println(F("Reseting calendars:"));
      waterInfo.length = 0;
      fanInfo.length = 0;
      ledInfo.length = 0;
      lampInfo.length = 0;
      preferences.begin("doa", false);
      preferences.clear();
      preferences.end();
    }
  }
  else if (topicString == (preStrCon + String("datetime")))
  {
    Serial.println(F("Adjust RTC datetime: "));
    DateTime newDateTime = DateTime(messageTemp.c_str());
    if (newDateTime > DateTime("2021-08-22T07:30:00"))
    {
      rtc.adjust(newDateTime);
      //Serial.println(newDateTime.toString("yyyy-MM-ddTHH:mm:ss"));
      Serial.println(F(" =>Datetime is adjusted."));
    }
    else
    {
      Serial.println(F(" =>Requested date is expired"));
    }
  }
  else if (topicString == (preStrCon + String("water_calendar")))
  {
    Serial.println(F("Water Calendar Output: "));
    waterInfo.length = jsonCalendarParse((char *)message, length, waterCalendar);
    if (waterInfo.length > 0)
    {
      waterInfo.index = 0;
      preferences.begin("doa", false);
      preferences.putUShort("waterLength", waterInfo.length);
      preferences.putBytes("water", waterCalendar, waterInfo.length * sizeof(calendar));
      preferences.end();
    }
  }
  else if (topicString == (preStrCon + String("fan_calendar")))
  {
    Serial.println(F("Fan Calendar Output: "));
    fanInfo.length = jsonCalendarParse((char *)message, length, fanCalendar);
    if (fanInfo.length > 0)
    {
      fanInfo.index = 0;
      preferences.begin("doa", false);
      preferences.putUShort("fanLength", fanInfo.length);
      preferences.putBytes("fan", fanCalendar, fanInfo.length * sizeof(calendar));
      preferences.end();
    }
  }
  else if (topicString == (preStrCon + String("led_calendar")))
  {
    Serial.println(F("Led Calendar Output: "));
    ledInfo.length = jsonCalendarParse((char *)message, length, ledCalendar);
    if (ledInfo.length > 0)
    {
      ledInfo.index = 0;
      preferences.begin("doa", false);
      preferences.putUShort("ledLength", ledInfo.length);
      preferences.putBytes("led", ledCalendar, ledInfo.length * sizeof(calendar));
      preferences.end();
    }
  }
  else if (topicString == (preStrCon + String("lamp_calendar")))
  {
    Serial.println(F("Lamp Calendar Output: "));
    lampInfo.length = jsonCalendarParse((char *)message, length, lampCalendar);
    if (lampInfo.length > 0)
    {
      lampInfo.index = 0;
      preferences.begin("doa", false);
      preferences.putUShort("lampLength", lampInfo.length);
      preferences.putBytes("lamp", lampCalendar, lampInfo.length * sizeof(calendar));
      preferences.end();
    }
  }
}

void reconnect()
{
  delay(5000);
  mqttStatus = false;
  if (WiFi.status() == WL_CONNECTED)
  {
    if (!client.connected())
    {
      //digitalWrite(BLUE_LED, LOW);
      Serial.print(F("Attempting MQTT connection..."));
      // Attempt to connect
      if (client.connect(deviceID))
      {
        Serial.println(F("connected"));
        // Subscribe
        String subscribeStr = String("doa/") + String(deviceID) + String("/control/#"); //client.subscribe("doa/dev123/control/#");
        client.subscribe(subscribeStr.c_str());

        mqttStatus = true;
        publishStatus(&waterInfo);
        publishStatus(&fanInfo);
        publishStatus(&ledInfo);
        publishStatus(&lampInfo);
      }
      else
      {
        Serial.print(F("failed, rc="));
        Serial.print(client.state());
      }
    }
  }
}

uint32_t readADCCal(int ADC_Raw)
{
  esp_adc_cal_characteristics_t adc_chars;

  esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, &adc_chars);
  return (esp_adc_cal_raw_to_voltage(ADC_Raw, &adc_chars));
}

uint32_t calculateAvg(int sample)
{
  int i = 0;
  uint32_t sum = 0;

  adcBuffer[filterIndex++] = sample;
  if (filterIndex == FILTER_LEN)
  {
    filterIndex = 0;
  }
  for (i = 0; i < FILTER_LEN; i++)
  {
    sum += adcBuffer[i];
  }
  return (sum / FILTER_LEN);
}

void sort_calendar(calendar *cal, uint16_t length)
{
  calendar temp;
  //Sort the array in ascending order
  for (int i = 0; i < length; i++)
  {
    for (int j = i + 1; j < length; j++)
    {
      if (cal[i].dayofmin > cal[j].dayofmin)
      {
        temp = cal[i];
        cal[i] = cal[j];
        cal[j] = temp;
      }
    }
  }
}

uint16_t jsonCalendarParse(const char *input, unsigned int inputLength, calendar *itemCalendar)
{
  Serial.println(F("Json Calendar Parse Started:"));
  DeserializationError error = deserializeJson(doc, input, inputLength);

  if (error)
  {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.f_str());
    return 0;
  }

  uint16_t i = 0, j = 0;
  for (JsonObject calendarItem : doc["calendar"].as<JsonArray>())
  {
    uint16_t dayofweek = calendarItem["dofw"];
    uint16_t hour = calendarItem["h"];
    uint16_t minute = calendarItem["m"];
    uint16_t repeat = calendarItem["r"];
    uint16_t action = calendarItem["a"];
    Serial.printf("dayofweek:%d, hour:%d, minute:%d, repeat:%d, action:%d\n", dayofweek, hour, minute, repeat, action);
    if (i < CALENDAR_SIZE && hour < 24 && minute < 60 && action < 2)
    {
      //repeat, 0: Haftanın bir günü, 1: Her gün, 2:Hafta içi, 3:Hafta sonu
      if (repeat == 0) //0: Haftanın bir günü
      {
        itemCalendar[i].dayofmin = 1440 * dayofweek + 60 * hour + minute;
        itemCalendar[i].action = action;
#ifdef DEBUG
        Serial.printf(" =>Index:%d, Dayofmin:%d, action:%d\n", i, itemCalendar[i].dayofmin, itemCalendar[i].action);
#endif
        i++;
      }
      else if (repeat == 1) //1: haftanın her günü
      {
        for (j = 0; j < 7 && i < CALENDAR_SIZE; j++)
        {
          itemCalendar[i].dayofmin = 1440 * j + 60 * hour + minute;
          itemCalendar[i].action = action;
#ifdef DEBUG
          Serial.printf(" =>Index:%d, Dayofmin:%d, action:%d\n", i, itemCalendar[i].dayofmin, itemCalendar[i].action);
#endif
          i++;
        }
      }
      else if (repeat == 2) //2:Hafta içi günler
      {
        for (j = 1; j < 6 && i < CALENDAR_SIZE; j++)
        {
          itemCalendar[i].dayofmin = 1440 * j + 60 * hour + minute;
          itemCalendar[i].action = action;
#ifdef DEBUG
          Serial.printf(" =>Index:%d, Dayofmin:%d, action:%d\n", i, itemCalendar[i].dayofmin, itemCalendar[i].action);
#endif
          i++;
        }
      } //3:Hafta sonu günler
      else if (repeat == 3)
      {
        for (j = 0; j < 7 && i < CALENDAR_SIZE; j = j + 6) //j=0 Pazar,  j=6 Cumartesi
        {
          itemCalendar[i].dayofmin = 1440 * j + 60 * hour + minute;
          itemCalendar[i].action = action;
#ifdef DEBUG
          Serial.printf(" =>Index:%d, Dayofmin:%d, action:%d\n", i, itemCalendar[i].dayofmin, itemCalendar[i].action);
#endif
          i++;
        }
      }
    }
  }

  uint16_t itemCalendarLength = i;
  sort_calendar(itemCalendar, itemCalendarLength);
  Serial.print(F("### SORT itemCalendar, itemCalendarLength="));
  Serial.println(itemCalendarLength);
  for (i = 0; i < itemCalendarLength; i++)
  {
    Serial.printf(" ==>Index:%d, Dayofmin:%d, action:%d\n", i, itemCalendar[i].dayofmin, itemCalendar[i].action);
  }
  return itemCalendarLength;
}

bool isDateEqualCalendar(const DateTime &dt, uint16_t calendarItem)
{
  uint16_t minuteOfWeek = 1440 * dt.dayOfTheWeek() + 60 * dt.hour() + dt.minute();
  return (minuteOfWeek == calendarItem);
}

void calendarPerformAction(DateTime nowDate, calendarInfo *itemInfo, calendar *itemCalendar)
{
  uint8_t action;
  for (int i = itemInfo->index; i < itemInfo->length; i++)
  {
    if (isDateEqualCalendar(nowDate, itemCalendar[i].dayofmin))
    {
      itemInfo->index = i + 1;
      action = (itemCalendar[i].action > 0) ? LOW : HIGH; //LOW: Open, HIGH:Close
      if (itemInfo->actionPin == PUMP_PIN)
      {
        //Sulama komutu geldi ve yeterli su var ise
        if (itemCalendar[i].action == 1)
        {
          openWaterPump();
        }
        else
        {
          digitalWrite(PUMP_PIN, HIGH);
          Serial.println(F("Close Pump"));
          itemInfo->status = 0;
        }
      }
      else //if (itemInfo->actionPin == FAN_PIN)
      {
        digitalWrite(itemInfo->actionPin, action);
        itemInfo->status = itemCalendar[i].action;
      }
      publishStatus(itemInfo);
      Serial.printf("  *Action performed.Index=%d, Action:%d, Pin:%d\n", itemInfo->index, itemCalendar[i].action, itemInfo->actionPin);
    }
  }
}

void publishStatus(calendarInfo *itemInfo)
{
  if (mqttStatus)
  {
    String typeStr;

    switch (itemInfo->type)
    {
    case WATER:
      typeStr = String("water");
      break;
    case FAN:
      typeStr = String("fan");
      break;
    case LED:
      typeStr = String("led");
      break;
    case LAMP:
      typeStr = String("lamp");
      break;
    default:
      break;
    }

    //Publish  status
    if (itemInfo->status == 0)
    {
      client.publish((preStrMon + typeStr).c_str(), "off");
    }
    else
    {
      client.publish((preStrMon + typeStr).c_str(), "on");
    }
  }
}

bool isLowWater()
{
  int status = 0;
  int lowWater1 = digitalRead(LOW_WATER_PIN);
  delay(DEBOUNCE_DELAY); //Delay 30ms for debounce
  int lowWater2 = digitalRead(LOW_WATER_PIN);
  if (lowWater1 == lowWater2 && lowWater1 == HIGH)
  {
    //lowWater1 = 0 durumunda şamandıra boş konumunda
    status = false;
  }
  else
  {
    status = true;
  }
  return status;
}

int openWaterPump()
{
  int status;

  lowWaterCheck = isLowWater();
  if (lowWaterCheck)
  {
    digitalWrite(PUMP_PIN, HIGH); //Close the pump
    Serial.println(F("  Low water alarm. Not starting Pump"));
    status = 0;
    waterInfo.status = 0;
  }
  else
  {
    Serial.println("Pump on");
    digitalWrite(PUMP_PIN, LOW); //Open the pump
    status = 1;
    waterInfo.status = 1;
  }

  return status;
}

void readSavedData()
{
  preferences.begin("doa", false);
  waterInfo.index = 0;
  waterInfo.actionPin = PUMP_PIN;
  waterInfo.status = 0;
  waterInfo.type = WATER;
  waterInfo.length = preferences.getUShort("waterLength", 0);

  fanInfo.index = 0;
  fanInfo.actionPin = FAN_PIN;
  fanInfo.status = 0;
  fanInfo.type = FAN;
  fanInfo.length = preferences.getUShort("fanLength", 0);

  ledInfo.index = 0;
  ledInfo.actionPin = LED_PIN;
  ledInfo.status = 0;
  ledInfo.type = LED;
  ledInfo.length = preferences.getUShort("ledLength", 0);

  lampInfo.index = 0;
  lampInfo.actionPin = LAMP_PIN;
  lampInfo.status = 0;
  lampInfo.type = LAMP;
  lampInfo.length = preferences.getUShort("lampLength", 0);

  Serial.printf("waterLength:%d, fanLength:%d, ledLength:%d, lampLength:%d\n",
                waterInfo.length, fanInfo.length, ledInfo.length, lampInfo.length);

  if (waterInfo.length > 0)
  {
    preferences.getBytes("water", waterCalendar, waterInfo.length * sizeof(calendar));
  }
  if (fanInfo.length > 0)
  {
    preferences.getBytes("fan", fanCalendar, fanInfo.length * sizeof(calendar));
  }
  if (ledInfo.length > 0)
  {
    preferences.getBytes("led", ledCalendar, ledInfo.length * sizeof(calendar));
  }
  if (lampInfo.length > 0)
  {
    preferences.getBytes("lamp", lampCalendar, lampInfo.length * sizeof(calendar));
  }

  Serial.println(F("=== Read stored calendar values ==="));
  int i;

  Serial.println(F(" =>Water caledar values ==="));
  for (i = 0; i < waterInfo.length; i++)
  {
    Serial.printf("%d, %d - ", waterCalendar[i].dayofmin, waterCalendar[i].action);
  }

  Serial.println();
  Serial.println(F(" =>Fan caledar values ==="));
  for (i = 0; i < fanInfo.length; i++)
  {
    Serial.printf("%d, %d - ", fanCalendar[i].dayofmin, fanCalendar[i].action);
  }

  Serial.println();
  Serial.println(F(" =>Led caledar values ==="));
  for (i = 0; i < ledInfo.length; i++)
  {
    Serial.printf("%d, %d - ", ledCalendar[i].dayofmin, ledCalendar[i].action);
  }

  Serial.println();
  Serial.println(F(" =>Lamp caledar values ==="));
  for (i = 0; i < lampInfo.length; i++)
  {
    Serial.printf("%d, %d - ", lampCalendar[i].dayofmin, lampCalendar[i].action);
  }

  preferences.end();

  // === jsonCalendarParse Test ====
  // const char *input = "{\"calendar\":[{\"dofw\":1,\"h\":22,\"m\":26,\"r\":0,\"a\":1},{\"dofw\":1,\"h\":22,\"m\":27,\"r\":0,\"a\":0},{\"dofw\":1,\"h\":22,\"m\":28,\"r\":0,\"a\":1},{\"dofw\":1,\"h\":22,\"m\":29,\"r\":0,\"a\":0},{\"dofw\":1,\"h\":22,\"m\":30,\"r\":0,\"a\":1},{\"dofw\":1,\"h\":22,\"m\":31,\"r\":0,\"a\":0},{\"dofw\":1,\"h\":22,\"m\":32,\"r\":0,\"a\":1},{\"dofw\":1,\"h\":22,\"m\":33,\"r\":0,\"a\":0}]}";
  // waterInfo.length = jsonCalendarParse(input, strlen(input), waterCalendar);
  // waterInfo.index = 0;
}

void getDateString(char *dateBuffer, const DateTime &dt)
{
  snprintf(dateBuffer, 20, "%04d-%02d-%02d %02d:%02d:%02d", dt.year(), dt.month(), dt.day(),
           dt.hour(), dt.minute(), dt.second());
}

void getDeviceID(char *deviceID)
{
  uint64_t chipid = ESP.getEfuseMac(); //The chip ID is essentially its MAC address(length: 6 bytes).
  //snprintf(deviceID, 16, "%04X%08X", (uint16_t)(chipid >> 32), (uint32_t)chipid);
  snprintf(deviceID, 16, "%02X%02X%02X%02X%02X%02X", (uint8_t)chipid, (uint8_t)(chipid >> 8),
           (uint8_t)(chipid >> 16), (uint8_t)(chipid >> 24),
           (uint8_t)(chipid >> 32), (uint8_t)(chipid >> 40));
}
