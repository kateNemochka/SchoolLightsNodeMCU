#include <ESP8266WiFi.h> //Підключення плати до Wi-Fi
#include <PubSubClient.h> //Підключення до MQTT брокера та робота з ним
#include <Ticker.h> //Таймер з перериваннями
#include <Wire.h> //Зв'язок з пристроями через інтерфейс I2C/TWI
#include <BH1750.h> //Обробка даних з датчика освітленості
#include <ArduinoJson.h> //Для обробки JSON в повідомленнях MQTT
#include <time.h>                       // time() ctime()
#include <sys/time.h>                   // struct timeval
#include "CronAlarms.h"


#define SECOND 1000 //значення 1 секунки у мілісекундах
#define dtNBR_ALARMS 50

/**************** РЕЖИМИ РОБОТИ ****************/
#define INIT "init"
//спосіб керування
#define AUTO "auto"
#define ADAPTIVE "adaptive"
#define FULL_POWER "full_power"
#define MANUAL "manual"
#define OFF "off"
#define PRESENTATION "presentation"

//розклад
#define LESSON "lesson"
#define BREAK "break"
#define PASSIVE "passive"

char* modes[] = {INIT, AUTO, ADAPTIVE, FULL_POWER, MANUAL, OFF, PRESENTATION}; //light power
char* periods[] = {LESSON, BREAK, PASSIVE}; //timeouts


/******* Дані про клас, плату, прошивку *******/
char* room = "207";
char* firwareUpdate = "20.05.2021";
char* modeUpdate = "20.05.2021";
char* deviceMode = INIT;
char* currentPeriod = LESSON;
int periodIndex = 0;
int zoneId = 1;
int rowNumber = 1;
char* roomType = "computer_class";
bool isDayOff = false;
int timezone = 3;
int dst = 0;


/*********** Змінні стану системи (Задаються через MQTT)  **********/
//Датчики руху
//набори значень для режимів LESSON, BREAK, PASSIVE
unsigned int detectionIntervals[] = {240, 30, 30}; //інтервал перевірки спрацьовування датчика руху
unsigned int lightTimeouts[] = {600, 60, 30}; //час, після якого вимикається світильник після закінчення руху

//Оновлення MQTT
unsigned int sensorsUpdateTimeout = 60; //інтервал між сповіщеннями про дані датчиків
unsigned int statusUpdateTimeout = 3600; //інтервал між сповіщеннями про статус контролера
//Час роботи нестандартного режиму (не AUTO)
unsigned int modeTimeout = 3600; //час роботи

//Диммер
int maxDimValue = 9990; //максимальне допустиме значення потужності для диммера
int minDimValue = 150; //мінімальне допустиме значення потужності для диммера
int customDimValue = 0; //власноруч встановлене значення

//Рівень освітленості та норми
unsigned long minLux = 300; //мінімальний рівень освітленості відповідно до норм
unsigned long deltaLux = 150; //діапазон між мінімальним та максимальним рівнем освітленості


/******************** Піни ********************/
const int DOPPLER_PIN = D7;
const int LUX_SDA_PIN = D2; //A4 - 3.3 v
const int LUX_SCL_PIN = D1; //A5
const int DIMMER_DIM_PIN = D5; //M1
const int DIMMER_INTERRUPT_PIN = D6; //M2

/******************** WiFi ********************/
// SSID мережі Wi-Fi та пароль до неї
const char* WIFI_SSID = "YOUR_WIFI";
const char* WIFI_PASSWORD = "YOUR_PASSWORD";

/******************** MQTT ********************/
// Дані MQTT сервера
const char* MQTT_SERVER = "MQTT_IP_ADDRESS";
const int   MQTT_PORT = 1883;
const char* MQTT_CLIENT_ID = "MQTT_CLIENT_ID"; //Назва пристрою (hostname)
const char* MQTT_USERNAME = "MQTT_USERNAME";
const char* MQTT_PASSWORD = "MQTT_PASSWORD";

/***************** MQTT Topics *****************/
//subscription
char* MQTT_TOPIC_MODE = "school/floor-2/room-207/mode";
char* MQTT_TOPIC_PERIOD = "school/period";
char* MQTT_TOPIC_SCHEDULE = "school/schedule";
char* MQTT_TOPIC_ROOMTYPE = "school/"; //+roomType
char* MQTT_TOPIC_CONFIG = "school/floor-2/room-207/config";
//publishing
char* MQTT_TOPIC_DEVICE = "school/floor-2/room-207/device";
char* MQTT_TOPIC_DATA = "school/floor-2/room-207/data";
char* MQTT_TOPIC_PARAMS = "school/floor-2/room-207/params";

/******** Робочі змінні ********/
int dimValue = 0; //значення потужності на диммері (від 0 до 10000)
boolean motionDetected = false; //виявлено рух на датчику руху (мікрохвильовий)
boolean lightsOn = false; //стан світильників (увімкнені/вимкнені)
unsigned long lux = 0; //поточний рівнь освітленості
unsigned int dimmerCounter = 0;

/******* Часові відмітки *******/
unsigned long statusTimestamp = 0; //час останнього повідомлення про статус системи
unsigned long dataTimestamp = 0; //час від оновлення даних з датчиків
unsigned long motionTimestamp = 0; //момент, від якого відраховується час спрацьовування датчика
unsigned long modeTimestamp = 0; //момент початку роботи пристрою у новому режимі


/*********** Ініціалізація об'єктів **********/
WiFiClient wifiClient;
PubSubClient mqttClient(MQTT_SERVER, MQTT_PORT, wifiClient);
BH1750 lightMeter;
Ticker dimmerChecker;
CronId id;


/************ ФУНКЦІЇ ПЕРЕРИВАННЯ ************/
/*Обробник переривання за фронтом (пін визначення фази димера)*/
void ICACHE_RAM_ATTR handleInterrupt() {
  dimmerCounter++;
  lux = lightMeter.readLightLevel();
  if (strcmp(deviceMode, FULL_POWER) == 0 & dimValue < maxDimValue) {
    dimValue += 100;
  }
  else if (strcmp(deviceMode, MANUAL) == 0) {
    dimValue = customDimValue;
  }
  else if ((!lightsOn || strcmp(deviceMode, OFF) == 0) & dimValue > 0) {
    dimValue -= 100;
  } 
  else if (lightsOn) {
    int goal = strcmp(deviceMode, PRESENTATION) == 0 ? (minLux / 10 * 6) : minLux;
    int dimStep = calculateStepValue(lux, minLux + deltaLux / 2);
    if (lux < goal + deltaLux / 10) 
      dimValue += dimStep;
    else if (lux > goal + deltaLux) 
      dimValue -= dimStep;
    else
      dimValue = dimValue;
  }
  /*Повернення значення до допустимих меж*/
  if (dimValue > maxDimValue)
    dimValue = maxDimValue;
  else if (lightsOn & dimValue < minDimValue)
    dimValue = minDimValue;
  else if ((!lightsOn || strcmp(deviceMode, OFF) == 0) & dimValue < 0)
    dimValue = 0;

  int power = 49000 - 4.75 * dimValue;
  timer1_write(power);
}

/*Обробник переривання таймера*/
void ICACHE_RAM_ATTR onTimerISR() {
  digitalWrite(DIMMER_DIM_PIN, HIGH);
  delayMicroseconds(40);
  digitalWrite(DIMMER_DIM_PIN, LOW);
  timer1_write(50000);//10мс
}

/*Перевірка роботи світильника незалежно від його стану*/
void ICACHE_RAM_ATTR checkDimmerState() {
  if (dimmerCounter == 0)
    lightsOn = false;
  dimmerCounter = 0;
}


/*Первинні налаштування*/
void setup() {
  
  /*Ініціалізація пінів та об'єктів*/
  initializeObjectsPins();
  /*Підключення до Wi-Fi та MQTT*/
  connectWifiMqtt();

  configTime(timezone * 3600, dst * 0, "pool.ntp.org", "time.nist.gov");
  Serial.println("\nWaiting for time");
  while (!time(nullptr)) {
    Serial.print(".");
    delay(1000);
  }
  Serial.println("");
  // create timers, to trigger relative to when they're created
  //Cron.create("*/15 * * * * *", setPeriodLesson, false);           // timer for every 15 seconds
  //id = Cron.create("*/2 * * * * *", setPeriodBreak, false);      // timer for every 2 seconds
  Serial.println("Ending setup...");

  /*Налаштування переривань*/
  attachInterrupt(digitalPinToInterrupt(DIMMER_INTERRUPT_PIN), handleInterrupt, RISING);
  timer1_attachInterrupt(onTimerISR);
  timer1_enable(TIM_DIV16, TIM_EDGE, TIM_SINGLE);
  dimmerChecker.attach(60, checkDimmerState);

  sendMqttMessage(MQTT_TOPIC_DEVICE);
  sendMqttMessage(MQTT_TOPIC_PARAMS);
}


/*Основний цикл*/
void loop() {
  /*Перевірка з'єднання з сервером*/
  if (!mqttClient.connected()) connectWifiMqtt();
  mqttClient.loop();
  Cron.delay();

  /*При переповненні змінної для функції millis() часовим відміткам
  присвоюється значення 0 для коректності порівняння значень
  (1 раз на +-5 днів 1 з циклів при перевірці буде коротшим)*/
  checkTimestamps();

  boolean sendDataSooner = false; //позначка для надсилання повідомлення при виявленні початку або завершення руху

  if (strcmp(deviceMode, AUTO) != 0 && millis() - modeTimestamp > modeTimeout) {
    deviceMode = AUTO;
    modeTimestamp = millis();
  }

  /*Зчитування даних датчиків*/
  lux = lightMeter.readLightLevel();
  int movementSensorValue = digitalRead(DOPPLER_PIN);

  /*Визначення наявності руху*/
  if (movementSensorValue == HIGH) {
    if (!motionDetected) {
      motionDetected = true;
      sendDataSooner = true;
    }
    motionTimestamp = millis();
  } else if (millis() - motionTimestamp > detectionIntervals[periodIndex] * SECOND) {
    motionDetected = false;
    sendDataSooner = true;
  }

  /*Керування світильниками*/
  if ((strcmp(deviceMode, FULL_POWER) == 0 
      || strcmp(deviceMode, MANUAL) == 0 
      || strcmp(deviceMode, ADAPTIVE) == 0)
      && !lightsOn) {
    sendDataSooner = true;
    lightsOn = true;
  }
  else if (strcmp(deviceMode, OFF) == 0 && lightsOn) {
    sendDataSooner = true;
    lightsOn = false;
  }
  else if (motionDetected) {
    if (!lightsOn & lux < minLux) {
      lightsOn = true;
      sendDataSooner = true;
    } else if (lightsOn & lux >= minLux * 2) {
      lightsOn = false;
      sendDataSooner = true;
    }
  } else if (lightsOn && millis() - motionTimestamp > (detectionIntervals[periodIndex] + lightTimeouts[periodIndex]) * SECOND) {
    sendDataSooner = true;
    lightsOn = false;
  }
  
  /*Моніторинг показників системи*/
  if (millis() - dataTimestamp > sensorsUpdateTimeout * SECOND || sendDataSooner) {
    dataTimestamp = millis();
    sendDataSooner = false;
    sendMqttMessage(MQTT_TOPIC_DATA);
    printSensorsData();
    printConfigurationData();
  }
  if (millis() - statusTimestamp > statusUpdateTimeout * SECOND) {
    sendMqttMessage(MQTT_TOPIC_DEVICE);
    sendMqttMessage(MQTT_TOPIC_PARAMS);
    statusTimestamp = millis();
  }
  
  yield();
}



/****************** ФУНКЦІЇ ******************/
/*Підключення до Wi-Fi та сервера MQTT*/
void connectWifiMqtt() {
  while (!mqttClient.connected()) { // if there is no connection to MQTT server
    /*Connecting to Wi-Fi*/
    if (WiFi.status() != WL_CONNECTED) {
      WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
      while (WiFi.status() != WL_CONNECTED) {
        delay(SECOND);
        Serial.print(".");
      }
      Serial.print("Connected to ");
      Serial.println(WIFI_SSID);
      Serial.print("IP address: ");
      Serial.println(WiFi.localIP());
    }
    /*Connecting to MQTT Server*/
    mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
    mqttClient.setCallback(callback);
    /*String client_id = "esp8266-client-";
    client_id += String(WiFi.macAddress());*/
    if (mqttClient.connect(MQTT_CLIENT_ID/*, mqttUser, mqttPassword*/)) {
      mqttClient.subscribe(MQTT_TOPIC_PERIOD);
      mqttClient.subscribe(MQTT_TOPIC_SCHEDULE);
      mqttClient.subscribe(MQTT_TOPIC_CONFIG);
      mqttClient.subscribe(MQTT_TOPIC_MODE);
      
      Serial.println("Connected to MQTT Server");
      if (mqttClient.setBufferSize(mqttClient.getBufferSize() * 2)) {
        Serial.print("New MQTT buffer size: ");
        Serial.println(mqttClient.getBufferSize());
        sendMqttMessage(MQTT_TOPIC_DEVICE);
      }
    } else {
      Serial.print("[FAILED] [ rc = ");
      Serial.print(mqttClient.state());
      Serial.println(" : retrying in 5 seconds]");
      delay(5 * SECOND);
    }
  }
}

/*Обробка вхідних повідомлень MQTT*/
void callback(char* topic, byte* payload, unsigned int length) {
  String message = "";
  for (int i = 0; i < length; i++) {
    message = message + (char)payload[i];
  }
  Serial.print("Message arrived in topic: ");
  Serial.println(topic);
  Serial.print("Message:");
  Serial.println(message);
  
  StaticJsonDocument<500> doc;
  deserializeJson(doc, message);

  if (strcmp(topic, MQTT_TOPIC_CONFIG) == 0) {
    strlcpy(room, doc["room"] | "default", 32);
    strlcpy(currentPeriod, doc["period"] | "default", 8);
    strlcpy(deviceMode, doc["mode"] | "default", 13);
    zoneId = doc["zone"];
    rowNumber = doc["row_number"];
    strlcpy(roomType, doc["room_type"] | "default", 32);
    sensorsUpdateTimeout = doc["sens_upd"];
    statusUpdateTimeout = doc["stat_upd"];
    customDimValue = doc["cust_dim"];
    minLux = doc["min_lux"];
    deltaLux = doc["delta_lux"];
    copyArray(doc["detect_int"], detectionIntervals);
    copyArray(doc["timeout"], lightTimeouts);
  }
  else if (strcmp(topic, MQTT_TOPIC_MODE) == 0) {
    strlcpy(deviceMode, doc["mode"] | "default", 13);
    strlcpy(modeUpdate, doc["date"] | "default", 32);
    customDimValue = doc["cust_dim"];
    modeTimeout = doc["mode_time"];
  }
  else if (strcmp(topic, MQTT_TOPIC_PERIOD) == 0) {
    strlcpy(currentPeriod, doc["period"] | "default", 8);
    for (int i = 0; i < 3; i++) {
      if (strcmp(currentPeriod, periods[i]) == 0) {
        periodIndex = i;
      }
    }
    strlcpy(modeUpdate, doc["date"] | "default", 32);
  }
  else if (strcmp(topic, MQTT_TOPIC_SCHEDULE) == 0) {
    Serial.println("\nNEW SCHEDULE RECEIVED");
    printCurrentTime();
    for (CronID_t id = 0; id < dtNBR_ALARMS; id++) {
      if (Cron.isAllocated(id)) {
        Cron.free(id);
      }
    }
    JsonArray scheduleArray;
    char* eventTime = new char[30];
    if (doc.containsKey("lesson")) {
      scheduleArray = doc["lesson"];
      for (int i = 0; i < scheduleArray.size(); i++) {
        const char* event = doc["lesson"][i];
        strcpy(eventTime, doc["lesson"][i]);
        Serial.print(Cron.create(eventTime, setPeriodLesson, false));
        Serial.print(" - New lesson event: ");
        Serial.println(event);
      }
    }
    if (doc.containsKey("break")) {
      scheduleArray = doc["break"];
      for (int i = 0; i < scheduleArray.size(); i++) {
        const char* event = doc["break"][i];
        strcpy(eventTime, doc["break"][i]);
        Serial.print(Cron.create(eventTime, setPeriodBreak, false));
        Serial.print(" - New break event: ");
        Serial.println(event);
      }
    }
    if (doc.containsKey("passive")) {
      scheduleArray = doc["passive"];
      for (int i = 0; i < scheduleArray.size(); i++) {
        const char* event = doc["passive"][i];
        strcpy(eventTime, doc["passive"][i]);
        Serial.print(Cron.create(eventTime, setPeriodPassive, false));
        Serial.print(" - New passive mode event: ");
        Serial.println(event);
      }
    }
    if (doc.containsKey("day_off")) {
      isDayOff = doc["day_off"].as<bool>();
      if (isDayOff) {
        disableAlarms();
        setPeriodPassive();
      } else {
        enableAlarms();
      }
    }
    strlcpy(modeUpdate, doc["date"] | "default", 32);
  }
  modeTimestamp = millis();
}

/*Надсилання повідомлення MQTT на брокер (дані контролера/сенсорів/світильників)*/
bool sendMqttMessage(const char* topic) {
  DynamicJsonDocument doc(500); //виділення пам'яті для документу JSON
  JsonObject JSONencoder = doc.to<JsonObject>(); //створення базового об'єкту
  JSONencoder["room"] = room;
  JSONencoder["period"] = currentPeriod;
  JSONencoder["mode"] = deviceMode;
  JSONencoder["mac"] = WiFi.macAddress();
  
  if (strcmp(topic, MQTT_TOPIC_DEVICE) == 0) {
    JSONencoder["ip"] = WiFi.localIP();
    JSONencoder["device"] = MQTT_CLIENT_ID;
    JSONencoder["fw_upd"] = firwareUpdate;
    JSONencoder["md_upd"] = modeUpdate;
    if (deviceMode == "init") {
      JsonObject pins = doc.createNestedObject("pins");
      pins["mv"] = DOPPLER_PIN;
      pins["lt_sda"] = LUX_SDA_PIN;
      pins["lt_scl"] = LUX_SCL_PIN;
      pins["dim"] = DIMMER_DIM_PIN;
    }
  }
  else if (strcmp(topic, MQTT_TOPIC_DATA) == 0) {
    JsonArray mv_sensors = doc.createNestedArray("mv_sens");
    JsonObject motion_sensor = mv_sensors.createNestedObject();
    motion_sensor["pin"] = DOPPLER_PIN;
    motion_sensor["zone_id"] = zoneId;
    motion_sensor["motion"] = motionDetected;
    
    JsonArray lt_sensors = doc.createNestedArray("lt_sens");
    JsonObject light_sensor = lt_sensors.createNestedObject();
    light_sensor["pin_sda"] = LUX_SDA_PIN;
    light_sensor["pin_scl"] = LUX_SCL_PIN;
    light_sensor["zone_id"] = zoneId;
    light_sensor["level"] = lux;

    JsonArray rows = doc.createNestedArray("rows");
    JsonObject row = rows.createNestedObject();
    row["pin"] = DIMMER_DIM_PIN;
    row["zone_id"] = zoneId;
    row["row_num"] = rowNumber;
    row["on"] = lightsOn;
    row["type"] = "desk";
    row["dim"] = dimValue;
  }
  else if (strcmp(topic, MQTT_TOPIC_PARAMS) == 0) {
    JSONencoder["zone"] = zoneId;
    JSONencoder["row_number"] = rowNumber;
    JSONencoder["room_type"] = roomType;
    JSONencoder["sens_upd"] = sensorsUpdateTimeout;
    JSONencoder["stat_upd"] = statusUpdateTimeout;
    JSONencoder["cust_dim"] = customDimValue;

    JSONencoder["min_lux"] = minLux;
    JSONencoder["delta_lux"] = deltaLux;

    JsonArray intervals = doc.createNestedArray("detect_int");
    for (int i = 0; i < sizeof(detectionIntervals) / sizeof(detectionIntervals[0]); i++) {
      intervals.add(detectionIntervals[i]);
    }
    JsonArray timeouts = doc.createNestedArray("timeout");
    for (int i = 0; i < sizeof(lightTimeouts) / sizeof(lightTimeouts[0]); i++) {
      timeouts.add(lightTimeouts[i]);
    }
  }
  
  char JsonPayload[400];
  serializeJson(doc, JsonPayload);

  Serial.print("\n\nSENDING MESSAGE to MQTT topic ");
  Serial.print(topic);
  
  //serializeJsonPretty(doc, Serial);
  if (mqttClient.publish(topic, JsonPayload) == true) {
    Serial.print(" - Success - ");
    Serial.print(doc.memoryUsage());
    Serial.println(" bytes");
  } else {
    Serial.println(" - Error");
  }
}

/*Розрахунок кроку зміни потужності диммера*/
int calculateStepValue(unsigned long currentLux, unsigned long goalLux) {
  long gap = abs((long) currentLux - (long) goalLux);
  int bounds[] = {1000, 500, 100, 50};
  int steps[] = {100, 20, 5, 3};
  for (int i = 0; i < sizeof(bounds) / sizeof(int); i++) {
    if (gap >= bounds[i]) {
      return steps[i];
    }
  }
  return 1;
}

/*Ініціалізація пінів та об'єктів*/
void initializeObjectsPins() {
  /*Піни*/
  pinMode(DOPPLER_PIN, INPUT);
  pinMode(LUX_SDA_PIN, INPUT);
  pinMode(LUX_SCL_PIN, INPUT);
  pinMode(DIMMER_INTERRUPT_PIN, INPUT_PULLUP);
  pinMode(DIMMER_DIM_PIN, OUTPUT);
  /*Датчики та послідовний порт*/
  Wire.begin();
  if (lightMeter.begin()) {
    Serial.println(F("BH1750 initialised"));
  }
  else {
    Serial.println(F("Error initialising BH1750"));
    ESP.restart();
  }
  Serial.begin(115200);
}

/*Перевірка обнулення показників millis() при переповненні змінної*/
void checkTimestamps() {
  if (millis() < statusTimestamp || millis() < motionTimestamp || millis() < dataTimestamp) {
    statusTimestamp = 0;
    motionTimestamp = 0;
    dataTimestamp = 0;
  }
}

void syncTime() {
  
}

void enableAlarms() {
  for (CronID_t id = 0; id < dtNBR_ALARMS; id++) {
    Cron.enable(id);
  }
}

void disableAlarms() {
  for (CronID_t id = 0; id < dtNBR_ALARMS; id++) {
    Cron.disable(id);
  }
}

void setPeriodLesson() {
  currentPeriod = LESSON;
  periodIndex = getPeriodIndex();
  Serial.print("Lesson beginning - ");
  printCurrentTime();
}

void setPeriodBreak() {
  currentPeriod = BREAK;
  periodIndex = getPeriodIndex();
  Serial.print("Break beginning - ");
  printCurrentTime();
}

void setPeriodPassive() {
  currentPeriod = PASSIVE;
  periodIndex = getPeriodIndex();
  Serial.print("Passive mode beginning - ");
  printCurrentTime();
}

int getPeriodIndex() {
  for (int i = 0; i < 3; i++) {
    if (strcmp(currentPeriod, periods[i]) == 0) {
      return i;
    }
  }
}

void printCurrentTime() {
  time_t tnow = time(nullptr);
  Serial.println(ctime(&tnow));
}

/*Завершення калібрування датчика*/
void finishPirCalibrating(int calibrationTime, unsigned long startTimestamp) {
  if (millis() - startTimestamp < calibrationTime * SECOND) {
    Serial.print("PIR sensor calibration in process, ");
    Serial.print((calibrationTime * SECOND - (millis() - startTimestamp)) / SECOND);
    Serial.print(" seconds left");
    while (millis() - startTimestamp < calibrationTime * SECOND) {
      delay(SECOND);
      Serial.print(".");
    }
    Serial.print("\nSensor is calibrated:)");
  }
}


void printSensorsData() {
  Serial.print("[SENSORS]: ");

  lux = lightMeter.readLightLevel();
  Serial.print("Light: ");
  Serial.print(lux);
  Serial.print(" lx; ");
  
  motionDetected = digitalRead(DOPPLER_PIN) == HIGH;
  Serial.print("Motion: ");
  Serial.print(motionDetected);
  digitalWrite(LED_BUILTIN, digitalRead(DOPPLER_PIN));

  Serial.print("; Dimmer value: ");
  Serial.print(dimValue);

  Serial.print("; Lights ON: ");
  Serial.println(lightsOn);
 
}


void printConfigurationData() {
  Serial.print("[CONFIG]: ");

  Serial.print("Room: ");
  Serial.print(room);
  Serial.print("; Zone ID: ");
  Serial.print(zoneId);
  Serial.print("; Row number: ");
  Serial.print(rowNumber);
  Serial.print("; Room type: ");
  Serial.println(roomType);
  
  Serial.print("Device mode: ");
  Serial.print(deviceMode);
  Serial.print("; Current period: ");
  Serial.print(currentPeriod);
  Serial.print("; Device period index: ");
  Serial.println(periodIndex);


  Serial.print("Min lux: ");
  Serial.print(minLux);
  Serial.print("; Delta lux: ");
  Serial.print(deltaLux);
  Serial.print("; Custom dim value: ");
  Serial.println(customDimValue);
  
  Serial.print("Sensors update timeout: ");
  Serial.print(sensorsUpdateTimeout);
  Serial.print("; Status update timeout: ");
  Serial.print(statusUpdateTimeout);
  Serial.print("; Detection intervals: ");
  for (int i = 0; i < 3; i++) {
    Serial.print(detectionIntervals[i]);
    Serial.print(" ");
  }
  Serial.print("; Light timeouts: ");
  for (int i = 0; i < 3; i++) {
    Serial.print(lightTimeouts[i]);
    Serial.print(" ");
  }
  Serial.println("");
}

/*https://github.com/Martin-Laclaustra/CronAlarms/blob/master/examples/CronAlarms_example/CronAlarms_example.ino*/
