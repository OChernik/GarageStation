// датчик движения PIR1 GPIO26
// датчик движения PIR2 GPIO27
// датчик движения PIR3 GPIO35
// датчик расстояния HC-SR04 ECHO GPIO34 TRIGGER GPIO35
// реле управления вентилятором GPIO16 включается при логическом нуле
// реле управления прожектором GPIO17 включается при логическом нуле
// реле управления воротами GPIO18 включается при логической единице
// вход геркона GPIO25
// Датчик SHT41, SHT31, BMP280  SDA - 21, SCL - 22
// Экран SSH1106 1,3''  SDA - 21, SCL - 22
// Esp32 с антенной
// вентилятор включается в автоматическом режиме если 
// humidityGarage - humidityCalc >= myData.deltaHumidity + myData.hysteresis 

// Настройки____________________________________________________________________
const char* ssid = "*********";        // wifi Login
const char* password = "*********";        // wifi Password
const char* mqttLogin = "*********";        // mqtt Login
const char* mqttPass = "*********";         // mqtt Password

const char* hubPrefix = "*********";  // GyverHub hubPrefix
const char* hubClientID = "*********";    // GyverHub Client ID
const char* OpenMonKey = "*********";     // Open Monitoring Key
const char* otaPass = "*********";        // OTA Password
#define BOT_TOKEN "****"                  // Telegram bot token
#define CHAT_ID "*****"                   // Telegram chat ID
#define OLEG_ID "*****"                   // Telegram chat ID

#define pir1 26                          // sensor PIR1 to GPIO26
#define pir2 27                          // sensor PIR2 to GPIO27
#define pir3 35                          // sensor PIR2 to GPIO35
#define echoPin 34                       // HC-SR04 ECHO to GPIO34
#define triggerPin 32                    // HC-SR04 TRIGGER to GPIO35
#define MaxDistance 300                  // HC-SR04 максимально возможное расстояние
#define relayVent 16                     // relay to GPIO16
#define relayLight 17                    // relay to GPIO17
#define relayGate 18                     // relay to GPIO18
#define gercon 25                        // геркон to GPIO25
#define idleTimePeriod 5 * 60 * 1000L    // время отсутствия движения, через которое закрываются ворота
#define sensorReadPeriod 2 * 1000        // период между опросом датчиков температуры, влажности, давления в мс.
#define gateReadPeriod 300               // период между опросом геркона, датчиков дистании и ПИР в мс.
#define openMonPeriod 5 * 60 * 1000L     // период между отправкой данных на сервер ОМ в мс.
#define narodMonPeriod 6 * 60 * 1000L    // период между отправкой данных на сервер NM в мс.
#define checkWifiPeriod 30 * 1000L       // период проверки состояния WiFi соединения в мс.
#define oledInvertPeriod 60 * 1000L      // период инверсии дисплея
#define heatPeriod 24 * 60 * 60 * 1000L  // период безусловного включения нагрева датчиков
#define heat3xTime 5 * 60 * 1000L        // время, на которое включается нагрев датчика SHT3x
#define heat4xPeriod 3 * 60 * 1000L      // период включения нагрева SHT4x
#define heat3xBorder 65                  // значение влажности, выше которого включается нагрев датчика SHT3x
#define heat4xBorder 75                  // значение влажности, выше которого включается нагрев датчика SHT4x
#define WDT_TIMEOUT 30                   // 30 секунд отсутствия отклика для перезагрузки через WDT
#define ATOMIC_FS_UPDATE                 // чтобы можно было отсылать боту  .bin.gz архив прошивки
#define latitude 52.6                    // широта места для расчета времени восхода/захода солнца
#define longitude 39.6                   // долгота места для расчета времени восхода/захода солнца
#define scanPeriod 30000                 // период между сканированиями Bluetooth
#define stopPeriod 10000                 // длительность сканирования Bluetooth

// Библиотеки_______________________________________________________________
#include <Arduino.h>
#include <esp_task_wdt.h>  // библиотека WatchDogTimer
#include <WiFi.h>
#include <HTTPClient.h>
#include <Wire.h>
#include <SensirionI2cSht3x.h>  // библиотека датчиков температуры и влажности SHT3х
#include <SensirionI2cSht4x.h>  // библиотека датчиков температуры и влажности SHT4х
#include <GyverBME280.h>        // библиотека датчика температуры и давления BMP280 или BME280
#include <ArduinoOTA.h>         // бибилотека ОТА обновления по WiFi
#include <ESPmDNS.h>            // нужно для работы бибилиотеки ArduinoOTA.h
#include <WiFiUdp.h>            // нужно для работы бибилиотеки ArduinoOTA.h
#include <GyverOLED.h>          // библиотека дисплея
#include <TimerMs.h>            // библиотека таймера
#include <GyverHub.h>           // GyverHub
#include <NewPing.h>            // подключаем библиотеку NewPing для работы датчика расстояния
#include <FileData.h>           // для сохранения переменных в памяти ESP32 вместо EEPROM
#include <LittleFS.h>           // для сохранения переменных в памяти ESP32 вместо EEPROM
#include <FastBot.h>            // библиотека управления телеграм-ботом
#include <GyverNTP.h>           // для получения точного времени с серверов и удобных действий со временем 
#include <sunset.h>             // для расчета времени восхода/захода солнца. Требуется чтобы вычислять время наступления темноты

struct Data {                  // структура для хранения настроек в памяти ESP32
  float humInCorrection = 2.6; // поправка влажности датчика внутри гаража
  float humOutCorrection = 0;  // поправка влажности датчика на улице
  uint8_t deltaHumidity = 12;   // порог автовключения вентилятора
  uint8_t hysteresis = 2;      // разница влажности между включением и выключением вентилятора. Чтобы реле 220V не щелкало слишком часто
                               // если время между открытием ворот и выездом машины меньше gateOpenedPeriod,
                               // ворота будут автоматически закрываться через carLeavePeriod после выезда из гаража
  uint32_t gateOpenedPeriod = 600000;
  uint32_t carLeavePeriod = 20000;  // через какое время после выезда машины из гаража закрываются ворота
  uint32_t lightPeriod = 120000;    // время, на которое включается прожектор
  uint8_t day = 20;                 // текущий день
  uint8_t month = 10;               // текущий месяц
  uint16_t year = 2024;             // текущий год
  uint16_t dayLightShift = 600;     // сдвиг времени сумерек относительно времени восхода/захода
};
Data myData;  // объявляем структуру myData с типом  Data
// создание объекта data библиотеки FileData для сохранения настроек на флеше ESP32
FileData data(&LittleFS, "/myData.dat", 136, &myData, sizeof(myData));

// Объекты библиотек____________________________________________________________________________________
SensirionI2cSht3x sensorIn;                       // создание объекта датчика sensorIn в гараже
SensirionI2cSht4x sensorOut;                      // создание объекта датчика sensorOut на улице
GyverBME280 bme;                                  // Создание обьекта bme
GyverOLED<SSH1106_128x64> oled;                   // создание объекта экрана SSH1106 1,3''
HTTPClient http;                                  // создаем объект http библиотеки HTTPClient
WiFiClient client;                                // создаем объект client библиотеки WiFiClient
TimerMs oledTmr(oledInvertPeriod, 1, 0);          // создаем объект oledTmr таймера MyTimer с периодом oledInvertPeriod
TimerMs heat4xTmr(heat4xPeriod, 1, 0);            // создаем объект heat4xTmr таймера MyTimer с периодом heat4xPeriod
TimerMs checkWifiTmr(checkWifiPeriod, 1, 0);      // создаем объект checkWifiTmr таймера MyTimer с периодом checkWifiPeriod
TimerMs sensorReadTmr(sensorReadPeriod, 1, 0);    // создаем объект sensorReadTmr таймера MyTimer с периодом sensorReadPeriod
TimerMs gateReadTmr(gateReadPeriod, 1, 0);        // создаем объект gateReadTmr таймера MyTimer с периодом gateReadPeriod
GyverHub hub;                                     // создаем объект GyverHub
FastBot bot(BOT_TOKEN);                           // создаем объект FastBot
NewPing sonar(triggerPin, echoPin, MaxDistance);  // создаем объект NewPing
SunSet lipetsk;                                   // создаем объект SunSet

// Переменные___________________________________________________________________________________
float temperatureGarage;     // значение температуры в гараже
float temperatureOut;        // значение температуры на улице
float temperatureBox;        // значение температуры в коробке
float humidityGarage;        // значение влажности в гараже
float humidityOut;           // значение влажности на улице
float humidityCalc;          // расчетное значение уличной влажности, приведенное к температуре гаража
float pressure;              // значение давления
float tempTemperature;       // первичное значение температуры с датчика до проверки на выброс
float tempHumidity;          // первичное значение влажности с датчика до проверки на выброс
int distance;                // значение расстояния до машины
int8_t rssi;                 // переменная измеренного значения rssi, dB
uint16_t mornDawn;           // время начала утреннего рассвета, секунд
uint16_t nightFall;          // время начала сумерек, секунд
uint32_t heat3xTmr = 0;      // переменная таймера нагрева датчика SHT31
uint32_t openMonTmr = 0;     // переменная таймера отправки сообщений на сервер open-monitoring.online
uint32_t narodMonTmr = 0;    // переменная таймера отсылки данных на сервер NarodMon
uint32_t heat3xStart = 0;    // переменная времени начала нагрева датчика SHT31
uint32_t heat4xStart = 0;    // переменная времени начала нагрева датчика SHT41
uint32_t idleTime = 0;       // текущее время покоя
uint32_t idleTimeTmr = 0;    // переменная таймера состояния покоя
uint32_t gateOpenedTmr = 0;  // переменная таймера открытия ворот
uint32_t carLeaveTmr = 0;    // переменная таймера выезда машины
uint32_t lightTmr = 0;       // переменная таймера включения прожектора
bool pir1State = 0;          // состояние датчика pir1. 0 - покой, 1 - движение
bool pir2State = 0;          // состояние датчика pir2. 0 - покой, 1 - движение
bool pir3State = 0;          // состояние датчика pir3. 0 - покой, 1 - движение
bool heat3xFlag = 0;         // флаг нагрева датчика SHT31
bool heat4xFlag = 0;         // флаг нагрева датчика SHT41
bool oledFlag = 0;           // флаг состояния инверсии дисплея
bool gateState;              // состояние ворот. 0 - закрыты (геркон разомкнут), 1 - открыты
bool gateOpened = 0;         // флаг факта открытия ворот. Для обновления ПУ
bool gateClosed = 0;         // флаг факта закрытия ворот. Для обновления ПУ
bool ventState = 0;          // состояние вентилятора. 0 - выключен, 1 - включен
bool autoLightState = 0;     // состояние прожектора. 0 - выключен, 1 - включен
bool manualLightState = 0;   // состояние прожектора. 0 - выключен, 1 - включен
bool lightButtonPressed = 0; // нажатие кнопки включения/выключения света в боте 
bool ventAuto = 1;           // управление вентилятором. 0 - ручное, 1 - автоматическое
bool carStatus = 0;          // 1 - машина в гараже, 0 - машина отсутствует
bool hubChanged = 0;         // 1 - требуется изменить конфигурацию виджетов ПУ
bool idleState = 0;          // состояние общего покоя. 0 - покой, 1 - движение
bool isDark;                 // темно ли на улице. 1 - темно, 0 - светло
int idleSec = 0;             // текущее время покоя в целых секундах

void build(gh::Builder& b) {  // билдер GyverHub.
  // добавляем в меню два пользовательских пункта 0.Контроль гаража и 1.Настройки гаража.
  // пункт 1.Настройки гаража видит только пользователь с указанным Client ID
  if (b.build.client.id == hubClientID) {
    b.Menu("Контроль гаража;Настройки гаража");
  } else {
    b.Menu("Контроль гаража");
  }

  b.show(b.menu() == 0);                                                  // если выбран пункт меню 0: Контроль гаража
  b.Title(F("Контроль гаража")).fontSize(32).color(gh::Colors::Default);  // добавим заголовок
  // горизонтальный контейнер с температурой и влажностью на улице
  if (b.beginRow()) {
    b.Label_(F("TempOut"), temperatureOut).label(F("Температура на улице")).color(gh::Colors::Red);
    b.Label_(F("HumOut"), humidityOut).label(F("Влажность на улице")).color(gh::Colors::Aqua);
    b.endRow();
  }
  // горизонтальный контейнер с температурой и влажностью в гараже
  if (b.beginRow()) {
    b.Label_(F("TempGarage"), temperatureGarage).label(F("Температура в гараже")).color(gh::Colors::Red);
    b.Label_(F("HumGarage"), humidityGarage).label(F("Влажность в гараже")).color(gh::Colors::Aqua);
    b.endRow();
  }
  // горизонтальный контейнер с приведенной влажностью и давлением
  if (b.beginRow()) {
    b.Label_(F("Pressure"), pressure).label(F("Атмосферное давление")).color(gh::Colors::Orange);
    b.Label_(F("HumCalc"), humidityCalc).label(F("Приведенная влажность")).color(gh::Colors::Aqua);
    b.endRow();
  }
  // горизонтальный контейнер с работой вентилятора и прожектора
  if (b.beginRow()) {
    if (b.Switch(&ventAuto).label(F("АвтоРежим")).click()) (hubChanged = 1);
    if (ventAuto) {  // если вентилятор в режиме автоматического управления
      b.LED_(F("ventLed"), &ventState).label(F("Вентилятор ON/OFF"));
    } else {
      if (b.Switch(&ventState).label(F("Вентилятор ON/OFF")).click()) (hubChanged = 1);
    }
    if (b.Switch(&manualLightState).label(F("Прожектор ON/OFF")).click()){
      hubChanged = 1;
      if (manualLightState) lightTmr = millis();
    } 
    b.endRow();
  }
  // статус ворот
  if (gateState) {  // если ворота открыты
    if (b.beginRow()) {
      if (b.Switch_(F("gateSwitch"), &gateState).size(2).label(F("Закрыть ворота?")).click()) {
        switchGate();  // закрываем ворота
        gateClosed = 1;
      }
      b.LED_(F("PIR1"), &pir1State).label(F("PIR1")).size(1);
      b.LED_(F("PIR2"), &pir2State).label(F("PIR2")).size(1);
      b.LED_(F("PIR3"), &pir3State).label(F("PIR3")).size(1);
      b.Label_(F("IDLE"), idleSec).label(F("Время покоя")).size(2).color(gh::Colors::Aqua);
      b.endRow();
    }
  } else {
    b.Title(F("Ворота закрыты")).fontSize(32).color(gh::Colors::Green);
  }
  if (b.beginRow()) {
    if (carStatus) {
      b.Title_(F("Car"), F("Машина в гараже")).fontSize(32).color(gh::Colors::Green);
    } else {
      b.Title_(F("Car"), F("Машина не в гараже")).fontSize(32).color(gh::Colors::Orange);
    }
    b.endRow();
  }

  // горизонтальный контейнер с полями для тестовых переменных
  if (b.beginRow()) {
    // bool temp = digitalRead(relayVent);
    b.Label_(F("Test1"), lipetsk.calcSunrise()).label(F("Восход солнца")).color(gh::Colors::Red).fontSize(16);
    b.Label_(F("Test2"), lipetsk.calcSunset()).label(F("Заход солнца")).color(gh::Colors::Aqua).fontSize(16);
    b.Label_(F("Test3"), isDark).label(F("isDark")).color(gh::Colors::Aqua).fontSize(16);
    b.endRow();
  }

  b.show(b.menu() == 1);  // если выбран пункт меню 1: Настройки гаража
  // добавляем спиннеры с поправкой влажности
  if (b.beginRow()) {
    if (b.Spinner(&myData.humOutCorrection).range(-10, 10, 0.1).label(F("Поправка влажности на улице")).click()) data.update();
    if (b.Spinner(&myData.humInCorrection).range(-10, 10, 0.1).label(F("Поправка влажности в гараже")).click()) data.update();
    b.endRow();
  }
  // добавляем спиннеры с дельтой включения вентилятора и гистерезисом
  if (b.beginRow()) {
    if (b.Spinner(&myData.deltaHumidity).range(0, 15, 1).label(F("Дельта влажности")).click()) data.update();
    if (b.Spinner(&myData.hysteresis).range(0, 5, 1).label(F("Гистерезис")).click()) data.update();
    b.endRow();
  }
  // добавляем спиннеры с gateOpenedPeriod и carLeavePeriod
  if (b.beginRow()) {
    if (b.Spinner(&myData.gateOpenedPeriod).range(0, 1200000, 60000).label(F("gateOpenedPeriod")).click()) data.update();
    if (b.Spinner(&myData.carLeavePeriod).range(0, 60000, 5000).label(F("carLeavePeriod")).click()) data.update();
    b.endRow();
  }

  // добавляем спиннеры с lightPeriod и dayLightShift
  if (b.beginRow()) {
    if (b.Spinner(&myData.lightPeriod).range(0, 1200000, 60000).label(F("Время включения прожектора")).click()) data.update();
    if (b.Spinner(&myData.dayLightShift).range(0, 1800, 60).label(F("Время до темноты, с")).click()) data.update();    
    b.endRow();
  }
}  // end void build()

// Setup______________________________________________________________________________________________
void setup() {
  pinMode(pir1, INPUT_PULLDOWN);    // задаем вход датчика pir1 и подтягиваем его к земле
  pinMode(pir2, INPUT_PULLDOWN);    // задаем вход датчика pir2 и подтягиваем его к земле
  pinMode(pir3, INPUT_PULLDOWN);    // задаем вход датчика pir3 и подтягиваем его к земле
  pinMode(relayVent, OUTPUT);       // пин реле вентилятора - выход
  pinMode(relayLight, OUTPUT);      // пин реле прожектора - выход
  pinMode(relayGate, OUTPUT);       // пин реле ворот - выход
  pinMode(gercon, INPUT_PULLDOWN);  // подтягиваем вход геркона к земле
  pinMode(triggerPin, OUTPUT);      // назначаем trigPin, как выход
  pinMode(echoPin, INPUT);          // назначаем echoPin, как вход

  (digitalRead(gercon)) ? (gateState = 1) : (gateState = 0);  // задаем исходное состояние ворот

  distance = sonar.ping_cm();                            // определили расстояние до машины
  (distance > 100) ? (carStatus = 0) : (carStatus = 1);  // определили исходный статус машины - в гараже или нет

  digitalWrite(relayGate, LOW);   // исходное низкое значение, реле нормальное
  digitalWrite(relayVent, HIGH);  // исходное высокое значение, реле обращенное
  digitalWrite(relayLight, HIGH); // исходное низкое значение, реле обращенное

  esp_task_wdt_init(WDT_TIMEOUT, true);  //enable panic so ESP32 restarts
  esp_task_wdt_add(NULL);                //add current thread to WDT watch

  LittleFS.begin();                  // инициализация файловой системы на флеше для записи настроек
  FDstat_t stat = data.read();       // считываем данные настроек из флеша. При первом запуске во флеш пишутся данные из структуры

  Serial.begin(115200);
  Wire.begin();                              // SensirionI2cSht3x.h and SensirionI2cSht4x.h
  sensorIn.begin(Wire, SHT31_I2C_ADDR_45);   // SensirionI2cSht3x.h
  sensorIn.disableHeater();                  // изначально выключаем нагрев датчика в гараже
  sensorOut.begin(Wire, SHT41_I2C_ADDR_44);  // SensirionI2cSht4x.h
  bme.begin();                               // инициализируем датчик BMP280

  oled.init();                   // инициализация дисплея
  oled.setContrast(10);          // яркость 0..255
  oled.textMode(BUF_REPLACE);    // вывод текста на экран с заменой символов
  oled.invertDisplay(oledFlag);  // вывод текста на экран с заменой символов
  oled.flipH(true);              // true/false - отзеркалить по горизонтали (для переворота экрана)
  oled.flipV(true);              // true/false - отзеркалить по вертикали (для переворота экрана)

  initWiFi();  // установили соединение WiFi

  NTP.begin(3);                                // запустить объект NTP и указать часовой пояс
  lipetsk.setPosition(latitude, longitude, 3); // задали расположение объекта lipetsk
  lipetsk.setCurrentDate(myData.year, myData.month, myData.day);        // задали начальную дату

  // библиотека ArduinoOTA.h делает что-то нужное для работы ОТА
  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else  // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

  ArduinoOTA.setHostname("ESP32_Garage");
  ArduinoOTA.setPassword(otaPass);
  ArduinoOTA.begin();

  hub.mqtt.config("m6.wqtt.ru", 17108, mqttLogin, mqttPass);  // подключаем платный защищенный MQTT сервис
  // hub.mqtt.config(F("test.mosquitto.org"), 1883);          // подключаем бесплатный незащищенный MQTT сервис
  hub.config(hubPrefix, F("Garage"), F("f494"));  // конфигурация GyverHub
  hub.onBuild(build);
  hub.begin();

  bot.setChatID(CHAT_ID);      // задаем  CHAT_ID бота
  bot.setPeriod(5000);         // период опроса в мс (по умолч. 3500)
  bot.attach(newMsg);          // подключаем функцию-обработчик сообщений
  // показываем меню бота с сообщением
  bot.showMenu("Обзор \t Улица \t Гараж \t Корпус \n Вент. ON/OFF \t Свет ON/Off \n Авто режим ON/OFF \t Ворота ON/OFF"); 

  // устанавливаем начальные измеряемые значения для корректной работы фильтра checkValue
  sensorIn.measureSingleShot(REPEATABILITY_HIGH, false, temperatureGarage, humidityGarage);  // SensirionI2cSht3x.h
  // sensorOut.measureSingleShot(REPEATABILITY_HIGH, false, temperatureOut, humidityOut);       // SensirionI2cSht3x.h
  sensorOut.measureHighPrecision(temperatureOut, humidityOut);  // SensirionI2cSht4x.h
  temperatureBox = bme.readTemperature();
  pressure = pressureToMmHg(bme.readPressure());

}  // end void Setup()

// Loop_______________________________________________________________________________________________
void loop() {

  esp_task_wdt_reset();  // сбрасываем Watch Dog Timer чтобы не прошла перезагрузка
  ArduinoOTA.handle();   // поддерживаем работу ОТА

  data.tick();  // сохранение настроек во Флеш памяти по таймауту

  bot.tick();  // тикаем для работы телеграм бота

  NTP.tick();  
  // делаем в течение дня несколько расчетов темного времени суток. На случай если не будет коннекта
  if (NTP.online() && (NTP.daySeconds() == 1000 || NTP.daySeconds() == 9000 || NTP.daySeconds() == 18000 || NTP.daySeconds() == 50000)) {
    myData.year = NTP.year();
    myData.month = NTP.month();
    myData.day = NTP.day();
    data.update();                                                   // записать измененные данные на флеш
    lipetsk.setCurrentDate(myData.year, myData.month, myData.day);
    mornDawn = lipetsk.calcSunrise() * 60 - myData.dayLightShift;    // время наступления рассвета, секунд
    nightFall = lipetsk.calcSunset() * 60 + myData.dayLightShift;    // время наступления сумерек, секунд
  } 
  (NTP.daySeconds() < mornDawn || NTP.daySeconds() > nightFall) ? (isDark = 1) : (isDark = 0);

  if (sensorReadTmr.tick()) {  // если пришло время опроса датчиков погоды
    sensorsRead();             // функция считывает все датчики погоды
    showScreen();              // вывод показаний датчиков на экран
  }                            // end if

  if (gateReadTmr.tick()) {  // если пришло время опроса датчиков
    gateRead();              // функция считывает геркон, датчик дистанции и ПИР датчики и выставляет флаги
    showScreen();            // вывод показаний датчиков на экран
  }                          // end if

  hub.tick();                      // тикаем для нормальной работы конструктора интерфейса
  static gh::Timer tmr(2000);      // период 2 секунды
  if (tmr) {                       // если прошел период
    hub.sendUpdate("TempOut");     // обновляем значение температуры на улице
    hub.sendUpdate("HumOut");      // обновляем значение влажности на улице
    hub.sendUpdate("TempGarage");  // обновляем значение температуры в гараже
    hub.sendUpdate("HumGarage");   // обновляем значение влажности в гараже
    hub.sendUpdate("HumCalc");     // обновляем значение приведенной влажности
    hub.sendUpdate("Pressure");    // обновляем значение давления
    hub.sendUpdate("ventLed");     // обновляем статус вентилятора    
    hub.sendUpdate("Car");         // обновляем статус машины
    hub.sendUpdate("Test1");       // обновляем статус тестовой переменной 1
    hub.sendUpdate("Test2");       // обновляем статус тестовой переменной 2
    hub.sendUpdate("Test3");       // обновляем статус тестовой переменной 3

    if (gateState) {  // если ворота открыты
      idleSec = round(idleTime / 1000);
      hub.sendUpdate("PIR1");  // обновляем состояние PIR1
      hub.sendUpdate("PIR2");  // обновляем состояние PIR2
      hub.sendUpdate("PIR3");  // обновляем состояние PIR3
      hub.sendUpdate("IDLE");  // обновляем состояние IdleSec
    }
  }

  // определили момент открытия ворот с машиной в гараже
  if (gateOpened && carStatus) gateOpenedTmr = millis();

  // включаем прожектор по кнопке бота или переключателю ПУ   
  if ((lightButtonPressed && !manualLightState) || (!lightButtonPressed && manualLightState)) {
    digitalWrite(relayLight, LOW);  // включаем прожектор
    manualLightState = 1;           // поднимаем флаг ручного включения
    lightButtonPressed = 0;         // сбрасываем факт нажатия кнопки    
  } 

  // выключаем включенный вручную прожектор по кнопке бота, переключателю ПУ или по таймеру   
  if ((lightButtonPressed && manualLightState) || (!lightButtonPressed && !manualLightState && !autoLightState) \ 
    || (manualLightState && (millis() - lightTmr) > myData.lightPeriod)) {
    digitalWrite(relayLight, HIGH); // выключаем прожектор
    manualLightState = 0;
    lightButtonPressed = 0;         // сбрасываем факт нажатия кнопки    
  }  // end if 

  // включаем прожектор автоматически при открытии ворот в темное время
  if (gateOpened && isDark) {
    digitalWrite(relayLight, LOW);  // включаем прожектор
    autoLightState = 1;
  }

  // если прожектор был включен автоматически, при закрытии ворот выключаем прожектор
  if (autoLightState && !gateState) {
    digitalWrite(relayLight, HIGH);  // выключаем прожектор
    autoLightState = 0;
  } 

  // если требуется изменить ПУ
  if (hubChanged || gateOpened || gateClosed) {
    hub.sendRefresh();
    gateOpened = 0;
    gateClosed = 0;
    hubChanged = 0;
  }

  // с периодом heatPeriod включаем прогрев датчика SHT31 на время heat3xTime
  // нагрев включается безусловно
  // начальные значения heat3xFlag = 0, heat3xTmr = 0
  if ((millis() - heat3xTmr) >= (heat3xFlag ? heat3xTime : heatPeriod)) {
    heat3xTmr = millis();       // сброс таймера
    heat3xFlag = !heat3xFlag;   // переключаем флаг состояния нагрева датчика
    if (heat3xFlag) {           // если поднят флаг нагрева
      heat3xStart = millis();   // сохраняем время начала нагрева
      sensorIn.enableHeater();  // включаем нагрев датчика в гараже
    } else {
      sensorIn.disableHeater();  // выключаем нагрев датчика в гараже
    }
  }  // end If

  // подогреваем датчик SHT41 если Humidity > heat4xBorder
  // с периодом heat4xPeriod включаем прогрев датчика SHT41 на 1 секунду
  // или если пришло время ежесуточного прогрева датчика SHT41
  if (((humidityOut > heat4xBorder) && heat4xTmr.tick()) || ((millis() - heat4xStart) > heatPeriod)) {
    heat4xStart = millis();                                                  // сохраняем время начала нагрева датчика
    sensorOut.activateHighestHeaterPowerLong(temperatureOut, tempHumidity);  // SensirionI2cSht4x.h
    humidityOut = tempHumidity + myData.humOutCorrection;                    // SensirionI2cSht4x.h
    showScreen();                                                            // вывод показаний датчиков на экран
    delay(1000);                                                             // чтобы заметить макс. темп. и RH на дисплее. Не придумал, как обойтись без delay
  }                                                                          // end If

  if (oledTmr.tick()) {            // если пришло время инвертировать дисплей
    oledFlag = !oledFlag;          // инвертируем флаг состояния дисплея
    oled.invertDisplay(oledFlag);  // инвертируем дисплей
  }

  // включаем/выключаем вентилятор в зависимости от состояния ventState в ручном режиме
  if (!ventAuto && ventState) digitalWrite(relayVent, LOW);  // включаем вентилятор
  else digitalWrite(relayVent, HIGH);                        // выключаем вентилятор

  // если машина в гараже в автоматическом режиме включаем вентилятор в зависимости от разницы влажности
  // при условии большого промежутка времени от момента нагрева датчика
  if (ventAuto && carStatus && ((millis() - heat4xStart) >= (heat4xPeriod - 15000))
      && ((humidityGarage - humidityCalc) >= (myData.deltaHumidity + myData.hysteresis))) {
    digitalWrite(relayVent, LOW);   // включаем вентилятор
    ventState = 1;                  // устанавливаем флаг вентилятора
    hub.sendUpdate("ventLed");      // обновляем состояние вентилятора на ПУ
  }

  // в автоматическом режиме выключаем вентилятор в зависимости от разницы влажности
  // при условии большого промежутка времени от момента нагрева датчика
  if (ventAuto && ((millis() - heat4xStart) >= (heat4xPeriod - 15000)) \ 
    && ((humidityGarage - humidityCalc) < (myData.deltaHumidity - myData.hysteresis))) {
    digitalWrite(relayVent, HIGH); // выключаем вентилятор
    ventState = 0;                 // устанавливаем флаг вентилятора
    hub.sendUpdate("ventLed");     // обновляем состояние вентилятора на ПУ
  }

  // восстанавливаем соединение при случайной пропаже
  if (checkWifiTmr.tick() && (WiFi.status() != WL_CONNECTED)) {
    Serial.println("Reconnecting to WiFi...");
    WiFi.disconnect();
    initWiFi();  // установили соединение WiFi
  }

  // если ворота открыты и имеем состояние покоя,  считаем  прошедшее время idleTime
  if (gateState && idleState) {
    idleTime = millis() - idleTimeTmr;
  } else {
    idleTimeTmr = millis();  // сброс таймера на текущее время
    idleTime = 0;            // сброс времени покоя
  }                          // end if

  // если время покоя достигает заданного значения, закрываем ворота
  if (gateState && (idleTime > idleTimePeriod)) switchGate();  // закрываем ворота

  // закрываем ворота через carLeavePeriod после выезда машины. carLeaveTmr отмечает время выезда машины
  // закрытие ворот при условии, что выезд произошел раньше, чем gateOpenedPeriod после открытия ворот
  // carLeaveTmr = 0 после закрытия ворот или если машина в гараже
  if (carLeaveTmr && gateState && (millis() - carLeaveTmr > myData.carLeavePeriod)
      && (millis() - gateOpenedTmr < myData.gateOpenedPeriod)) {
    switchGate();  // закрываем ворота
  }

  // Если пришло время очередной отправки на Open Monitoring
  // и прошло заданное время с момента последнего нагрева датчиков
  if ((millis() - openMonTmr) >= openMonPeriod && ((millis() - heat4xStart) >= (heat4xPeriod - 3000)) \ 
    && ((millis() - heat3xStart) >= (heat3xTime + 30000))) {
    openMonTmr = millis();  // сбрасываем таймер отправки данных
    sendToOpenMon();        // отправляем данные
  }                         // end if (sendtoOM)

  // Если пришло время очередной отправки  на NarodMon и прошло заданное время с момента последнего нагрева датчиков
  if (((millis() - narodMonTmr) >= narodMonPeriod) && ((millis() - heat4xStart) >= (heat4xPeriod - 3000)) \ 
    && ((millis() - heat3xStart) >= (heat3xTime + 30000))) {
    narodMonTmr = millis();  // сбрасываем таймер отправки данных
    sendToNarodMon();        // отправляем данные
  }                          // end if (sendtoNM)

}  // end Loop
