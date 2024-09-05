// датчик движения PIR1 GPIO26
// датчик движения PIR2 GPIO27
// датчик расстояния HC-SR04 ECHO GPIO34 TRIGGER GPIO35
// реле управления вентилятором GPIO16
// реле управления воротами GPIO18
// вход геркона GPIO25
// Датчик SHT41, SHT31, BMP280  SDA - 21, SCL - 22
// Экран SSH1106 1,3''  SDA - 21, SCL - 22
// Esp32 с антенной

// Настройки____________________________________________________________________

const char* ssid = "*********";           // wifi Login
const char* password = "********";        // wifi Password
const char* hubPrefix = "*********";      // GyverHub hubPrefix
const char* hubClientID = "********";     // GyverHub Client ID
const char* OpenMonKey = "********";      // Open Monitoring Key

#define pir1 26                           // sensor PIR1 to GPIO26
#define pir2 27                           // sensor PIR2 to GPIO27
#define echoPin 34                        // HC-SR04 ECHO to GPIO34
#define triggerPin 32                     // HC-SR04 TRIGGER to GPIO35
#define MaxDistance 300                   // HC-SR04 максимально возможное расстояние
#define relayVent 16                      // relay to GPIO16
#define relayGate 18                      // relay to GPIO18
#define gercon 25                         // геркон to GPIO25
#define idleTimePeriod 5 * 60 * 1000L     // время отсутствия движения, через которое закрываются ворота
// если время между открытием ворот и выездом машины меньше gateOpenedPeriod, 
// ворота будут автоматически закрываться через carLeavePeriod после выезда из гаража 
#define gateOpenedPeriod 5 * 60 * 1000L     
#define carLeavePeriod 20 * 1000L         // через какое время после выезда машины из гаража закрываются ворота
#define sensorReadPeriod 2 * 1000L        // период между опросом датчиков в мс.
#define openMonPeriod 5 * 60 * 1000L      // период между отправкой данных на сервер ОМ в мс.
#define narodMonPeriod 6 * 60 * 1000L     // период между отправкой данных на сервер NM в мс.
#define checkWifiPeriod 30 * 1000L        // период проверки состояния WiFi соединения в мс.
#define oledInvertPeriod 60 * 1000L       // период инверсии дисплея
#define heat3xPeriod 2 * 60 * 60 * 1000L  // период включения нагрева датчика SHT3x (время МЕЖДУ включениями)
#define heat3xTime 5 * 60 * 1000L         // время, на которое включается нагрев датчика SHT3x
#define heat4xPeriod 120 * 1000L          // период включения нагрева SHT4x
#define heat3xBorder 65                   // значение влажности, выше которого включается нагрев датчика SHT3x
#define heat4xBorder 75                   // значение влажности, выше которого включается нагрев датчика SHT4x
#define INIT_KEY 78                       // ключ первого запуска EEPROM. 0-254, на выбор
#define INIT_ADDR 0                       // номер ячейки для хранения ключа
#define WDT_TIMEOUT 30                    // 30 секунд отсутствия отклика для перезагрузки через WDT

// Библиотеки_______________________________________________________________
#include <esp_task_wdt.h>  // библиотека WatchDogTimer
#include <WiFi.h>
#include <HTTPClient.h>
#include <Wire.h>
#include <SensirionI2cSht3x.h>  // библиотека датчиков температуры и влажности SHT3х
#include <SensirionI2cSht4x.h>  // библиотека датчиков температуры и влажности SHT4х
#include <GyverBME280.h>        // библиотека датчика температуры и давления BMP280 или BME280
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>  //бибилотека ОТА обновления по WiFi
#include <GyverOLED.h>   //библиотека дисплея
#include <EncButton.h>   // библиотека енкодера v3.x
#include <Arduino.h>
#include <MyTimer.h>   // тестовая библиотека простейшего таймера моего изготовления
#include <GyverHub.h>  // GyverHub
#include <EEPROM.h>    // стандартная библиотека управления энергонезависимой памятью
#include <FileData.h>  // для сохранения переменных в памяти ESP32 вместо EEPROM
#include <LittleFS.h>  // для сохранения переменных в памяти ESP32
#include <NewPing.h>   // подключаем библиотеку NewPing для работы датчика расстояния

struct Data {                  // структура для хранения настроек в памяти ESP32
  int8_t hum3xCorrection = 0;  // поправка влажности датчика внутри гаража 
  int8_t hum4xCorrection = 0;  // поправка влажности датчика на улице 
  uint8_t deltaHumidity = 10;  // порог автовключения вентилятора 
  uint8_t hysteresis = 1;      // разница влажности между включением и выключением вентилятора. Чтобы реле 220V не щелкало слишком часто
};
Data myData;               // объект myData структуры с данными, сохраняемыми в памяти ESP32  
FileData data(&LittleFS, "/myData.dat", 'A', &myData, sizeof(myData));  // создание файла с данными на флеше ESP32

// Объекты библиотек____________________________________________________________________________________
//  EncButton enc(encodA, encodB, encodKEY);  //создание объекта енкодера enc и инициализация пинов с кнопкой EncButton v3.x
SensirionI2cSht3x sht3x;                          // создание объекта датчика sht3x библиотеки SensirionI2cSht3x
SensirionI2cSht4x sht4x;                          // создание объекта датчика sht4x библиотеки SensirionI2cSht4x
GyverBME280 bme;                                  // Создание обьекта bme
GyverOLED<SSH1106_128x64> oled;                   // создание объекта экрана SSH1106 1,3''
HTTPClient http;                                  // создаем объект http библиотеки HTTPClient
WiFiClient client;                                // создаем объект client библиотеки WiFiClient
MyTimer oledTmr(oledInvertPeriod);                // создаем объект oledTmr таймера MyTimer с периодом oledInvertPeriod
MyTimer heat4xTmr(heat4xPeriod);                  // создаем объект heat4xTmr таймера MyTimer с периодом heat4xPeriod
MyTimer checkWifiTmr(checkWifiPeriod);            // создаем объект checkWifiTmr таймера MyTimer с периодом checkWifiPeriod
MyTimer sensorReadTmr(sensorReadPeriod);          // создаем объект sensorReadTmr таймера MyTimer с периодом sensorReadPeriod
GyverHub hub;                                     // создаем объект GyverHub
NewPing sonar(triggerPin, echoPin, MaxDistance);  // создаем объект NewPing

// Переменные___________________________________________________________________________________
float temperatureGarage;          // значение температуры в гараже
float temperatureOut;             // значение температуры на улице
float temperatureBox;             // значение температуры в коробке
float humidityGarage;             // значение влажности в гараже
float humidityOut;                // значение влажности на улице
float humidityCalc;               // расчетное значение уличной влажности, приведенное к температуре гаража
float pressure;                   // значение давления
float tempTemperature;            // первичное значение температуры с датчика до проверки на выброс
float tempHumidity;               // первичное значение влажности с датчика до проверки на выброс
int distance;                     // значение расстояния до машины
int8_t rssi;                      // переменная измеренного значения rssi, dB
// int8_t hum3xCorrection = 0;       // поправка измеренного значения влажности SHT31
// int8_t hum4xCorrection = 0;       // поправка измеренного значения влажности SHT41
uint32_t heat3xTmr = 0;           // переменная таймера нагрева датчика SHT31
uint32_t openMonTmr = 0;          // переменная таймера отправки сообщений на сервер open-monitoring.online
uint32_t narodMonTmr = 0;         // переменная таймера отсылки данных на сервер NarodMon
uint32_t heat3xStart = 0;         // переменная времени начала нагрева датчика SHT31
uint32_t heat4xStart = 0;         // переменная времени начала нагрева датчика SHT41
uint32_t gateOpenedTmr = 0;       // переменная таймера открытия ворот
uint32_t carLeaveTmr = 0;         // переменная таймера выезда машины
bool pir1State = 0;               // состояние датчика pir1. 0 - покой, 1 - движение
bool pir2State = 0;               // состояние датчика pir2. 0 - покой, 1 - движение
bool pir3State = 0;               // состояние датчика pir3. 0 - покой, 1 - движение
bool heat3xFlag = 0;              // флаг нагрева датчика SHT31
bool heat4xFlag = 0;              // флаг нагрева датчика SHT41
bool oledFlag = 0;                // флаг состояния инверсии дисплея
bool gateState;                   // состояние ворот. 0 - закрыты (геркон разомкнут), 1 - открыты
bool gateOpened = 0;              // флаг факта открытия ворот. Для обновления ПУ
bool gateClosed = 0;              // флаг факта закрытия ворот. Для обновления ПУ
bool ventState = 0;               // состояние вентилятора. 0 - выключен, 1 - включен
bool ventAuto = 0;                // управление вентилятором. 0 - ручное, 1 - автоматическое
bool carStatus = 0;               // 1 - машина в гараже, 0 - машина отсутствует
bool hubChanged = 0;              // 1 - требуется изменить конфигурацию виджетов ПУ
bool idleState = 0;               // состояние общего покоя. 0 - покой, 1 - движение
uint32_t idleTimeTmr = 0;         // переменная таймера состояния покоя
uint32_t idleTime = 0;            // текущее время покоя
int idleSec = 0;                  // текущее время покоя в целых секундах

void build(gh::Builder& b) {      // билдер GyverHub.
  // добавляем в меню два пользовательских пункта: 0.Контроль гаража и 1.Настройки гаража.
  // пункт 0.Контроль гаража доступен для всех, пункт 1.Настройки гаража видит только пользователь с указанным Client ID  
  if (b.build.client.id == hubClientID) {
    b.Menu("Контроль гаража;Настройки гаража");
  } else {
    b.Menu("Контроль гаража");
  }
  
  b.show(b.menu() == 0); // если выбран пункт меню 0: Контроль гаража
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
  // горизонтальный контейнер с работой вентилятора
  if (b.beginRow()) {
    if (b.Switch(&ventAuto).label(F("АвтоРежим")).click()) (hubChanged = 1);
    if (ventAuto) {  // если вентилятор в режиме автоматического управления
      b.LED_("ventLed", &ventState).label(F("Вентилятор ON/OFF"));
    } else {
      if (b.Switch(&ventState).label(F("Вентилятор ON/OFF")).click()) (hubChanged = 1);
    }
    b.endRow();
  }
  // статус ворот - открыты или закрыты
  if (gateState) {  // если ворота открыты
    if (b.beginRow()) {
      if (b.Switch_(F("gateSwitch"), &gateState).label(F("Ворота открыты. Закрыть?")).click()) {
        closeGate();       // закрываем ворота         
      }
      b.LED_("PIR1", &pir1State).label(F("PIR1"));
      b.LED_("PIR2", &pir2State).label(F("PIR2"));
      b.Label_(F("IDLE"), idleSec).label(F("Время покоя")).color(gh::Colors::Aqua);
      b.endRow();
    }
  } else {
    b.Title(F("Ворота закрыты")).fontSize(32).color(gh::Colors::Green);
  }
  // статус машины - в гараже или нет
  if (b.beginRow()) {
    if (carStatus) {
      b.Title_("Car", F("Машина в гараже")).fontSize(32).color(gh::Colors::Green);
    } else {
      b.Title_("Car", F("Машины нет")).fontSize(32).color(gh::Colors::Mint);
    }
    b.endRow();
  }

  b.show(b.menu() == 1); // если выбран пункт меню 1: Настройки гаража  
  // добавляем спиннеры с поправкой влажности
  if (b.beginRow()) {
    if (b.Spinner(&myData.hum4xCorrection).range(-10, 10, 1).label(F("Поправка влажности на улице")).click()) myData.update();
    if (b.Spinner(&myData.hum3xCorrection).range(-10, 10, 1).label(F("Поправка влажности в гараже")).click()) myData.update();
    b.endRow();
  }  
  // добавляем спиннеры с дельтой включения вентилятора и гистерезисом
  if (b.beginRow()) {
    if (b.Spinner(&myData.deltaHumidity).range(0, 15, 1).label(F("Дельта влажности")).click()) myData.update();
    if (b.Spinner(&myData.hysteresis).range(0, 5, 1).label(F("Гистерезис")).click()) myData.update();
    b.endRow();
  }  
}  // end void build()

// Setup______________________________________________________________________________________________
void setup() {
  pinMode(pir1, INPUT_PULLDOWN);    // подтягиваем вход датчика pir1 к земле
  pinMode(pir2, INPUT_PULLDOWN);    // подтягиваем вход датчика pir2 к земле
  pinMode(relayVent, OUTPUT);       // пин реле вентилятора - выход
  pinMode(relayGate, OUTPUT);       // пин реле ворот - выход
  pinMode(gercon, INPUT_PULLDOWN);  // подтягиваем вход геркона к земле
  pinMode(triggerPin, OUTPUT);      // назначаем trigPin, как выход
  pinMode(echoPin, INPUT);          // назначаем echoPin, как вход
  if (digitalRead(gercon)) gateState = 1;
  else gateState = 0;                    // определяем исходное состояние ворот
  digitalWrite(relayGate, LOW);          // исходное низкое значение
  digitalWrite(relayVent, LOW);          // исходное низкое значение
  esp_task_wdt_init(WDT_TIMEOUT, true);  //enable panic so ESP32 restarts
  esp_task_wdt_add(NULL);                //add current thread to WDT watch
  
  LittleFS.begin();                      // инициализация файловой системы на флеше для записи настроек
  myData.read();                         // считываем данные настроек из флеша
  
  Serial.begin(115200);
  Wire.begin();                          // SensirionI2cSht3x.h and SensirionI2cSht4x.h
  sht3x.begin(Wire, SHT31_I2C_ADDR_45);  // SensirionI2cSht3x.h
  sht4x.begin(Wire, SHT41_I2C_ADDR_44);  // SensirionI2cSht4x.h
  bme.begin();                           // инициализируем датчик BMP280  
  
  oled.init();                   // инициализация дисплея
  oled.setContrast(10);          // яркость 0..255
  oled.textMode(BUF_REPLACE);    // вывод текста на экран с заменой символов
  oled.invertDisplay(oledFlag);  // вывод текста на экран с заменой символов

  initWiFi();  // установили соединение WiFi

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
  ArduinoOTA.begin();

  hub.mqtt.config("test.mosquitto.org", 1883);              // подключаем MQTT сервис
  hub.config(F(hubPrefix), F("Garage"), F("f494"));         // конфигурация GyverHub
  hub.onBuild(build);
  hub.begin();
  // устанавливаем начальные измеряемые значения для корректной работы фильтра checkValue
  sht3x.measureSingleShot(REPEATABILITY_HIGH, false, temperatureGarage, humidityGarage);  // SensirionI2cSht3x.h
  sht4x.measureHighPrecision(temperatureOut, humidityOut);                                // SensirionI2cSht4x.h
  temperatureBox = bme.readTemperature();
  pressure = pressureToMmHg(bme.readPressure());
}  // end void Setup()

// Loop_______________________________________________________________________________________________
void loop() {

  esp_task_wdt_reset();  // сбрасываем Watch Dog Timer чтобы не прошла перезагрузка
  ArduinoOTA.handle();   // поддерживаем работу ОТА

  myData.tick();         // сохранение настроек по таймауту
  
  hub.tick();                      // тикаем для нормальной работы конструктора интерфейса
  static gh::Timer tmr(2000);      // период 2 секунды
  if (tmr) {                       // если прошел период
    hub.sendUpdate("TempOut");     // обновляем значение температуры на улице
    hub.sendUpdate("HumOut");      // обновляем значение влажности на улице
    hub.sendUpdate("TempGarage");  // обновляем значение температуры в гараже
    hub.sendUpdate("HumGarage");   // обновляем значение влажности в гараже
    hub.sendUpdate("HumCalc");     // обновляем значение приведенной влажности
    hub.sendUpdate("Pressure");    // обновляем значение давления
    hub.sendUpdate("Car");         // обновляем статус машины
    if (gateState) {               // если ворота открыты
      idleSec = round(idleTime / 1000);
      hub.sendUpdate("PIR1");      // обновляем состояние PIR1
      hub.sendUpdate("PIR2");      // обновляем состояние PIR2
      hub.sendUpdate("IDLE");      // обновляем состояние IdleSec
    }
  }
  
  if (gateOpened && carStatus) gateOpenedTmr = millis(); // определили момент открытия ворот с машиной в гараже
   
  // если требуется изменить ПУ 
  if (hubChanged || gateOpened || gateClosed) {
    hub.sendRefresh();
    gateOpened = 0;
    gateClosed = 0;   
    hubChanged = 0;
  }
      
  // с периодом heat3xPeriod включаем прогрев датчика SHT31 на время heat3xTime
  // нагрев включается если измеренная влажность больше heat3xBorder
  // начальные значения heat3xFlag = 0, heat3xTmr = 0
  if ((humidityGarage > heat3xBorder) && (millis() - heat3xTmr >= (heat3xFlag ? heat3xTime : heat3xPeriod))) {
    heat3xTmr = millis();      // сброс таймера
    heat3xFlag = !heat3xFlag;  // переключаем флаг состояния нагрева датчика
    if (heat3xFlag) {          // если включен нагрев
      heat3xStart = millis();  // сохраняем время начала нагрева
      sht3x.enableHeater();    // включаем нагрев датчика
    } else {
      sht3x.disableHeater();   // выключаем нагрев датчика
    }
  }  // end If

  // подогреваем датчик SHT41 если Humidity > heat4xBorder
  // с периодом heat4xPeriod включаем прогрев датчика SHT41 на 1 секунду
  if ((humidityOut > heat4xBorder) && heat4xTmr.tick()) {
    heat4xStart = millis();                                              // сохраняем время начала нагрева датчика
    sht4x.activateHighestHeaterPowerLong(temperatureOut, tempHumidity);  // SensirionI2cSht4x.h
    humidityOut = tempHumidity + myData.hum4xCorrection;                 // SensirionI2cSht4x.h
    showScreen();                                                        // вывод показаний датчиков на экран
    delay(1000);                                                         // чтобы заметить макс. темп. и RH на дисплее. Не придумал, как обойтись без delay
  }                                                                      // end If

  if (sensorReadTmr.tick()) {  // если пришло время опроса датчиков
    sensorsRead();             // функция считывает все датчики и выставляет флаги
    showScreen();              // вывод показаний датчиков на экран
  }                            // end if

  if (oledTmr.tick()) {            // если пришло время инвертировать дисплей
    oledFlag = !oledFlag;          // инвертируем флаг состояния дисплея
    oled.invertDisplay(oledFlag);  // инвертируем дисплей
  }

  // включаем/выключаем вентилятор в зависимости от состояния ventState в ручном режиме
  if (!ventAuto && ventState ) digitalWrite(relayVent, HIGH);  // включаем вентилятор
  else digitalWrite(relayVent, LOW);                           // выключаем вентилятор

  // если машина в гараже в автоматическом режиме включаем вентилятор в зависимости от разницы влажности
  if (ventAuto && carStatus && (humidityGarage - humidityCalc >= myData.deltaHumidity + myData.hysteresis)) {
    digitalWrite(relayVent, HIGH); // включаем вентилятор
    ventState = 1;                 // устанавливаем флаг вентилятора
    hub.sendUpdate("ventLed");     // обновляем состояние вентилятора на ПУ
  } 
  // в автоматическом режиме выключаем вентилятор в зависимости от разницы влажности
  if (ventAuto && (humidityGarage - humidityCalc < myData.deltaHumidity - myData.hysteresis)) {
    digitalWrite(relayVent, LOW);  // выключаем вентилятор
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
  if (gateState && (idleTime > idleTimePeriod)) {
    closeGate();  // закрываем ворота    
  }

  // закрываем ворота через carLeavePeriod после выезда машины. carLeaveTmr отмечает время выезда машины
  // закрытие ворот при условии, что выезд произошел раньше, чем gateOpenedPeriod после открытия ворот
  // carLeaveTmr = 0 после закрытия ворот
  if(carLeaveTmr && gateState && (millis() - carLeaveTmr > carLeavePeriod) \
    && (millis() - gateOpenedTmr < gateOpenedPeriod)) {
    closeGate();  // закрываем ворота    
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
