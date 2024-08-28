// реле управления вентилятором GPIO16
// реле управления воротами GPIO18
// вход геркона GPIO25
// Датчик SHT41, SHT31  SDA - 21, SCL - 22
// Экран SSH1106 1,3''  SDA - 21, SCL - 22
// Esp32 с антенной 192.168.31.85

// Настройки____________________________________________________________________
#define relayVent 16                      // relay to GPIO16
#define relayGate 18                      // relay to GPIO18
#define gercon 25                         // геркон to GPIO25
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
#include <Arduino.h>
#include <MyTimer.h>   // тестовая библиотека простейшего таймера моего изготовления
#include <GyverHub.h>  // GyverHub
#include <EEPROM.h>    // стандартная библиотека управления энергонезависимой памятью

// Объекты библиотек____________________________________________________________________________________
SensirionI2cSht3x sht3x;                  // создание объекта датчика sht3x библиотеки SensirionI2cSht3x
SensirionI2cSht4x sht4x;                  // создание объекта датчика sht4x библиотеки SensirionI2cSht4x
GyverBME280 bme;                          // Создание обьекта bme
GyverOLED<SSH1106_128x64> oled;           // создание объекта экрана SSH1106 1,3''
HTTPClient http;                          // создаем объект http библиотеки HTTPClient
WiFiClient client;                        // создаем объект client библиотеки WiFiClient
MyTimer oledTmr(oledInvertPeriod);        // создаем объект oledTmr таймера MyTimer с периодом oledInvertPeriod
MyTimer heat4xTmr(heat4xPeriod);          // создаем объект heat4xTmr таймера MyTimer с периодом heat4xPeriod
MyTimer checkWifiTmr(checkWifiPeriod);    // создаем объект checkWifiTmr таймера MyTimer с периодом checkWifiPeriod
MyTimer sensorReadTmr(sensorReadPeriod);  // создаем объект sensorReadTmr таймера MyTimer с периодом sensorReadPeriod
GyverHub hub;                             // создаем объект GyverHub

// Переменные___________________________________________________________________________________
float temperatureGarage;     // значение температуры в гараже
float temperatureOut;        // значение температуры на улице
float temperatureBox;        // значение температуры в коробке
float humidityGarage;        // значение влажности в гараже
float humidityOut;           // значение влажности на улице
float pressure;              // значение давления
float tempTemperature;       // первичное значение температуры с датчика до проверки на выброс
float tempHumidity;          // первичное значение влажности с датчика до проверки на выброс
int8_t rssi;                 // переменная измеренного значения rssi, dB
int8_t hum3xCorrection = 0;  // поправка измеренного значения влажности SHT31
int8_t hum4xCorrection = 0;  // поправка измеренного значения влажности SHT41
uint32_t heat3xTmr = 0;      // переменная таймера нагрева датчика SHT31
uint32_t openMonTmr = 0;     // переменная таймера отправки сообщений на сервер open-monitoring.online
uint32_t narodMonTmr = 0;    // переменная таймера отсылки данных на сервер NarodMon
uint32_t heat3xStart = 0;    // переменная времени начала нагрева датчика SHT31
uint32_t heat4xStart = 0;    // переменная времени начала нагрева датчика SHT41
bool heat3xFlag = 0;         // флаг нагрева датчика SHT31
bool heat4xFlag = 0;         // флаг нагрева датчика SHT41
bool oledFlag = 0;           // флаг состояния инверсии дисплея
bool relayGateState = 0;     // состояние реле ворот. 0 - разомкнуто, 1 - замкнуто.
bool gateState;              // состояние ворот. 0 - закрыты (геркон разомкнут), 1 - открыты
bool gateStateChanged = 0;   // состояние ворот изменилось. Для обновления ПУ
bool idleState = 0;          // состояние общего покоя. 0 - покой, 1 - движение
bool ventState = 0;          // состояние вентилятора. 0 - выключен, 1 - включен
bool ventAuto = 0;           // управление вентилятором. 0 - ручное, 1 - автоматическое
bool hubChanged = 0;         // 1 - требуется изменить конфигурацию виджетов ПУ
uint32_t idleTimeTmr = 0;    // переменная таймера состояния покоя
uint32_t idleTime;           // время покоя

// const char* ssid = "***";
// const char* password = "***";
const char* ssid = "****";
const char* password = "*****

void build(gh::Builder& b) {      // билдер GyverHub.
  b.Title(F("Климат в гараже"));  // добавим заголовок
                                  // горизонтальный контейнер с температурой и влажностью на улице
                                  // Функции beginRow() и beginCol() всегда возвращают true
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
  // горизонтальный контейнер с температурой и давлением в корпусе
  if (b.beginRow()) {
    b.Label_(F("TempBox"), temperatureBox).label(F("Температура в корпусе")).color(gh::Colors::Red);
    b.Label_(F("Pressure"), pressure).label(F("Атмосферное давление")).color(gh::Colors::Aqua);
    b.endRow();
  }
  // добавляем слайдеры с поправкой влажности
  if (b.beginRow()) {
    if (b.Slider(&hum4xCorrection).range(-10, 10, 1).label(F("Поправка влажности на улице")).click()) {
      EEPROM.write(INIT_ADDR + 2, hum4xCorrection);  // записали измененное значение humCorrection
      EEPROM.commit();                               // сохранили изменения в эмулированной ЕЕПРОМ во флеш памяти для esp8266/esp32
    }
    if (b.Slider(&hum3xCorrection).range(-10, 10, 1).label(F("Поправка влажности в гараже")).click()) {
      EEPROM.write(INIT_ADDR + 1, hum3xCorrection);  // записали измененное значение humCorrection
      EEPROM.commit();                               // сохранили изменения в эмулированной ЕЕПРОМ во флеш памяти для esp8266/esp32
    }
    b.endRow();
  }
  // горизонтальный контейнер с работой вентилятора
  if (b.beginRow()) {
    if (b.Switch(&ventAuto).label("АвтоРежим").click()) (hubChanged = 1);
    if (ventAuto) {  // если вентилятор в режиме автоматического управления
      b.LED_("ventLed", &ventState).label("Вентилятор ON/OFF");
    } else {
      if (b.Switch(&ventState).label("Вентилятор ON/OFF").click()) (hubChanged = 1);
    }
    b.endRow();
  }
  // статус ворот
  if (gateState) {  // если ворота открыты
    if (b.Switch_("gateSwitch", &gateState).label("Ворота открыты. Закрыть?").click()) {
      closeGate();     // закрываем ворота      
    }
  } else {
    b.Title(F("Ворота закрыты"));
  }
}  // end void build()

// Setup______________________________________________________________________________________________
void setup() {

  pinMode(relayVent, OUTPUT);       // пин реле вентилятора - выход
  pinMode(relayGate, OUTPUT);       // пин реле ворот - выход
  pinMode(gercon, INPUT_PULLDOWN);  // подтягиваем вход геркона к земле
  if (digitalRead(gercon)) gateState = 1;
  else gateState = 0;                    // определяем исходное состояние ворот
  digitalWrite(relayGate, LOW);          // исходное низкое значение
  digitalWrite(relayVent, LOW);          // исходное низкое значение
  esp_task_wdt_init(WDT_TIMEOUT, true);  //enable panic so ESP32 restarts
  esp_task_wdt_add(NULL);                //add current thread to WDT watch

  EEPROM.begin(3);                                 // инициализация на ESP32 ЕЕРROM размером 3 байта
  if (EEPROM.read(INIT_ADDR) != INIT_KEY) {        // в случае первого запуска
    EEPROM.write(INIT_ADDR, INIT_KEY);             // записали ключ
    EEPROM.write(INIT_ADDR + 1, hum3xCorrection);  // записали стандартное значение hum3xCorrection
    EEPROM.write(INIT_ADDR + 2, hum4xCorrection);  // записали стандартное значение hum4xCorrection
    EEPROM.commit();                               // сохранили изменения в эмулированной ЕЕПРОМ во флеш памяти для esp8266/esp32
  } else {
    hum3xCorrection = EEPROM.read(INIT_ADDR + 1);  // восстановили ранее записанное значение hum3xCorrection
    hum4xCorrection = EEPROM.read(INIT_ADDR + 2);  // восстановили ранее записанное значение hum4xCorrection
  }                                                // end if

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
  hub.config(F("ChernikDevices"), F("Garage"), F("f494"));  // конфигурация GyverHub
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

  hub.tick();                      // тикаем для нормальной работы конструктора интерфейса
  static gh::Timer tmr(2000);      // период 2 секунды
  if (tmr) {                       // если прошел период
    hub.sendUpdate("TempOut");     // обновляем значение температуры на улице
    hub.sendUpdate("HumOut");      // обновляем значение влажности на улице
    hub.sendUpdate("TempGarage");  // обновляем значение температуры в гараже
    hub.sendUpdate("HumGarage");   // обновляем значение влажности в гараже
    hub.sendUpdate("TempBox");     // обновляем значение температуры в коробке
    hub.sendUpdate("Pressure");    // обновляем значение давления
  }
  // если требуется изменить ПУ
  if (hubChanged || gateStateChanged) {
    hub.sendRefresh();
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
      sht3x.disableHeater();  // выключаем нагрев датчика
    }
  }  // end If

  // подогреваем датчик SHT41 если Humidity > heat4xBorder
  // с периодом heat4xPeriod включаем прогрев датчика SHT41 на 1 секунду
  if ((humidityOut > heat4xBorder) && heat4xTmr.tick()) {
    heat4xStart = millis();                                              // сохраняем время начала нагрева датчика
    sht4x.activateHighestHeaterPowerLong(temperatureOut, tempHumidity);  // SensirionI2cSht4x.h
    humidityOut = tempHumidity + hum4xCorrection;                        // SensirionI2cSht4x.h
    showScreen();                                                        // вывод показаний датчиков на экран
    delay(1000);                                                         // чтобы заметить макс. темп. и RH на дисплее. Не придумал, как обойтись без delay
  }                                                                      // end If

  // если пришло время опроса датчиков
  if (sensorReadTmr.tick()) {
    sensorsRead();
    showScreen();  // вывод показаний датчиков на экран
  }                // end if

  if (oledTmr.tick()) {            // если пришло время инвертировать дисплей
    oledFlag = !oledFlag;          // инвертируем флаг состояния дисплея
    oled.invertDisplay(oledFlag);  // инвертируем дисплей
  }

  // включаем/выключаем вентилятор в зависимости от состояния ventState
  if (ventState) digitalWrite(relayVent, HIGH);  // включаем вентилятор
  else digitalWrite(relayVent, LOW);             // выключаем вентилятор

  // восстанавливаем соединение при случайной пропаже
  if (checkWifiTmr.tick() && (WiFi.status() != WL_CONNECTED)) {
    Serial.println("Reconnecting to WiFi...");
    WiFi.disconnect();
    initWiFi();  // установили соединение WiFi
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
