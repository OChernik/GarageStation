// Функция устанавливает WiFi соединения
void initWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi ..");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(1000);
  }
  Serial.println(WiFi.localIP());
}  // end void initWiFi()

// Процедура showScreen() выводит на экран значения температуры, влажности, пинг WiFi, RSSI
// и оставшееся время до импульса нагрева датчика
void showScreen() {
  // counterDown это время, оставшееся до включения нагрева датчика SHT4x
  float counterDown = (heat4xPeriod - (millis() - heat4xStart)) / 1000;
  oled.clear();                      // очищаем дисплей
  oled.setScale(2);                  // масштаб текста (1..4)
  oled.setCursor(0, 0);              // курсор на начало 1 строки
  oled.print("H ");                  // вывод H
  oled.print(humidityGarage, 1);     // вывод значения Humidity
  oled.setCursor(0, 2);              // курсор на начало 2 строки
  oled.print("T ");                  // вывод Т
  oled.print(temperatureGarage, 1);  // вывод значения Temperature

  if (humidityGarage <= heat4xBorder) {         // если значение влажности меньше heat4xBorder
    oled.print((heat3xFlag) ? " On" : " Off");  // вывод "On" если датчик греется
  } else {                                      // если значение влажности больше heat4xBorder
    oled.print(" ");                            // вывод " " если датчик будет греться
    oled.print(counterDown, 0);                 // вывод значения времени до начала нагрева counterDown
  }                                             // end IF

  oled.setCursor(0, 4);     // курсор на начало 3 строки
  oled.print("P ");         // вывод P
  oled.print(pressure, 1);  // вывод значения pressure
  oled.setCursor(0, 6);     // курсор на начало 4 строки
  oled.print("RSSI ");      // вывод RSSI
  oled.print(rssi);         // вывод значения RSSI.
  oled.update();            // Вывод содержимого буфера на дисплей. Только при работе с буфером.
}  // end showScreen

// функция проверяет величину, измеренную датчиком, на вхождение в допустимый диапазон значений
// и на отсутствие резких изменений
// если измеренное значние соответствует диапазону, возвращает измеренное значение, если нет, то предыдущее
// newValue - новое значение, требующее проверки. oldValue - предыдущее значение
// minValue, maxValue - диапазон допустимых значений величины
// delta - максимальная допустимая разница между замерами
float checkValue(float newValue, float oldValue, int minValue, int maxValue, int delta) {
  if (newValue >= minValue && newValue <= maxValue && abs(newValue - oldValue) <= delta) return newValue;
  else return oldValue;
}

// функция отсылает данные на open-monitoring.online
void sendToOpenMon() {
  String buf;                                                             // Буфер для отправки
  buf.reserve(150);                                                       // Буфер для отправки
  buf += F("http://open-monitoring.online/get?cid=3627&key=");            // формируем заголовок
  buf += OpenMonKey;                                                      // формируем заголовок
  buf += F("&p1=");                                                       // формируем заголовок
  buf += temperatureOut;                                                  // вывод температуры улицы
  buf += F("&p2=");
  buf += humidityOut;  // вывод влажности улицы
  buf += F("&p3=");
  buf += temperatureGarage;  // вывод температуры гаража
  buf += F("&p4=");
  buf += humidityGarage;  // вывод влажности в гараже
  buf += F("&p5=");
  buf += pressure;  // вывод давления
  buf += F("&p6=");
  buf += temperatureBox;  // вывод температуры в корпусе
  buf += F("&p7=");
  buf += rssi;  // вывод силы сигнала Wi-Fi, dBm
  buf += F("&p8=");
  buf += carStatus;         // вывод статуса машины в гараже, есть или нет
  buf += F("&p9=");
  buf += humidityCalc;      // вывод приведенной влажности 
  http.begin(buf.c_str());  // отправляем сформированную строку
  http.GET();               // Send HTTP GET request
  http.end();               // Free resources
}

// функция отсылает данные на NarodMon
void sendToNarodMon() {
  String buf;  // Буфер для отправки
  buf += F("#ESP32");
  buf += WiFi.macAddress();
  buf += F("\n");
  buf.replace(":", "");
  Serial.println(buf);
  buf += F("#Temp1#");
  buf += temperatureOut;
  buf += F("#Температура улица SHT41\n");  //NarodMon: вывод температуры на улице
  buf += F("#RH1#");
  buf += humidityOut;
  buf += F("#Влажность улица SHT41\n");  //NarodMon: вывод влажности на улице
  buf += F("#Press#");
  buf += pressure;
  buf += F("#Атм. давление BMP280\n");  //NarodMon: вывод атмосферного давления
  buf += F("#Temp2#");
  buf += temperatureGarage;
  buf += F("#Температура гараж\n");  //NarodMon: вывод температуры в гараже
  buf += F("#RH2#");
  buf += humidityGarage;
  buf += F("#Влажность гараж\n");  //NarodMon: вывод влажности в гараже
  buf += F("#DBM#");
  buf += rssi;
  buf += F("#RSSI гараж\n");            //NarodMon: вывод силы сигнала Wi-Fi, dBm
  buf += F("##\n");                     //NarodMon: закрываем пакет
  client.connect("narodmon.ru", 8283);  //NarodMon: Подключаемся
  client.print(buf.c_str());            // И отправляем данные в сеть
  client.stop();                        // Разрываем соединение с сервером
}

// функция закрывает ворота
void closeGate() {
  digitalWrite(relayGate, HIGH);  // для замыкания выставляем HIGH
  delay(500);
  digitalWrite(relayGate, LOW);  // для размыкания выставляем LOW
  idleTimeTmr = millis();        // сброс таймера покоя
  idleTime = 0;                  // сброс времени покоя
}  // end closeGate

// функция считывает все датчики, выставляет флаги и таймер выезда машины carLeaveTmr
void sensorsRead() {
  sht4x.measureHighPrecision(tempTemperature, tempHumidity);  // SensirionI2cSht4x.h
  if ((millis() - heat4xStart) < 30000) {                     // сразу после нагрева выводим данные как есть
    temperatureOut = tempTemperature;
    humidityOut = myData.hum4xCorrection + tempHumidity;
  } else {  // через 30 секунд начинается фильтрация
    temperatureOut = checkValue(tempTemperature, temperatureOut, -35, 40, 2);
    humidityOut = myData.hum4xCorrection + tempHumidity;
    // humidityOut = myData.hum4xCorrection + checkValue(tempHumidity, humidityOut, 20, 95, 2);
  }
  sht3x.measureSingleShot(REPEATABILITY_HIGH, false, tempTemperature, tempHumidity);  // SensirionI2cSht3x.h
  temperatureGarage = checkValue(tempTemperature, temperatureGarage, -5, 35, 2);
  humidityGarage = myData.hum3xCorrection + checkValue(tempHumidity, humidityGarage, 20, 95, 3);
  temperatureBox = checkValue(bme.readTemperature(), temperatureBox, 5, 45, 1);        // GyverBME280.h
  pressure = checkValue((pressureToMmHg(bme.readPressure())), pressure, 720, 770, 1);  // GyverBME280.h
  rssi = WiFi.RSSI();
  // считаем уличную влажность при температуре в гараже (Приведенная влажность)
  humidityCalc = humConversion(humidityOut, temperatureOut, temperatureGarage);

  distance = sonar.ping_cm();  // определили расстояние до машины
  bool newCarStatus;
  (distance > 100) ? (newCarStatus = 0) : (newCarStatus = 1);  // определили статус машины - в гараже или нет
  // если машина выехала, установили таймер  выезда машины из гаража, если приехала, обнулили таймер выезда машины 
  (newCarStatus < carStatus) ? (carLeaveTmr = millis()) : (carLeaveTmr = 0);        
  
  carStatus = newCarStatus;                                    // установили статус машины

  bool newGateState = digitalRead(gercon);  // считали состояние ворот
  if (newGateState > gateState) {           // если ворота открылись, установили флаги
    gateOpened = 1;                         // при открытии ворот меняется содержимое ПУ
    gateClosed = 0;
  }
  if (newGateState < gateState) {  // если ворота закрылись, установили флаги
    gateOpened = 0;
    gateClosed = 1;                // при закрытии ворот меняется содержимое ПУ
    carLeaveTmr = 0;               // таймер нужно сбросить на 0 чтобы ворота не закрылись сразу после открытия 
  }
  // if (newGateState = gateState) {  // если состояние ворот не поменялось, установили флаги
  //   gateOpened = 0;
  //   gateClosed = 0;
  // }
  gateState = newGateState;  // установили флаг состояния ворот

  if (digitalRead(pir1)) pir1State = 1;
  else pir1State = 0;  // если появился сигнал на входе pir1
  if (digitalRead(pir2)) pir2State = 1;
  else pir2State = 0;  // если появился сигнал на входе pir2
  // если все датчики в состоянии покоя
  if (!pir1State && !pir2State) idleState = 1;
  else idleState = 0;
}

// функция вычисляет давление насыщенного пара воды при заданной температуре (для апроксимации внесены табличные данные от -20С до 40С )
float vaporPressure(float t) {
  double k4 = 0.000000463; // коэффициент при члене четвертой степени многочлена 
  double k3 = 0.00002461;  // коэффициент при члене третьей степени многочлена 
  double k2 = 0.00135571;  // коэффициент при члене второй степени многочлена 
  double k1 = 0.0459398;   // коэффициент при линейном члене многочлена
  double k = 0.60534;      // константа
  float pressure = k4 * pow(t, 4) + k3 * pow(t, 3) + k2 * pow(t, 2) + k1 * t + k;
  return pressure;
}

// функция переводит влажность h1 при температуре t1 во влажность при температуре t2
float humConversion(float h1, float t1, float t2) {
  float p1 = vaporPressure(t1); // расчитали ДНП при t1 
  float p2 = vaporPressure(t2); // расчитали ДНП при t2 
  float h2 = h1 * (p1 / p2);    // расчитали приведенную влажность
  return h2;
}
