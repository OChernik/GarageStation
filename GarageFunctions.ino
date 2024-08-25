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
  float counterDown = (heat4xPeriod - (millis() - heat4xStart))/1000;
  oled.clear();                      // очищаем дисплей
  oled.setScale(2);                  // масштаб текста (1..4)
  oled.setCursor(0, 0);              // курсор на начало 1 строки
  oled.print("H ");                  // вывод H
  oled.print(humidityGarage, 1);     // вывод значения Humidity
  oled.setCursor(0, 2);              // курсор на начало 2 строки
  oled.print("T ");                  // вывод Т
  oled.print(temperatureGarage, 1);  // вывод значения Temperature

  if (humidityGarage <= heat4xBorder) {       // если значение влажности меньше heat4xBorder
    oled.print((heat3xFlag) ? " On" : " Off");  // вывод "On" если датчик греется
  } else {                                    // если значение влажности больше heat4xBorder
    oled.print(" ");                    // вывод " " если датчик будет греться
    oled.print(counterDown, 0);         // вывод значения времени до начала нагрева counterDown
  }  // end IF

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
  buf += F("http://open-monitoring.online/get?cid=3627&key=znU2bw&p1=");  // формируем заголовок
  buf += temperatureOut;                                                  // вывод температуры улицы
  buf += F("&p2=");
  buf += humidityOut;                                                     // вывод влажности улицы
  buf += F("&p3=");
  buf += temperatureGarage;                                               // вывод температуры гаража
  buf += F("&p4=");
  buf += humidityGarage;                                                  // вывод влажности в гараже
  buf += F("&p5=");
  buf += pressure;                                                        // вывод давления
  buf += F("&p6=");
  buf += temperatureBox;                                                  // вывод емпературы в корпусе
  buf += F("&p7=");
  buf += rssi;                                                            // вывод силы сигнала Wi-Fi, dBm
  http.begin(buf.c_str());  // отправляем сформированную строку
  http.GET();               // Send HTTP GET request
  http.end();               // Free resources
}

// функция отсылает данные на NarodMon
// ESP324022D803F9F4  // идентификатор прибора
void sendToNarodMon() {
  String buf;  // Буфер для отправки
  buf += F("#ESP324022D803F9F4");
  // buf += WiFi.macAddress();
  buf += F("\n");
  // buf.replace(":", "");  
  Serial.println(buf);
  buf += F("#Temp1#");
  buf += temperatureOut;
  buf += F("#Улица\n");  //NarodMon: вывод температуры на улице
  buf += F("#RH1#");
  buf += humidityOut;
  buf += F("#Улица\n");  //NarodMon: вывод влажности на улице
  buf += F("#Press#");
  buf += pressure;
  buf += F("#Улица\n");  //NarodMon: вывод атмосферного давления
  buf += F("#Temp2#");
  buf += temperatureGarage;
  buf += F("#Гараж\n");  //NarodMon: вывод температуры в гараже
  buf += F("#RH2#");
  buf += humidityGarage;
  buf += F("#Гараж\n");  //NarodMon: вывод влажности в гараже  
  buf += F("#DBM#");
  buf += rssi;
  buf += F("#Гараж\n");                //NarodMon: вывод силы сигнала Wi-Fi, dBm
  buf += F("##\n");                     //NarodMon: закрываем пакет
  client.connect("narodmon.ru", 8283);  //NarodMon: Подключаемся
  client.print(buf.c_str());            // И отправляем данные в сеть
  client.stop();                        // Разрываем соединение с сервером
}