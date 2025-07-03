// Обработчик сообщений Телеграм бота
void newMsg(FB_msg& msg) {

  String msgID = msg.chatID;  // сохраняем chatID запроса, чтобы отправлять ответы только запросившему

  // if (msg.OTA && msg.chatID == ADMIN_ID) {    // разрешить обновление прошивки для Админа
  //   bot.tickManual();                        // отметить сообщение прочитанным и избежать бесконечного обновления
  //   bot.update();                            // telegram update 
  // }  

  if (msg.text == "/state_garage"|| msg.text == "Обзор") {
    String buf;
    (gateState) ? (buf = F("Ворота открыты\n")) : (buf = "Ворота закрыты\n");
    (ventState) ? (buf += F("Вент. включен\n")) : (buf += F("Вент. выключен\n"));
    (autoLightState || manualLightState) ? (buf += F("Прож. включен\n")) : (buf += F("Прож. отключен\n"));
    (ventAuto) ? (buf += F("Авто режим\n")) : (buf += F("Ручной режим\n"));
    (carStatus) ? (buf += F("Машина в гараже\n")) : (buf += F("Машины нет\n"));
    (isDark) ? (buf += F("На улице темно\n")) : (buf += F("На улице светло\n"));
    bot.sendMessage(buf, msgID);  // отправили сообщение
  }

  if (msg.text == "/switch_gate" || msg.text == "Ворота ON/OFF"){
    String buf;
    (gateState) ? (buf = "Ворота закрываются") : (buf = "Ворота открываются");
    switchGate();           // отправили команду на переключение ворот
    bot.sendMessage(buf, msgID);  // отправили сообщение
  }

  if (msg.text == "/weather_out" || msg.text == "Улица"){
    bot.sendMessage("Температура " + String(temperatureOut) + " Влажность " + String(humidityOut), msgID);  // отправили сообщение
  }

  if (msg.text == "/weather_in" || msg.text == "Гараж"){
    bot.sendMessage("Температура " + String(temperatureGarage) + " Влажность " + String(humidityGarage), msgID);  // отправили сообщение
  }

  if (msg.text == "/weather_box" || msg.text == "Корпус"){
    bot.sendMessage("Температура " + String(temperatureBox) + " Давление " + String(pressure), msgID);  // отправили сообщение
  }

  if (msg.text == "/vent_switch" || msg.text == "Вент. ON/OFF"){
    String buf;    
    if (!ventAuto) {   // если ручной режим
      ventState = !ventState;
      (ventState) ? (buf = "Вентилятор включен") : (buf = "Вентилятор отключен");   
      bot.sendMessage(buf, msgID);  // отправили сообщение
    } else {          // если автоматический режим
      buf = "Выключите Авто режим";
      bot.sendMessage(buf, msgID);  // отправили сообщение
    }  // end if (!ventAuto)
  }

  if (msg.text == "/light_switch" || msg.text == "Свет ON/Off"){
    String buf;
    lightButtonPressed = 1;         // отметили факт нажатия кнопки включения/выключения прожектора    
    if (!manualLightState) {        // если прожектор был выключен
      lightTmr = millis();          // отметили время включения
      buf = "Прожектор включен";
    } else {
      (buf = "Прожектор отключен");
    }
    bot.sendMessage(buf, msgID);  // отправили сообщение
  }

  if (msg.text == "/auto_switch" || msg.text == "Авто режим ON/OFF"){
    String buf;
    ventAuto = !ventAuto;
    (ventAuto) ? (buf = "Авто режим включен") : (buf = "Ручной режим включен");   
    bot.sendMessage(buf, msgID);  // отправили сообщение
  } 

}  // end void newMsg

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

// Функция обновляет содержимое chatId
void chatIdRefresh() {
  uint16_t counter = dataId.amount();  // количество пар
  chatId = "";
  for (uint16_t i = 1; i <= counter; i++) {
  String text2 = dataId.get(i-1).toString();     // индекс начинается с 0
  chatId += text2;
  if (i < counter) chatId += ",";   // добавляем разделительную запятую между отдельными telegram_id
  }
}  // end void chatIdRefresh()

// Процедура showScreen() выводит на экран значения температуры, влажности, пинг WiFi, RSSI
// и оставшееся время до импульса нагрева датчика
void showScreen() {
  // counterDown это время, оставшееся до включения нагрева датчика SHT4x
  float counterDown = (heat4xPeriod - (millis() - heat4xStart)) / 1000;
  oled.clear();                   // очищаем дисплей
  oled.setScale(2);               // масштаб текста (1..4)
  oled.setCursor(0, 0);           // курсор на начало 1 строки
  oled.print("H ");               // вывод H
  oled.print(humidityOut, 1);     // вывод значения Humidity
  oled.setCursor(0, 2);           // курсор на начало 2 строки
  oled.print("T ");               // вывод Т
  oled.print(temperatureOut, 1);  // вывод значения Temperature

  if (humidityOut <= heat4xBorder) {         // если значение влажности меньше heat4xBorder
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

// функция усредняет значение, измеренное датчиком, по алгоритму "бегущее среднее"
// с адаптивным коэффициентом фильтрации
// newValue - новое значение, требующее усреднения. filtValue - предыдущее уже усредненное
// delta максимальная разница, выше которой применяется повышенный коэффициент фильтрации
float filterValue(float newValue, float filtValue, float delta) {
  float k;    // коэффициент фильтрации. от 0 до 1. Чем меньше, тем плавнее фильтр
  // резкость фильтра зависит от модуля разности значений
  (abs(newValue - filtValue) > delta) ? (k = 0.3) : (k = 0.1);  
  filtValue += (newValue - filtValue) * k;
  return filtValue;  
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
  buf += ventState;       // вывод состояния вентилятора
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

// функция закрывает  и открывает ворота
void switchGate() {
  digitalWrite(relayGate, HIGH); // для замыкания выставляем HIGH
  delay(300);                    // немного ждем срабатывания контроллера ворот для открытия/закрытия    
  digitalWrite(relayGate, LOW);  // для размыкания выставляем LOW
  delay(1000);                   // немного ждем для гарантированного размыкания геркона    
  idleTimeTmr = millis();        // сброс таймера покоя
  idleTime = 0;                  // сброс времени покоя
}  // end switchGate

// функция считывает датчики температуры, влажности, давления 
void sensorsRead() {
  sensorOut.measureHighPrecision(tempTemperature, tempHumidity); // SensirionI2cSht4x.h
  if ((millis() - heat4xStart) < 30000) {                        // сразу после нагрева выводим данные как есть
    temperatureOut = tempTemperature;
    humidityOut = myData.humOutCorrection + tempHumidity;
  } else {  // через 30 секунд начинается фильтрация
    // temperatureOut = tempTemperature;
    // if (tempHumidity <= 50) humidityOut =  tempHumidity;                           // при малой влажности не используем поправку
    // if (tempHumidity > 50 && tempHumidity <= 80) humidityOut = (1 + myData.humOutCorrection / 30) * tempHumidity - (50 / 30) * myData.humOutCorrection; // поправка влажности линейно увеличивается 
    // if (tempHumidity > 80) humidityOut = tempHumidity + myData.humOutCorrection;   // при высокой влажности добавляем поправку целиком
    temperatureOut = filterValue(tempTemperature, temperatureOut, 0.05);    
    if (tempHumidity <= 50) humidityOut =  filterValue(tempHumidity, humidityOut, 0.05);    // при малой влажности не используем поправку
    if (tempHumidity > 50 && tempHumidity <= 80) humidityOut = filterValue((1 + myData.humOutCorrection / 30) * tempHumidity - (50 / 30) * myData.humOutCorrection, humidityOut, 0.05); // поправка влажности линейно увеличивается 
    if (tempHumidity > 80) humidityOut = filterValue(tempHumidity + myData.humOutCorrection, humidityOut, 0.05);   // при высокой влажности добавляем поправку целиком
  }
  sensorIn.measureSingleShot(REPEATABILITY_HIGH, false, tempTemperature, tempHumidity);  // SensirionI2cSht3x.h
  // temperatureGarage = tempTemperature;
  // humidityGarage = tempHumidity + myData.humInCorrection;
  temperatureGarage = filterValue(tempTemperature, temperatureGarage, 0.05);
  humidityGarage = filterValue(tempHumidity + myData.humInCorrection, humidityGarage, 0.05);  
  // sensorOut.measureSingleShot(REPEATABILITY_HIGH, false, tempTemperature, tempHumidity);  // SensirionI2cSht3x.h
  // temperatureOut = filterValue(tempTemperature, temperatureGarage, 0.05);
  // humidityOut = filterValue(tempHumidity + myData.humOutCorrection, humidityGarage, 0.05);
  temperatureBox = filterValue(bme.readTemperature(), temperatureBox, 0.05);        // GyverBME280.h
  pressure = filterValue((pressureToMmHg(bme.readPressure())), pressure, 0.05);     // GyverBME280.h
  // temperatureBox = bme.readTemperature();         // GyverBME280.h
  // pressure = pressureToMmHg(bme.readPressure());  // GyverBME280.h
  rssi = WiFi.RSSI();
  // считаем уличную влажность при температуре в гараже
  humidityCalc = humConversion(humidityOut, temperatureOut, temperatureGarage);
} // end void sensorsRead()

// функция считывает геркон, датчик дистанции, ПИР датчики и выставляет флаги и таймер выезда машины carLeaveTmr
void gateRead() {

  bool newGateState = digitalRead(gercon);  // считали состояние ворот  
  if (newGateState > gateState) {           // если ворота открылись, установили флаги
    gateOpened = 1;                         // при открытии ворот меняется содержимое ПУ
    gateClosed = 0;
  }
  if (newGateState < gateState) {  // если ворота закрылись, установили флаги
    gateOpened = 0;
    gateClosed = 1;                // при закрытии ворот меняется содержимое ПУ
    carLeaveTmr = 0;               // таймер нужно сбросить на 0 чтобы случайно не закрылись ворота
  }
  
  gateState = newGateState;  // установили флаг состояния ворот

  if (gateState) distance = sonar.ping_cm();  // если ворота открыты, определили расстояние до машины
  bool newCarStatus;
  (distance > 100) ? (newCarStatus = 0) : (newCarStatus = 1);  // определили статус машины - в гараже или нет
  // если машина выехала, установили таймер  выезда машины из гаража
  if (newCarStatus < carStatus) carLeaveTmr = millis(); 
  // если машина приехала, обнулили таймер  выезда машины из гаража
  if (newCarStatus > carStatus) carLeaveTmr = 0; 

  carStatus = newCarStatus;                                    // установили статус машины

  (digitalRead(pir1)) ? (pir1State = 1) : (pir1State = 0); // если появился сигнал на входе pir1
  (digitalRead(pir2)) ? (pir2State = 1) : (pir2State = 0); // если появился сигнал на входе pir2
  (digitalRead(pir3)) ? (pir3State = 1) : (pir3State = 0); // если появился сигнал на входе pir3
  (!pir1State && !pir2State && !pir3State) ? (idleState = 1) : (idleState = 0); // если все датчики в состоянии покоя
}  // end void gateRead()

// функция вычисляет давление насыщенного пара воды при заданной температуре (для апроксимации внесены табличные данные от -20С до 40С )
float vaporPressure(float t) {
  double k4 = 0.000000463; // коэффициент при четырехстепенном члене многочлена
  double k3 = 0.00002461;  // коэффициент при кубическом члене многочлена
  double k2 = 0.00135571;  // коэффициент при квадратичном члене многочлена
  double k1 = 0.0459398;   // коэффициент при линейном члене многочлена
  double k = 0.60534;      // коэффициент при нулевом члене многочлена
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
