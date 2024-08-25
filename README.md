# GarageStation
К контроллеру ESP32 подключены:
1. Датчик температуры и влажности SHT41 на улице.
2. Датчик температуры и влажности SHT31 в гараже.
3. Датчик температуры и атм. давления BMP280 в корпусе.
4. Реле, включающее вентилятор.
   
Идея в том, чтобы при повышенной влажности в гараже автоматически включался вытяжной вентилятор.
Пока надо набрать статистику, чтобы выработать правильные критерии для его включения.
Проблема в том, что из-за теплого гаража большая разница температур на улице и в гараже приводит
к очень приличной разнице во влажности даже без мокрых поверхностей внутри.
На данный момент вентилятор включается/выключается вручную с телефона. 
