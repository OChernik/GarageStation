# GarageStation
К контроллеру гаража ESP32 подключены:
1. Датчик температуры и влажности SHT41 на улице.
2. Датчик температуры и влажности SHT31 в гараже.
3. Датчик температуры и атм. давления BMP280 в корпусе. Не то чтобы нужен, просто несложно.
4. Реле, включающее вентилятор.
5. Геркон, контролирующий состояние ворот (открыты - закрыты).
6. Реле, открывающее и закрывающее ворота.
7. Три датчика движения в гараже.
8. Ультразвуковой датчик, контролирующий присутствие/отсутствие машины в гараже.
   
Идея в том, чтобы при повышенной влажности в гараже автоматически включался вытяжной вентилятор.
Пока надо набрать статистику, чтобы выработать правильные критерии для его включения.
Проблема в том, что из-за теплого гаража большая разница температур на улице и в гараже приводит
к очень приличной разнице во влажности даже без мокрых поверхностей внутри.
На данный момент вентилятор включается/выключается вручную с телефона.

Ворота могут закрываться автоматически при выполнении следующих условий:
 1. Отсутствие движения в гараже в течение заданного времени. Бывает, забыл закрыть гараж или случайно нажал кнопку открытия.
 2. Ворота открылись и вскоре машина выехала из гаража.

Кроме того, данные с датчиков отправляются на серверы Narodmon и Open Monitoring,
чтобы набирать статистику.
