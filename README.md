# GarageStation
К контроллеру гаража ESP32 подключены:
1. Датчик температуры и влажности SHT41 на улице.
2. Датчик температуры и влажности SHT31 в гараже.
3. Датчик температуры и атм. давления BMP280 в корпусе. Не то чтобы нужен, просто несложно.
4. Реле, включающее вентилятор.
5. Реле, включающее прожектор.
6. Геркон, контролирующий состояние ворот (открыты - закрыты).
7. Реле, открывающее и закрывающее ворота.
8. Три датчика движения в гараже.
9. Ультразвуковой датчик, контролирующий присутствие/отсутствие машины в гараже.

Управляется все это хозяйство  в автоматическом режиме, при необходимости можно и самому что-то 
включить - выключить, открыть - закрыть через приложение GyverHub или Телеграм-бот. 

Вентилятор включается и выключается в автоматическом режиме в зависимости от показаний датчиков.
Сложность в том, что из-за теплого гаража и больших суточных колебаний значений температуры и влажности
проблематично напрямую использовать данные, непосредственно снимаемые с датчиков.
Опять же очевидно, что необходимо соответствие показаний влажности и температуры обоих датчиков.
Пришлось проверить с десяток разных SHT31 и SHT41, чтобы подобрать. Проверял на насыщенных растворах NaCl (75% влажности пара) и MgCl2 (35% влажности пара).
Подобрал пару SHT31 и SHT41 с разницей показаний влажности 2,6 процента. Программно эта разница убирается в приложении.

Чтобы избавиться от проблемы суточных колебаний ввел расчет Приведенной влажности. Смысл вот в чем. 
Возьмем пакет с воздухом с улицы и поместим его в гараж. Уличный воздух приобретет температуру гаража,
при этом значение его влажности изменится - я назвал это значение Приведенной влажностью. Расчет относительно несложен.
С использованием графика зависимости давления насыщенного пара воды от температуры.
Если влажность внутри гаража существенно превышает Приведенную влажность и машина находится в гараже, включается вентиляция.
Если влажность в гараже ниже Приведенной влажности, включение вентиляции бессмысленно. 
Проверил эту идею, когда случались дождливые дни. Работает отлично. Разница между влажностью в гараже и Приведенной влажностью в сухую погоду держится на приблизительно одном уровне
независимо от сильных дневных колебаний температуры. С моими датчиками это в пределах 10%, обычно 5-8%.
Когда в гараж заезжает мокрая машина разница сразу становится 30-40%. 
Таким образом, в настройках приложения задаем дельту влажности скажем 15% и гистерезис 2%. Вентилятор гарантированно включится при заезде мокрой машины
и выключится когда визуально все будет сухо.

![image](https://github.com/user-attachments/assets/9cd0ade7-d9dd-412b-aa5e-b630253174d8)


Лог влажности в гараже. Значения при сухой погоде меняются от 30% до 60%. Мокрая машина при влажности в гараже больше 70%

![image](https://github.com/user-attachments/assets/8d78a85b-ac01-4ad8-973b-f58ee00c77a0)

Лог Приведенной влажности в том же временном промежутке.

![image](https://github.com/user-attachments/assets/545cca78-23b7-4736-88c4-5dc59a78578a)

Разница между значением влажности и значением Приведенной влажности. Промежутки с мокрой машиной отлично выделяются и бросаются в глаза.

Ворота закрываются автоматически при выполнении следующих условий:
 1. Отсутствие движения в гараже в течение заданного времени. Бывает, забыл закрыть гараж или случайно нажал кнопку открытия.
 2. Когда выезжаешь из гаража сначала открываются ворота, затем машина выезжает. При таком сценарии через заданное время ворота автоматически закрываются.

Ворота открываются только вручную кнопкой со штатного ключа или кнопкой в телеграм - боте.

Прожектор включается:
 1. Вручную по нажатию кнопки в телеграм-боте или приложении.
 2. Автоматически при открытии ворот в темное время суток. Период темного времени суток вычисляется автоматически каждый день по дате.

Прожектор отключается:
 1. Вручную по нажатию кнопки в телеграм-боте или приложении.
 2. После ручного включения отключается автоматически после достижения заданного периода времени.
 3. После автоматического включения отключается сразу после закрытия ворот . 

Кроме того, данные с датчиков отправляются на серверы Narodmon и Open Monitoring,
чтобы набирать статистику и иметь возможность просмотра логов с телефона в любой момент.


