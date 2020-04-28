#          Адаптер силового блока зарядного устройства

##   Кратко 

Адаптер силового блока предназначен для соединения управляющего контроллера с силовым блоком используя минимальное количество связи и оптимизации схем измерения и управления, освобождая центральный микроконтроллер от исполнения рутинных операций.

   ###          Аппаратная поддержка

Не углубляясь в причину выбора микроконтроллера в пользу семейства SAMD20 - наличие  поддержки сообществом Arduino сыграло не последнюю роль. Хотя там и нет поддержки SAMD20, но есть поддержка старшей модели SAMD21, миграция на которую, надеюсь, не составит особого труда. 
   Прототипирование выполнялось на плате SAMD21-M0-Mini. Подробности включения хорошо описаны в https://github.com/BLavery/SAMD21-M0-Mini.
  К сожалению поддержка технологией Arduino не в полной мере соответствует потребностям проекта, но это преодолимо.
 
   ###          Особенности реализации обмена между контроллерами.

  Протокол WAKE является логическим уровнем интерфейса управления оборудованием с помощью асинхронного последовательного канала. Физический уровень интерфейса протоколом не определяется, может использоваться, например, RS-232, RS-485 или USB. Протокол позволяет производить обмен пакетами данных (data frames) длиной до 255 байт с адресуемыми устройствами, которых может быть до 127. Формат данных может быть любым. Могут передаваться байтовые поля, многобайтовые слова или строки символов. Для контроля правильности передачи данных используется контрольная сумма (CRC-8).
 Подробная информация о WAKE: http://leoniv.diod.club/articles/wake/wake.html
Выбор этого протокола объясняется тем, что хорошо известен автору и многолетней работой доказал свою надежность в устройстве ротации кондиционеров.
 Первоначальные планы использовать в качестве физического носителя SPI разбились об прозу жизни - в наличии не оказалось соответствующего адаптера USB-3SPI. Для целей прототипирования используется USB-UART терминал WakeUp! от Ридико Леонида Ивановича. Найдете в "resources". 

   ###          Особенности реализации управления измерениями.

... в разработке

   ###          Особенности реализации PID-регулирования.

  Для прототипирования выбран быстрый 32-битный пид-регулятор с фиксированной точкой для Arduino:
FastPID v1.3.1 by Mike Matera 
https://github.com/mike-matera/FastPID

  Этот пид-регулятор работает быстрее, поскольку он позволяет избежать дорогостоящих операций с плавающей запятой. Пид-регулятор сконфигурирован с коэффициентами с плавающей запятой и переводит их в фиксированную точку внутренне. Это накладывает ограничения на область применения коэффициентов. Установка условий I и D на ноль ускоряет работу регулятора. Регулятор сконфигурирован для работы на фиксированной частоте, и вызывающий код отвечает за работу на этой частоте. Параметры Ki и Kd масштабируются по частоте, чтобы сэкономить время во время операции step(). 

#### Описание коэффициентов регулирования

  * Kp - пропорциональный 
  * Ki - интегральный 
  * Kd - дифференциальный 
  * Hz - частота выполнения контроллера 

#### Диапазон значений коэффициентов 

Конвейер вычислений ожидает 16-битные коэффициенты. Это контролируется PARAM_BITS и не должно быть изменено, иначе может возникнуть нежелательное переполнение. Количество битов до и после десятичного знака контролируется параметром PARAM_SHIFT в FastPID.h. значение по умолчанию для PARAM_SHIFT равно 8 и может быть изменено в соответствии с вашим приложением.

  * Диапазон параметра P от 0.00390625 до 255 включительно.** 
  * Параметр I равен P / Hz** 
  * Параметр D равен P * Hz** 
* ** До выяснения реальной потребности в диапазоне диапазон P установлен от 0,01 до 255.
Регулятор проверяет наличие нарушений диапазона параметров и не будет работать, если коэффициент находится вне диапазона. Все операции конфигурации возвращают bool, чтобы предупредить пользователя об ошибке. Функция err() проверяет состояние ошибки. Ошибки можно устранить с помощью функции clear().

#### Частота Выполнения

* ** Частота выполнения не определяется автоматически, это значительно повышает производительность регулятора. Вместо этого Ki и Kd масштабируются на этапе настройки. Очень важно вызывать step() с той скоростью, которую вы укажете. 

#### Вход и выход

Входной сигнал и заданное значение являются int16_t, что соответствует разрядности АЦП и позволяет разместить отрицательные показания и заданные значения. Выходные данные PID - это int16_t. Фактическая битовая ширина и знаковость вывода могут быть настроены. 
  
  * bits - выходная ширина будет ограничена значениями внутри этого битового диапазона. Допустимые значения от 1 до 16 
  * sign если true, то выходной диапазон равен от -2^(бит-1) до -2^(бит-1) -1. Если выходной диапазон false, то от 0 до 2^(бит-1)-1. 
  * ** Максимальное выходное значение контроллера -32767 (даже в 16-битном беззнаковом режиме) ** 

#### Исполнение

Производительность FastPID варьируется в зависимости от коэффициентов. Когда коэффициент равен нулю, выполняется меньший расчет**. 

|  Kp |  Ki |  Kd | Step Time (uS) |  
| --- | --- | --- | -------------- |
| 0.1 | 0.5 | 0.1 |      ~64       | 
| 0.1 | 0.5 | 0   |      ~56       | 
| 0.1 | 0   | 0   |      ~28       | 

* ** На данный момент не проверялось.

#### ПРИКЛАДНОЙ ПРОГРАММНЫЙ ИНТЕРФЕЙС (API)
Здесь и далее приводится авторский текст из указанного первоисточника с незначительными изменениями.

API стремится быть простым и понятным. Я не буду имплицировать функции в регуляторе, которые были бы лучше реализованы вне его.
```c++
FastPID()
```
Создайте контроллер по умолчанию. Все коэффициенты по умолчанию равны нулю. Не используйте сконструированный по умолчанию контроллер до тех пор, пока вы не вызовете setCoefficients() и setOutputconfig()

```c++
FastPID(float kp, float ki, float kd, float hz, int bits=16, bool sign=false)
```
Постройте контроллер, который готов к работе. Вызывает следующие вызовы:
```c++
configure(kp, ki, kd, hz, bits, sign);

bool setCoefficients(float kp, float ki, float kd, float hz);
```
Настройка ПИД коэффициентов. Коэффициенты ki и kd масштабируются на величину hz. Значение hz информирует PID о скорости, которую вы назовете step(). Вызывающий код отвечает за вызов step() со скоростью в hz. Возвращает значение false, если произошла ошибка конфигурации. Что может быть связано с предыдущим вызовом.


bool setOutputConfig(int bits, bool sign);

Установите конфигурацию ouptut по битам/знаку. Диапазон выхода будет следующим:

Для знака, равного true  2^(n-1) - 1 вниз до -2^(n-1)

Для знака, равного false  2^n-1 вниз до 0

* ** bits равен 16-это особый случай. 
* ** Когда bits равен 16 и знак false диапазон выходных напряжений 32767 до 0

Возвращает значение false, если произошла ошибка конфигурации. Что может быть связано с предыдущим вызовом.
```c++
bool setOutputRange (int16_t min, int16_t max);
```
Установите диапазон ouptut непосредственно. Эффективный диапазон действия составляет:

* Мин:  от -32768 до 32766
* Макс: от -32767 до 32767

Min должно быть больше, чем max.

Возвращает значение false, если произошла ошибка конфигурации. Что может быть связано с предыдущим вызовом.
```c++
void clear();
```
Перезагрузите контроллер. Это должно быть сделано до того, как вы каким-либо образом измените конфигурацию.
```c++
bool configure(float kp, float ki, float kd, float hz, int bits=16, bool sign=false);
```
Массовая настройка контроллера. Эквивалентный:
```c++
clear();
setCoefficients(kp, ki, kd, hz);
setOutputConfig(bits, sign);

int16_t step(int16_t sp, int16_t fb);
```
Выполните один шаг контроллера и верните следующий вывод.
```c++
bool err();
```
Проверьте наличие ошибки конфиурации. Контроллер не будет работать, если эта функция возвращает true. 

#### Целочисленный Завод 

Приложения, которые управляют медленно движущимися системами и имеют ненулевой интегральный член, часто видят значительное превышение при запуске. 
Это вызвано интегральной суммой winidng up, поскольку она помнит долгое время вдали от заданной точки. Если это описывает вашу систему есть две вещи, которые вы можете сделать. 

##### Заводную решение: ограничить сумму 

В FastPID есть константы.h, которые контролируют максимально допустимый Интеграл. Понижение этих параметров предотвращает выход контроллера из строя. 
запоминание как можно большего смещения от заданного значения и позволит уменьшить превышение. 
```c++
#define INTEG_MAX    (INT32_MAX)
#define INTEG_MIN    (INT32_MIN)
```
Меняйте эти константы с осторожностью. Установка их слишком низко исправит вашу проблему превышения скорости, но это негативно скажется на способность регулятора регулировать нагрузку. Если вы не уверены в правильной константе используйте следующее решение вместо этого ограничения суммы. 

#### Ограничение Заводки: Ограниченное Регулирование 

Пид-контроллер лучше всего работает, когда система находится близко к заданному значению. Во время фазы запуска или в случае с a 
значительная экскурсия, вы можете полностью отключить пид-контроль. Пример этого можно найти в контроллере Sous-Vide 
пример в этом проекте. Ядро логики находится в этом коде: 
```c++
if (feedback < (setpoint * 0.9)) {
  analogWrite(PIN_OUTPUT, 1);
  myPID.clear();
}
else {
  analogWrite(PIN_OUTPUT, myPID.step(setpoint, feedback));
}
```
Код обходит ПИД, когда температура составляет менее 90% от заданного значения, просто включив нагреватель. Когда 
температура выше 90% от заданного значения пид включен. Фиксация вашего промаха таким образом дает вам гораздо лучший контроль
вашей системы без необходимости добавлять сложные, недопустимые и трудные для понимания функции к пид-контроллеру.

#### Sample Code 
```c++
#include <FastPID.h>

#define PIN_INPUT     A0
#define PIN_SETPOINT  A1
#define PIN_OUTPUT    9

float Kp=0.1, Ki=0.5, Kd=0.1, Hz=10;
int output_bits = 8;
bool output_signed = false;

FastPID myPID(Kp, Ki, Kd, Hz, output_bits, output_signed);

void setup()
{
  Serial.begin(9600);
  if (myPID.err()) {
    Serial.println("There is a configuration error!");
    for (;;) {}
  }
}

void loop()
{
  int setpoint = analogRead(PIN_SETPOINT) / 2; 
  int feedback = analogRead(PIN_INPUT);
  int ts = micros();
  uint8_t output = myPID.step(setpoint, feedback);
  int tss = micros();
  analogWrite(PIN_OUTPUT, output);
  Serial.print("(Fast) micros: "); 
  Serial.print(tss - ts);
  Serial.print(" sp: "); 
  Serial.print(setpoint); 
  Serial.print(" fb: "); 
  Serial.print(feedback);
  Serial.print(" out: ");
  Serial.println(output);
  delay(100);
}
```
###		Справочная информация

#### Команды АЦП-преобразований

* 0x20	- 
* 0x21	- 

#### Команды ПИД-регулятора

|  код  |        имя              |        параметры           |    примечание       |
|-------|-------------------------|----------------------------|---------------------|
| 0x40	| cmd_pid_configure       | kp, ki, kd, hz, bits, sign |                     |
| 0x41	| cmd_pid_coefficients    | kp, ki, kd, hz             |                     |
| 0x42	| cmd_pid_output_range    | min, max                   |                     |
| 0x43	| cmd_pid_output_config   | bits, sign                 |                     |
| 0x44	| cmd_pid_clear           |                            |                     |
| 0x45	| cmd_pid_stop_go         | (стоп-пауза-пуск)          |                     |
| 0x46	| cmd_pid_test            | mode, setpoint, min, max   |                     |
| 0x47	|                         |                            |                     |
|-------|-------------------------|----------------------------|---------------------| 
* kp, ki, kd - целочисленное, полученные умножением на множитель 100