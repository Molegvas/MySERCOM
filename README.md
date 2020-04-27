Проект MORO2020_2
Печатная плата, схема:      v43 
Дисплей:  TFT 1.8"  
Дата: Июль 2018 - Февраль 2020

   ###          Особенности реализации на данный момент.

***
Среда разработки - PlatformIO. Включена поддержка платформы Arduino для ESP32.

***
Поддержка Arduino автоматически предполагает, что программа будет работать в среде Free RTOS, хотя многое,как это характерно для Ардуино, скрыто. Так что философию операционной системы реального времени (RTOS) постичь придется, но вникать в тонкости... как говорится, "всё уже сделано за нас".

***
Разбивка на задачи производится в main.cpp. Задач всего на данный момент пять.
Производится выделение ресурсов для каждой задачи: память, приоритет, ядро.
Все наши задачи исполняются ядром 1, ядро 0 выделено для радиочастотных задач - BT и WiFi.
Учтите, что setup() выполнялся без обязательных требований по максимальному времени монопольного захвата ядра (13мс). Включение RTOS произойдет только после loop() {}. Цикл пустой, но может быть заполнен фоновой программой, которая автоматически будет оформлена в виде одной из задач по умолчанию.

***
####      Коротко и конкретно по каждой задаче:

Задача подключения к WiFi сети (полностью заимствована как есть) Период 10мс. Длительность 2-3мс.

Задача выдачи данных на дисплей (5 параметров, дисплей TFT 1.8") 250мс, 25мс.

Задача управления системой теплоотвода. Предполагается расширить функциональность, добавив 
слежение за правильностью подключения нагрузки, масштабирование тока и т.д, своего рода "инспектор". Менее 1мс, 0,2с.

Задача обслуживания выбора режима работы, она же управляет конечным автоматом выбранного режима вплоть до выхода из режима. И то и другое построены как конечные автоматы (FSM). Особенностью этой задачи является то, что она запускается не в порядке очереди задач, а по таймеру исключительно для того, чтобы измерять точно ампер-часы. 100мс, 100мс.

Задача управления измерениями напряжения с выбором диапазона, тока, температуры, напряжения с кнопок - всё с фильтрацией, линеаризацией и преобразованием в физические величины. 0,03мс, 10мс. 

***
По большому счету для обмена данными между задачами должны бы использоваться конвйеры и семафоры... Однако довольно длительное тестирование подтвердило, что разработчики ESP учли нечто такое... атомарность, транзакции ... Впрочем, в проекте нет глобальных переменных. Как-то обошлось.

***
О реализации конечных автоматов (SM, FSM). 
Они в проекте повсюду. Вдохновение почерпнуто из статьи Павла Бобкова:
https://chipenable.ru/index.php/programming-avr/item/90-realizatsiya-konechnogo-avtomata-state-machine.html
Однако реализация построена на виртуальных методах. Выгода очевидна - произвольное 
размещение классов, описывающих состояния, простая структура класса любого состояния, 
прозрачная проверка условий и переходов, хорошая самодокументированность. Но постичь философию конечных автоматов, не вникая в тонкости, придется.

***
Более-менее законченный вид имеет диспетчер выбора режима (mdispatcherfsm.h,.cpp) и
режим простого заряда (mcccvfsm.h,.cpp). Остальные пока не приведены к окончательному
виду, а потому как учебное пособие использоваться не могут. 

***
Между режимами нет никакой связи - что позволяет разрабатывать каждый в отдельности, разными разработчиками, сосредотачиваясь исключительно на одном из них. Некоторая избыточность при этом - 
небольшая плата. Реализация режима занимает в програмной памяти несколько десятых долей процента, проверено. Предполагается, что конечный пользователь, имея некоторый навык (...) сможет, используя шаблон режима, создать режим имени себя любимого без особого риска преобразования прибора в "кирпич". 

***
Общим ресурсом всех режимов является класс MTools, в котором определен довольно широкий
инструментарий для построения пользовательских режимов. Пока там много ненужного от первоначального варианта, не предполагавшего использование виртуальных методов. Чистка и оптимизация будет. Предполагается, что Tools для редактирования будет доступен только разработчику прибора, что естественно. 

***
Работа с энергонезависимой памятью (NVS) не вызывает затруднений. Использование имен и ключей сильно упрощает работу с данными, включая загрузку параметров при первом включении. Некоторый напряг может вызывать заложенная разработчиками ESP реакция на "испорченные" данные - рестарт. Мы как-то делали это иначе.

***
Обновление ПО...  По воздуху (оно так и называется - OTA) - заложено ESP-шниками. Память программ разделена на две равные части. В одной - рабочая программа, вторая - зарезервирована - полтора мегабайта!!! 
Что происходит при обновлении: новый код пишется в другую половину, там проверяется, потом рабочая программа безаварийно завершается, а управление передается новой. Красиво?
А что делать с таким подарком 1,5 мегабайта в "мирное время"? Думаю, что туда можно писать логи. Карточки то в проекте не предусмотрено - ну его этот раритет... как представлю, что в гараже, промасленными пальцами...

***     
Вывода на дисплей пока нет - закомментированные //Oled->xxx(); методы предназначались
для работы с OLED 1.54" 128x64. Проект MORO2020_2 предполагает использование TFT 1.8" 128x160 RGB дисплея. Структура вывода на экран пока окончательно не определена.   

20:18 2020.02.07
***
Вывод на дисплей TFT 1.8".
Получилось 9 строк. Шрифты разного размера. Строка-лента. Самый мелкий шрифт - 21 знакоместо.
Сеансы обновления занимают от 2 до 8,5мс

10:14 2020.02.14
***
Оптимизирован код режима Option, позволяющий пользователю регулировать в заданных пределах некоторые параметры: задержку включения, смещение показаний тока и напряжения, возврат к заводским регулировкам для каждого режима в отдельности. (не закончено)

***
Вывод на дисплей - для снижения заметности при обновлении реализован метод очистки строки путем повторной её записи цветом фона и записи новой только в том случае, если строка изменилась.

***
Оптимизирован код режимов CCCV и DC SUPPLY под новые возможности отображения и с использованием универсальных методов класса MTools вместо специализированных, коих стало слишком много))

***
Решено отказаться от параноидального размещения переменных в privat, что может быть спорно. Пока в реализациях режимов используются оба метода.

***
Режим DEVICE - это инструмент разработчика, облегчающий, например, вычисление коэффициентов для линеаризации характеристик прибора с использованием онлайн-калькулятора https://planetcalc.ru/5992/ .

***
   ###          Особенности реализации обмена между контроллерами.

   ###          Особенности реализации управления измерениями.

   ###          Особенности реализации PID-регулирования.

Для прототипирования выбран быстрый 32-битный пид-регулятор с фиксированной точкой для Arduino. 
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

Регулятор проверяет наличие нарушений диапазона параметров и не будет работать, если коэффициент находится вне диапазона. Все операции конфигурации возвращают bool, чтобы предупредить пользователя об ошибке. Функция err() проверяет состояние ошибки. Ошибки можно устранить с помощью функции clear().

#### Частота Выполнения

** Частота выполнения не определяется автоматически, это значительно повышает производительность регулятора. Вместо этого Ki и Kd масштабируются на этапе настройки. Очень важно вызывать step() с той скоростью, которую вы укажете. 

#### Вход и выход /////////

Входной сигнал и заданное значение являются int16_t, что соответствует ширине аналоговых контактов и позволяет разместить отрицательные показания и заданные значения. Выходные данные PID - это int16_t. Фактическая битовая ширина и знаковость вывода могут быть настроены. 
  
  * bits - выходная ширина будет ограничена значениями внутри этого битового диапазона. Допустимые значения от 1 до 16 
  * sign если true, то выходной диапазон равен от -2^(бит-1) до -2^(бит-1) -1. Если выходной диапазон false, то от 0 до 2^(бит-1)-1. 
  ** Максимальное выходное значение контроллера -32767 (даже в 16-битном беззнаковом режиме) ** 

#### Исполнение

Производительность FastPID варьируется в зависимости от коэффициентов. Когда коэффициент равен нулю, выполняется меньший расчет. Контроллер был сравнен с помощью Arduino UNO и приведенного ниже кода. 

|  Kp |  Ki |  Kd | Step Time (uS) |  
| --- | --- | --- | -------------- |
| 0.1 | 0.5 | 0.1 |      ~64       | 
| 0.1 | 0.5 | 0   |      ~56       | 
| 0.1 | 0   | 0   |      ~28       | 

Для сравнения отличный [ArduinoPID] (https://github.com/br3ttb/Arduino-PID-Library) библиотека занимает в среднем около 90-100 долларов США за шаг со всеми ненулевыми коэффициентами. 

#### ПРИКЛАДНОЙ ПРОГРАММНЫЙ ИНТЕРФЕЙС (API)

API стремится быть простым и понятным. Я не буду имплицировать функции в контроллере, которые были бы лучше реализованы вне контроллера.

FastPID()

Создайте контроллер по умолчанию. Все коэффициенты по умолчанию равны нулю. Не используйте сконструированный по умолчанию контроллер до тех пор, пока вы не вызовете setCoefficients() и setOutputconfig()


FastPID(float kp, float ki, float kd, float hz, int bits=16, bool sign=false)

Постройте контроллер, который готов к работе. Вызывает следующие вызовы:

configure(kp, ki, kd, hz, bits, sign);

bool setCoefficients(float kp, float ki, float kd, float hz);

Настройка ПИД коэффициентов. Коэффициенты ki и kd масштабируются на величину hz. Значение hz информирует PID о скорости, которую вы назовете step(). Вызывающий код отвечает за вызов step() со скоростью в hz. Возвращает значение false, если произошла ошибка конфигурации. Что может быть связано с предыдущим вызовом.


bool setOutputConfig(int bits, bool sign);

Установите конфигурацию ouptut по битам/знаку. Диапазон выхода будет следующим:

Для знака, равного true  2^(n-1) - 1 вниз до -2^(n-1)

Для знака, равного false  2^n-1 вниз до 0

** bits равен 16-это особый случай. ** Когда bits равен 16 и знак false диапазон выходных напряжений 32767 до 0

Возвращает значение false, если произошла ошибка конфигурации. Что может быть связано с предыдущим вызовом.

bool setOutputRange (int16_t min, int16_t max);

Установите диапазон ouptut непосредственно. Эффективный диапазон действия составляет:

* Мин:  от -32768 до 32766
* Макс: от -32767 до 32767

Min должно быть больше, чем max.

Возвращает значение false, если произошла ошибка конфигурации. Что может быть связано с предыдущим вызовом.

void clear();

Перезагрузите контроллер. Это должно быть сделано до того, как вы каким-либо образом измените конфигурацию.

bool configure(float kp, float ki, float kd, float hz, int bits=16, bool sign=false);

Массовая настройка контроллера. Эквивалентный:

clear();
setCoefficients(kp, ki, kd, hz);
setOutputConfig(bits, sign);

int16_t step(int16_t sp, int16_t fb);

Выполните один шаг контроллера и верните следующий вывод.

bool err();

Проверьте наличие ошибки конфиурации. Контроллер не будет работать, если эта функция возвращает true. 

#### Целочисленный Завод 

Приложения, которые управляют медленно движущимися системами и имеют ненулевой интегральный член, часто видят значительное превышение при запуске. 
Это вызвано интегральной суммой winidng up, поскольку она помнит долгое время вдали от заданной точки. Если это описывает вашу систему есть две вещи, которые вы можете сделать. 

##### Заводную решение: ограничить сумму 

В FastPID есть константы.h, которые контролируют максимально допустимый Интеграл. Понижение этих параметров предотвращает выход контроллера из строя. 
запоминание как можно большего смещения от заданного значения и позволит уменьшить превышение. 

#define INTEG_MAX    (INT32_MAX)
#define INTEG_MIN    (INT32_MIN)

Меняйте эти константы с осторожностью. Установка их слишком низко исправит вашу проблему превышения скорости, но это негативно скажется на способность регулятора регулировать нагрузку. Если вы не уверены в правильной константе используйте следующее решение вместо этого ограничения суммы. 

#### Ограничение Заводки: Ограниченное Регулирование 

Пид-контроллер лучше всего работает, когда система находится близко к заданному значению. Во время фазы запуска или в случае с a 
значительная экскурсия, вы можете полностью отключить пид-контроль. Пример этого можно найти в контроллере Sous-Vide 
пример в этом проекте. Ядро логики находится в этом коде: 

if (feedback < (setpoint * 0.9)) {
  analogWrite(PIN_OUTPUT, 1);
  myPID.clear();
}
else {
  analogWrite(PIN_OUTPUT, myPID.step(setpoint, feedback));
}

Код обходит ПИД, когда температура составляет менее 90% от заданного значения, просто включив нагреватель. Когда 
температура выше 90% от заданного значения пид включен. Фиксация вашего промаха таким образом дает вам гораздо лучший контроль
вашей системы без необходимости добавлять сложные, недопустимые и трудные для понимания функции к пид-контроллеру.

#### Sample Code 

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

======================

   