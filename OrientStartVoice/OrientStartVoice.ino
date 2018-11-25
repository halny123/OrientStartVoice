// Часы на старт говорильные
// Сделаны на основе лишней платы от SportIduino(Atmega328+DS3231) + MP3-TF-16P + усилитель PAM8610 + DC-DC 3В->12В
// Питание АКБ LiIo, ток потребления в режиме ожидания 20мА.
// Монтаж навесной 
// Структура папок на карточке mp3/0100.mp3
// Карточка должна быть отформатированна и папка MP3 залита за раз,
// т.к. мой модуль (на чипе MH2024K) отказался воспроизводить файлы из папок и по имени
// получилось воспроизводить только в порядке записи в FAT.
// Запись расчитана на 3 часа старта
// 0100.mp3-0280.mp3 - Файлы с озвучкой стартовой минуты
// 0290.mp3 - Запись "До старта осталось 4 минуты"
// 0291.mp3 - Запись "Конец старта"
// 0300.mp3-0480.mp3 - Файлы с озвучкой фамилий стартующих
// как формировать автоматически фамилии стартующих смотри на Gite
// Прерывание от часов - на INT0 (PD2)
// После подачи питания до старта 0 минуты остается 4 минуты
// Ver. 1.0.0

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

#include <avr/interrupt.h>  
#include "SoftwareSerial.h"
#include <mp3TF.h>
#include <Wire.h>
#include "Sodaq_DS3231.h"
#include <avr/sleep.h>

const uint8_t Sleep_ampfiler = 15;    // пин Busy_mp3
const uint8_t Busy_mp3 = 17;          // пин Busy_mp3
const uint8_t Int_pin = 2;            // пин INT DS3231
const uint8_t RX = 7;                 // SoftwareSerial RX
const uint8_t TX = 8;                 // SoftwareSerial TX

const uint8_t LED = 4;                // led diod pin
const uint8_t BUZ = 3;                // buzzer pin
const uint8_t MP3Volume = 254;        // volume MP3 Out

//  Устанавливаем начальное время -4 минута до нуля
DateTime dt(2000, 1, 1, 23, 59, 0, 1);

// Для установки прерывания от часов
uint8_t int_min = 0;
uint8_t int_hour = 0;
 
mp3TF mp3tf = mp3TF ();
SoftwareSerial SoundSerial(RX, TX);
 
uint16_t play_id =  1;                // Номер проигруемого трека в FAT
uint16_t time_now = 0;                // Перевод времени для вычисления номера нужного трека

DateTime now;

// ------------- Start  setup -------------
void setup()
{ 
Serial.begin(9600);
pinMode(Int_pin, INPUT_PULLUP);
pinMode(Busy_mp3, INPUT);
pinMode(Sleep_ampfiler, OUTPUT);
digitalWrite (Sleep_ampfiler, HIGH);

Wire.begin();
rtc.begin();
rtc.setDateTime(dt);                                //Adjust date-time as defined 'dt' above 
beep(200, 1);

SoundSerial.begin(9600);
  
mp3tf.init (&SoundSerial);
delay(2000);
mp3tf.volumeSet(MP3Volume);
mp3tf.playTrackPhisical(182);                       // на карточки треков 3часа*60мин+0мин = 181 следующие треки под сообщения
                                                    // 182 - до старта осталоси 4 мин 183 - пи-пи-пи
delay(100);                                         // Модуль не успевает выставить Busy
while (!digitalRead(Busy_mp3)){delay(500);}
digitalWrite (Sleep_ampfiler, LOW);                 // Ждем когда проговорит модуль и усыпляем
                           

rtc.clearINTStatus();
now = rtc.now();                                    //get the current date-time
rtc.enableInterrupts(now.hour(),now.minute(),55);   // interrupt at (h,m,s)
}
// ------------- End  setup -------------


// ------------- Start  loop -------------
void loop()
{
 now = rtc.now();                                   //get the current date-time    
 if( !digitalRead(Int_pin))
  {
    Wire.begin();
    now = rtc.now();                                //get the current date-time  
    rtc.clearINTStatus();  
 
     if (now.minute()==59) 
     {
      int_min = 0;
      int_hour = now.hour() +1;
      if (int_hour == 24) {int_hour = 0;}
     } 
      else {int_min = now.minute()+1;int_hour = now.hour();}
      
    rtc.enableInterrupts(int_hour,int_min,55);      // interrupt at (h,m,s)
    
    beep(200, 1); delay(800);beep(200, 1); delay(800);beep(200, 1); delay(800);beep(1000, 1);
    digitalWrite (Sleep_ampfiler, HIGH);
    delay(2000);
    now = rtc.now();                                //get the current date-time    
    time_now = now.hour()*60 + now.minute() +1 ;

 if ( time_now == play_id )                       // пропускаем -1 минуту
   {
    mp3tf.init (&SoundSerial);
    mp3tf.volumeSet(MP3Volume);
    mp3tf.playTrackPhisical(play_id);             //   Озвучка минуты
    delay(100);                                   // Модуль не успевает выставить Busy
    while (!digitalRead(Busy_mp3)){delay(500);}   // Ждем когда проговорит
    delay(1500);
    mp3tf.playTrackPhisical(play_id + 183);       //   Озвучка ФИО
    delay(100);                                   // Модуль не успевает выставить Busy
    while (!digitalRead(Busy_mp3)){delay(500);}   // Ждем когда проговорит
    play_id++;
    digitalWrite (Sleep_ampfiler, LOW);
   }
  }
delay(1000);
sleep();    
}
// ------------- End  loop -------------



// ------------- Start  beep -------------
void beep(uint16_t ms, uint8_t n)
{
  pinMode (LED, OUTPUT);
  pinMode (BUZ, OUTPUT);
 for (uint8_t i = 0; i < n; i++)
 {
  digitalWrite (LED, HIGH);
  tone (BUZ, 4000, ms);
  delay (ms);
  digitalWrite (LED, LOW);
  if (i < n - 1) {delay(ms);}
  }
} 
// ------------- End  beep -------------

// ------------- Start  обработка прерывания от INT0 -------------
ISR(INT0_vect)
{ 
 sleep_disable();
 cbi(EIMSK, INT0);                      //Бит INT0 (0) запрещает внешние прерывания INT0.
}
// ------------- End  обработка прерывания от INT0 -------------


// ------------- Start  sleep -------------
void sleep()
{
cbi(ADCSRA, ADEN);                      // disable ADC
set_sleep_mode(SLEEP_MODE_PWR_DOWN);
sleep_enable();
sbi(EIMSK, INT0);                       //Бит INT0 (1) разрешает внешние прерывания INT0.
sleep_mode();
}  
// ------------- End  sleep -------------
