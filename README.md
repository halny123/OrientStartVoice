# OrientStartVoice
Ver. 1.0.0
 
 Озвучка старта для соревнований по ориентированию. (Часы на старт говорильные)

Сделаны на основе лишней платы от SportIduino(Atmega328+DS3231) + MP3-TF-16P + усилитель 
  
 Питание АКБ 12v, ток потребления в режиме ожидания 20мА.
 
 Монтаж навесной 
 
 Структура папок на карточке mp3/0100.mp3
 
 Карточка должна быть отформатированна и папка MP3 залита за раз, т.к. мой модуль (на чипе MH2024K) отказался воспроизводить файлы из папок и по имени а получилось воспроизводить только в порядке записи в FAT.
 
 Запись расчитана на 3 часа старта
 
 0100.mp3-0280.mp3 - Файлы с озвучкой стартовой минуты
 
 0290.mp3 - Запись "До старта осталось 4 минуты"
 
 0291.mp3 - Запись "Конец старта"
 
 0300.mp3-0480.mp3 - Файлы с озвучкой фамилий стартующих
 
 как формировать автоматически фамилии стартующих опишу позже если это интересно.
  
 После подачи питания до старта 0 минуты остается 4 минуты
 
 Если дойдут руки сделать 3-ри кнопки. Перемотки вперед+1мин/назад-1мин/установка 0 секунд
