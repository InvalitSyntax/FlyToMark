# FlyToMark

## Работа с aruco - навигация по ним

Топик `aruco_detect/markers`.

Для виртуальной машины:
+ по заданному изначально размеру для метки вычисляется растояние для неё в метрах:
  + ось X: направленна вправо
  + ось Y: направленна вниз
  > Для X плюсовые значения до маркера направлены впрао, для Y соответственно вниз (относительно переда дрона)
  ![наглядно](/_img/напраление_маркеров_топик.png)
---
Следующий этап - Subscriber.
Узнаём тип сообщения топика `aruco_detect/markers` через `rostopic info`
+прописываем в терминал `rostopic info aruco_detect/markers`, а так же узнаем частоту обновления топика `rostopic hz aruco_detect/markers`
  + `Type: aruco_pose/MarkerArray`
  + `average rate: 23.779
	min: 0.020s max: 0.064s std dev: 0.01021s window: 564`

То что выше оказалось не нужным, но на будущее пригодится
> [Пример подписчика на `aruco_detect/markers` из справки](https://clover.coex.tech/ru/aruco_marker.html#работа-с-результатом-распознавания-из-python)

### Код Subscriber'a
[объяснение чуть доделаного кода](subscriber.md)