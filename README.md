# RGB_Led_Panel
Это программа для Arduino Mega 2560.
Запускает китайскую RGB Led панель 32х64, которая отсказалась работать 
на стандартных драйверах RGB_matrix_panel и Adafruit, а именно все выводилось 
через строчку. Простыми задержками и конфигурацией ничего не добился, 
поэтому вставил код драйверов в проект.
Покачто все рисуется через drawPixel, в идеале надо переделать на 
memcpy/memset напрямую во фреймбуффер для скорости работы
