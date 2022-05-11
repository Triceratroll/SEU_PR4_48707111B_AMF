/**
  ******************************************************************************
  * Comentarios funcionamineto del código
  ******************************************************************************
  * La totalidad del código necesario para hacerlo funcionar se encunetra en el
  * fichero main.c
  *
  * Por orden de aparición, en primer lugar en lugar reservado para ellas por el
  * templeate que se autogenera, tenemos las varibales globales del programa. A
  * conttinuación tenemos las funciones auxiliares, estas son :
  *
  * readPOT() para obtener el valor del potenciometro, condfiguramos el canal,
  * leemos el valor y hacemos una conversión para devolver un valor entre 1 y 100.
  *
  * readLUZ() para obtener el valor del sensor LDR, de igual forma configuramos
  * el canal, leemos el valor, y en este caso hacemos dos conversiones primero
  * invertimos el valor obtenido debido a que de esta forma más luz correspoonderá
  * a un valor más alto y después devolvemos un valor entre 1 y 100.
  *
  * readTEMP() para obtener el valor del sensor NTC, configuramos canal, leemos
  * valor y a continuación utilizamos la fórmula para obtener la temperatura en
  * Celsius, yo también ademas he practicado dos conversiones adicionales para
  * obtener la temperatura entre el intervalo 25 a 30 y depués obtenerla como
  * un porcentaje.
  *
  * setLeds() función para encender el número de leds correspondiente al valor del
  * sensor correspondiente. Necesita de un valor de porcentaje entre 0 y 100.
  *
  * setAlarm() función que dado un valor de procentaje entre 0 y 100 establece cual
  * es el led correspondiente en ese intervalo y lo enciende y apaga haciendo uso de
  * 2 contadores, con esta función mostramos el nivel de la alarma que se esta marcando
  * con el potenciometro
  *
  * fireAlarm() función que dado el estado de la alarma, esto es 1 sí se debería lanzar y
  * 0 si no debería sonar, hace sonar el Buzzer. Además escucha eventos del botón 2 para
  * detener la alarma en caso de que sea pulsado. Cuenta también con un sistema de cooldown
  * que haciendo uso de 2 contadores espera unos 10s para rearmar la alarma y que se pueda
  * volver a escuchar
  *
  * por último tenemos la función main que dentro del bucle while se ejecuta en cada
  * iteración, al principio de cada una de ellas leemos el valor de los sensores, haciendo
  * uso de las funciones readPOT(), readlUZ() y readTEMP(). Después leemos el botón 1
  * para saber en que modo estamos, si en el de LUZ o en el de temperatura, para
  * mostrar la información acorde y disparar la alarma con el valor correspondiente.
  *
  * Para saber el modo en el que estamos cada vez que se pulsa el botón 1 incrementamos
  * un contador y posteriormente miramos si este pulsador es par o impar ya que solo
  * hay dos modos. Lo primero que hacemos tras saber en que modo estamso es colocar
  * el valor leido del sensor en los leds y depués comparar si este valor es mayor al
  * valor del nivelde la alarma en caso de ser así, ponemos el valor de alarma_state a 1
  * y sinó pues a 0. Finalmente llamamos siemore a la función setAlarm() y fireAlarm()
  *
  ******************************************************************************
  */
