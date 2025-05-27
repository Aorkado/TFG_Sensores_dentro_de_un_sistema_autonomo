# Prueba de I2C en el entorno ESP-IDF

Prueba del funcionamiento del protocolo I2C en el entorno ESP-IDF. El código se basa en la
[documentación de espressif sobre I2C](https://docs.espressif.com/projects/esp-idf/en/v5.3.1/esp32/api-reference/peripherals/i2c.html).

## Objetivo

El objetivo de este repositorio es diseñar funciones que realicen, por un lado, la 
configuración de los buses y los dispositivos, y, por otro lado, el formateo de las 
transmisiones del protocolo I2C.

## Lectura

Para la lectura, se usa el sensor de temperatura LM75A como dispositivo de prueba. Todo su
funcionamiento se ilustra en el directorio `i2c_read_LM75A/`.

## Escritura

Aún no se ha desarrollado el soporte de escritura mediante I2C.
