# Lectura del dispositivo LM75A mediante I2C

La prueba realizada para hacer una lectura a través de I2C se hizo con el sensor de 
temperatura LM75A. El resultado principal de esta prueba es la implementeación de la función
`i2c_read()`. 

## Componentes

El circuito empleado necesita varios componentes, sin incluir los cables de conexión, que 
se muestran en la siguiente tabla. Aclarar que las resistencias de 10 kOhm son las 
resistencias de pull-up, vitales para la comunicación por I2C.

| Componentes        | Marca	 | Cantidad |
|--------------------|-----------|----------|
| ESP32              | Espressif | x1       |
| LM75A	             | NXP       | x1       |
| Resistencia 10kOhm | (-)       | x2       |

## LM75A

El sensor [LM75A](https://www.nxp.com/docs/en/data-sheet/LM75A.pdf) es un dispositivo con
sensor de temperatura con un conversor ADC y con puertos de comunicación I2C. Toda la 
información necesaria para usar el dispositivo se encuentra en la hoja técnica.

**Pinout**

El pinout es modificable. No sólo la alimentación se puede modificar, sino que también se
pueden alterar los valores de A2, A1 y A0. El propósito de estos tres pines es de fijar
la dirección I2C del dispositivo según se explica en la hoja técnica.

| Símbolo | Pin | Conexiones |
|---------|-----|------------|
| SDA	  | 1   | SDA        |
| SCL     | 2   | SCL        |
| OS	  | 3   | (-)        |
| GND     | 4   | GND        |
| A2      | 5   | 3.3 V      |
| A1      | 6   | GND        |
| A0      | 7   | GND        |
| VCC     | 8   | 3.3 V      |

**Dirección de LM75A** 

0b1001100 o 0x4C

**Nota:** La alimentación de 3.3 V la ofrece el Esp32.

## Implementación de la función de lectura

La función de lectura desarrollada es `i2c_read(uint16_t device_adress, uint8_t * data_rd)`.
Está aún esta en desarrollo, pues aún se pretende implementar más funcionalidades. 

### Funcionalidad actual

`i2c_read(uint16_t device_adress, uint8_t * data_rd)` se comporta de la siguiente manera:

Primero se define la configuración del bus que va a usar el master; despues la configuración
del dispositivo esclavo. A partir de estas, se inicializa el bus del master a partir de
`i2c_new_master_bus()` y luego se añade el dispositivo esclavo al bus del master
mediante el handle del bus y la función `i2c_master_bus_add_device`.

Por último, se reciben los datos del dispositivo con `i2c_master_receive()`, que se guardan en
un buffer. Si no se recibe nada en un tiempo determinado, cuarto argumento de la función, el
periodo de lectura se termina.

En cuanto a los argumentos de `i2c_read()`, se usan dos.

**`uint16_t device_address`:** Dirección I2C del dispositivo esclavo. Es necesario que el tipo
de la variable sea `uint16_t`, aunque la dirección use sólo 7 bits.

**`uint8_t * data_rd`:** Buffer de datos de lectura. El registro de temperatura guarda la
información en 16 bits, 11 de ellos útiles; aún así, es necesario que el tipo sea `uint8_t`, 
porque es el tipo de buffer que usa `i2c_master_receive`. Aparte, se usa un puntero porque
el buffer podría tener diferentes tamaños.

### Objetivos

Esta función no da mucha libertad de uso. Se necesita dar un periodo de lectura, en caso de
querer tener más información sin tener que configurar el dispositivo y el bus siempre que
se quiera leer más de un dato.

Otra funcionalidad que habría que implementar es la de eliminar la configuración y dejar 
libre el bus, pues, por ejemplo, el sensor LM75A necesita que el bus esté libre para operar
correctamente.
