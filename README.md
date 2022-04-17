# Conectividad IoT para anemómetros
Cada dia es más habitual el uso de sensores de viento integrados con la lógica domótica de los edificios. Para ello es imprescindible considerar el anemómetro como un dispositivo IoT con capacidad de conexión en los diferentes protocolos de comunicación: MQTT, KNX, y medios de transmisión como Ethernet o WiFi.

## Versión Arduino WemosD1 MQTT por WiFi
Esta es la versión más simple y versátil. El sensor está montado con resistencia IP67 por lo que puede instalarse en el exterior e incluso utilizarse como dispositivo portátil.

### Lista de materiales
- [Anemómetro de cazoletas SKU:SEN0170](https://www.dfrobot.com/search-SEN0170.html)

## Versión RaspberryPi MQTT por Ethernet

## Versión Ardunio UNO KNX

## Back-end
El procesado y almacenamiento de la información se orquesta desde una aplicación Node-RED. El _flow_ está subscrito al _topic_ de velocidad de viento "/home/meteo/anemometer/wind_speed". A partir del mensaje con la velocidad de viento en m/s, se hace el cálculo a km/h para almacenar el dato en una BBDD MySQL, presentar la información en un _dashboard_ y validar el umbral de alerta para enviar un mensaje por el servicio IFTTT.
