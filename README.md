# Conectividad IoT para anemómetros
Cada dia es más habitual el uso de sensores de viento integrados con la lógica domótica de los edificios. Para ello es imprescindible considerar el anemómetro como un dispositivo IoT con capacidad de conexión en los diferentes protocolos de comunicación: MQTT, KNX, y medios de transmisión como Ethernet o WiFi.

## Versión Arduino WemosD1 MQTT por WiFi
Esta es la versión más simple y versátil. El sensor está montado con resistencia IP67 por lo que puede instalarse en el exterior e incluso utilizarse como dispositivo portátil.

<img src="img/anemometro_arduino_wifi_mqtt.png" width="500"  align="center" />

### Lista de materiales
- [Anemómetro de cazoletas SKU:SEN0170](https://www.dfrobot.com/search-SEN0170.html)
- [WEMOS D1 Mini Pro 4M](https://es.aliexpress.com/item/32801063577.html)
- [Adaptador de corriente 12V a micro-USB](https://es.aliexpress.com/item/32973455778.html)
- [Caja exterior resistencia IP66 tapa transparente](https://es.aliexpress.com/item/4000200361035.html)

### Montaje
Utilizando cualquier protoboard solo se requiere conectar la masa comùn del microcontrolador y el anemómetro y la senal de tensión de salida del mismo a la entrada analógica del Wemos (ESP8266)

<img src="img/anemometro_arduino_wifi_mqtt_bb.png" width="400"  align="center" />

### Software
Partimos de que se dispone de un broker MQTT tipo Mosquitto y un Node-RED para procesar la información. El firmware utiliza la libreria para WiFi (ESP8266WiFi.h) y la de MQTT (PubSubClient.h)

```cpp
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include "settings.h"

unsigned long int temp1;

/* Configuración sensor */
unsigned int WindSpeed ;              

/* Configuración cliente WiFi */
WiFiClient espClient;

/* Configuración MQTT */
PubSubClient clientMqtt(espClient);
char msg[50];
String mqttcommand = String(14);

void setup() {
  Serial.begin(9600);

  // Inicializa el LED de la placa
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(500);

  /* Iniciar wifi */
  setup_wifi();
  clientMqtt.setServer(mqtt_server, mqtt_port);
  clientMqtt.setCallback(callback);
}

void setup_wifi() {
  delay(10);

  // Comienza el proceso de conexión a la red WiFi
  Serial.println();
  Serial.print("[WIFI]Conectando a ");
  Serial.println(ssid);

  // Modo estación
  WiFi.mode(WIFI_STA);
  // Inicio WiFi
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("[WIFI]WiFi conectada");
  Serial.print("[WIFI]IP: ");
  Serial.print(WiFi.localIP());
  Serial.println("");
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("[MQTT]Mensaje recibido (");
  Serial.print(topic);
  Serial.print(") ");
  mqttcommand = "";
  for (int i = 0; i < length; i++) {
    mqttcommand += (char)payload[i];
  }
  Serial.print(mqttcommand);
  Serial.println();
  // Switch on the LED if an 1 was received as first character
  if (mqttcommand == "comando") {
    Serial.println("comando");
  }  
}

void reconnect() {
  Serial.print("[MQTT]Intentando conectar a servidor MQTT... ");
  // Bucle hasta conseguir conexión
  while (!clientMqtt.connected()) {
    Serial.print(".");
    // Intento de conexión
    if (clientMqtt.connect(mqtt_id)) { // Ojo, para más de un dispositivo cambiar el nombre para evitar conflicto
      Serial.println("");
      Serial.println("[MQTT]Conectado al servidor MQTT");
      // Once connected, publish an announcement...
      clientMqtt.publish(mqtt_sub_topic_healthcheck, "starting");
      // ... and subscribe
      clientMqtt.subscribe(mqtt_sub_topic_operation);
    } else {
      Serial.print("[MQTT]Error, rc=");
      Serial.print(clientMqtt.state());
      Serial.println("[MQTT]Intentando conexión en 5 segundos");

      delay(5000);
    }
  }
}

void loop() {
  if (!clientMqtt.connected()) {
    reconnect();
  }
  clientMqtt.loop();
    
  if (millis()-temp1>update_time_sensors) {
    temp1=millis();
    
    // Lectura del puerto analógico y traducción del voltaje a velocidad  
    WindSpeed = 6 * analogRead(A0) * (5.0 / 1023.0);

    Serial.print("Velocidad del viento: ");
    Serial.print(WindSpeed);
    Serial.println(" m/s");
      
    // Envía la lectura por MQTT
    snprintf (msg, 10, "%6i", WindSpeed);
    Serial.print("[MQTT] Sending data: ");
    Serial.println(msg);
    clientMqtt.publish(mqtt_pub_topic_voltage, msg);   
    delay (1000);
    if (WindSpeed==0) {
      digitalWrite(LED_BUILTIN, HIGH);       
    } else {
      digitalWrite(LED_BUILTIN, LOW);
    }
  }
}
```
Los parámetros de conexión y tiempos, están definidos en un setting.h:

```cpp
// WiFi Configuration
const char* ssid = "MiFibra-2D79-terraza";
const char* password = "";

// MQTT Configuration
const char* mqtt_server = "192.168.1.114";
const int mqtt_port = 1883;
const char* mqtt_id = "anemometer_cups";
const char* mqtt_sub_topic_healthcheck = "/home/meteo/anemometer/healthcheck";
const char* mqtt_pub_topic_voltage = "/home/meteo/anemometer/wind_speed";
const char* mqtt_sub_topic_operation = "/home/meteo/anemometer/operation";

// Other params
const int update_time_sensors = 59000;
```

### Back-end
El procesado y almacenamiento de la información se orquesta desde una aplicación Node-RED. El _flow_ está subscrito al _topic_ de velocidad de viento "/home/meteo/anemometer/wind_speed". A partir del mensaje con la velocidad de viento en m/s, se hace el cálculo a km/h para almacenar el dato en una BBDD MySQL, presentar la información en un _dashboard_ y validar el umbral de alerta para enviar un mensaje por el servicio IFTTT.

<img src="img/node-red_flow.png" width="500"  align="center" />

## Versión RaspberryPi MQTT por Ethernet
También se puede implementar un sensor de viento sobre una _single board computer_ como la Raspberry Pi y utilizando señal de pulsos como la que tiene anemómetros como el [WH-SP-WS01](https://es.aliexpress.com/item/1005001484228267.html)

<img src="img/anemometro_arduino_wifi_mqtt_bb.png" width="500"  align="center" />
anemometro_rpi_ethernet_mqtt

### Sofware
En este caso he empleado un programa Python que además de enviar el mensaje MQTT, almancena la lectura y notifica por mail si la velocidad del viento excede de un umbral.

## Versión Ardunio UNO KNX
Otra opción más compleja es la integrar el sensor de viento en una red KNX.


