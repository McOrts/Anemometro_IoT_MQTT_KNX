/*****************************************************
 * Date: noviembre 2023
 * Written by: McOrts
 * 
 * ***************************************************/
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include "settings.h"

unsigned long int temp1;

/* Configuración lectura sensor */
unsigned int PWM;
float WSPE;
float TEMP;
unsigned int CRC;
String trama;

/* Configuración cálculos */
unsigned int CountRead;
float temperature;
float wind;
float wind_avg;
float wind_peak;
float wind_min;
float wind_sum;

/* Configuración cliente WiFi */
WiFiClient espClient;
String IP = String(15);

/* Configuración MQTT */
PubSubClient clientMqtt(espClient);
char msg[50];
String mqttcommand = String(14);

void setup() {
  Serial.begin(115200);

  // Inicializa el LED de la placa
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(500);

  /* Iniciar wifi */
  setup_wifi();
  clientMqtt.setServer(mqtt_server, mqtt_port);
  clientMqtt.setCallback(callback);

  /* Inicializa variables cálculos */
  CountRead = 0;
  wind = 0.0;
  wind_avg = 0.0;
  wind_peak = 0.0;
  wind_min = 0.0;
  wind_sum = 0.0;
}

String IpAddress2String(const IPAddress& ipAddress)
{
  return String(ipAddress[0]) + String(".") +\
  String(ipAddress[1]) + String(".") +\
  String(ipAddress[2]) + String(".") +\
  String(ipAddress[3]); 
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

  IP = IpAddress2String(WiFi.localIP());
  Serial.println("");
  Serial.println("[WIFI]WiFi conectada");
  Serial.print("[WIFI]IP: ");
  Serial.print(IP);
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
    if (clientMqtt.connect(mqtt_id, mqttUser, mqttPassword )) {
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

void clearSerialBuffer() {
  while (Serial.available() > 0) {
    Serial.read(); // Read and discard the incoming byte
  }
}

void loop() {
  
   if (!clientMqtt.connected()) {
    reconnect();
  }
  clientMqtt.loop();
  
  if (Serial.available() > 0) {
    trama = Serial.readStringUntil('\n'); // Lee la trama hasta encontrar un salto de línea
    //Serial.println(trama);

    // Divide la trama en sus componentes separados por espacios
    int valores[4];
    int contador = 0;
    char *ptr = strtok((char *)trama.c_str(), " ");
    while (ptr != NULL && contador < 4) {
      valores[contador] = atoi(ptr);
      ptr = strtok(NULL, " ");
      contador++;
    }

    // Asegúrate de que se hayan encontrado los 4 valores esperados
    if (contador == 4) {
      PWM = valores[0];
      WSPE = valores[1];
      TEMP = valores[2];
      CRC = valores[3];

      wind = WSPE / 10;
      temperature = TEMP / 100;

      // Cáculos estadísticos
      wind_sum += wind;
      CountRead ++;

      if (wind > wind_peak) {
        wind_peak = wind;
        Serial.print("wind peak: ");
        Serial.println(wind_peak);
       }
      if (wind < wind_min) {
        wind_min = wind;
        Serial.print("wind min: ");
        Serial.println(wind_min);
      }
      
    } else {
      Serial.println("Error: Trama de datos incorrecta");
      clientMqtt.publish(mqtt_sub_topic_healthcheck, "Frame error");
      clearSerialBuffer();
    }
  }

  if (millis()-temp1>update_time_sensors) {
    temp1=millis();
      
    wind_avg = wind_sum / CountRead;

    // Lecturas
    Serial.print("Measurements = ");
    Serial.println(CountRead);
    Serial.print("Speed = ");
    Serial.println(wind_avg);
    Serial.print("Speed peak = ");
    Serial.println(wind_peak);
    Serial.print("Speed minimun = ");
    Serial.println(wind_min);
    Serial.print("TEMP = ");
    Serial.println(temperature);

    // Envía la lectura por MQTT
    Serial.println("[MQTT] Sending data... ");
    snprintf (msg, 10, "%.2f", wind_avg);
    clientMqtt.publish(mqtt_pub_topic_windspeed, msg);  
    snprintf (msg, 10, "%.2f", wind_peak);
    clientMqtt.publish(mqtt_pub_topic_windspeedpeak, msg);  
    snprintf (msg, 10, "%.2f", wind_min);
    clientMqtt.publish(mqtt_pub_topic_windspeedmin, msg);  
    snprintf (msg, 10, "%.2f", temperature);
    clientMqtt.publish(mqtt_pub_topic_windtemperature, msg);  
    clientMqtt.publish(mqtt_sub_topic_ip, IP.c_str());
    clientMqtt.publish(mqtt_sub_topic_cp, "07014");
    delay (1000);
    if (WSPE==0) {
      digitalWrite(LED_BUILTIN, HIGH);       
    } else {
      digitalWrite(LED_BUILTIN, LOW);
    }

    wind_peak = 0;
    wind_min = 1000;
    wind_sum = 0;
    CountRead = 0;

  }

}

