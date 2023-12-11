/*****************************************************
 * Date: abril 2022
 * Written by: McOrts
 * 
 * ***************************************************/
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


