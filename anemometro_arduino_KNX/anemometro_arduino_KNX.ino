/*
  Sensor de alarma gateway KNX

Estado incial: leds apagados y no lee el sensor.
Cuando se pulsa el botón  
(1) Se empieza a leer el sensor cada 2 segundos
  * Si el valor < 800
    * Se envia "NOAL"
  * si el valor >=4 m/s
    * Se envía "WIND_AL" 
    * Se conmuta el LED del verde al rojo
    * Deja de leer el sensor
    * Se espera a recibir un "CLOSED_S" que indica que la persiana está cerrada
    * Se apaga el led rojo
    * Se espera a recibir un "UNFROZEN_AL" indica desbloqueo del usuario desde la botonera KNX
    * Se espera un segundo y
       * Se envia un "UNFR"
       * Se reestablece el estado al punto (1)
*/
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

#define sensor A0 
#define boton 2 
#define ledrojo 7
#define ledverde 12

const int sensorvalue = 4;

bool leyendo;
unsigned long int temps1;
String incomingString;

//Creamos el objeto lcd con la dirección 0x3F, 16 columnas y 2 filas
LiquidCrystal_I2C lcd(0x3F,16,2);  

void setup() {
  Serial.begin(9600);
  pinMode (sensor, INPUT);
  pinMode (boton, INPUT);
  pinMode (ledrojo, OUTPUT);
  pinMode (ledverde, OUTPUT);
  temps1=millis();
  leyendo = false;
  // Inicializamos la pantalla LCD
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("<-");
  lcd.setCursor(0, 1);
  lcd.print("->          m/s");
}

void analizar_sensor(){
  if (millis()-temps1>2000) {
    float outvoltage = analogRead(sensor) * (5.0 / 1023.0);
    int Level = 6*outvoltage;
    lcd.setCursor(13, 0);
    lcd.print(Level);
    if (Level<sensorvalue) {
      Serial.println("NOAL");
      lcd.setCursor(3, 0);
      lcd.print("NOAL    ");
      temps1=millis();
    } else {
      Serial.println("WIND_AL");
      lcd.setCursor(3, 0);
      lcd.print("WIND_AL ");
      digitalWrite(ledverde, LOW);
      digitalWrite(ledrojo, HIGH);
      leyendo = false;
    }
  }
}

void lee_serial(){
  while(Serial.available()) {
//  if (Serial.read()!=-1) {
      incomingString = Serial.readStringUntil('\n');// read the incoming data as string
      if (incomingString=="CLOSED_S") {
        lcd.setCursor(3, 1);
        lcd.print("CLOSED_S ");
        digitalWrite(ledrojo, LOW);
      } else if (incomingString=="UNFR_AL") {
        lcd.setCursor(3, 1);
        lcd.print("UNFR_AL  ");
        delay (1000);
        Serial.println("UNFR");
        lcd.setCursor(3, 0);
        lcd.print("UNFR      ");
        leyendo = true;        
        digitalWrite(ledverde, HIGH);
      }
  }
}

void loop() {
  if (digitalRead(boton)==HIGH) {
    delay (250);
    leyendo = true;
    digitalWrite(ledverde, HIGH);
    digitalWrite(ledrojo, LOW);
  }
  if (leyendo) {
    analizar_sensor();
  }
   
  lee_serial();
}
