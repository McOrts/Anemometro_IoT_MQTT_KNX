// ------------------------------------------------------------------------------------------------------------------
// ARXIU DE CONFIGURACIÓ
// ------------------------------------------------------------------------------------------------------------------

#pragma once

#include <Arduino.h>
#include <lmic.h>

// Configurar "credentials.h" amb la configuració del nostre node
#include "credentials.h"

void ttn_register(void (*callback)(uint8_t message));

// -----------------------------------------------------------------------------
// Configuració
// -----------------------------------------------------------------------------

#define DEBUG_PORT              Serial
int velocitatTransmissio = 115200;          // Velocitat de transmissió del Monitor Serial (en bauds/seg)
int tempsEnviamentMsg = 60000;              // Envia missatges cada interval de temps (en millis)
byte port_LORAWAN = 20;                     // Port de la plataforma TTN on s'enviaran els missatges
byte LORAWAN_CONFIRMED_EVERY = 0;           // Envia un missatge de confirmat a tots aquests missatges (0 significa mai)
unsigned char LORAWAN_SF = DR_SF7;          // Spreading factor (Factor de propagació)
byte LORAWAN_ADR = 0;                       // Activa l'ADR

// -----------------------------------------------------------------------------
// DEBUG
// -----------------------------------------------------------------------------

#ifdef DEBUG_PORT
#define DEBUG_MSG(...) DEBUG_PORT.printf( __VA_ARGS__ )
#else
#define DEBUG_MSG(...)
#endif

// -----------------------------------------------------------------------------
// Missatges personalitzats
// -----------------------------------------------------------------------------

byte EV_QUEUED = 100;
byte EV_PENDING = 101;
byte EV_ACK = 102;
byte EV_RESPONSE = 103;

// -----------------------------------------------------------------------------
// Hardware
// https://www.thethingsnetwork.org/forum/t/big-esp32-sx127x-topic-part-2/11973
// -----------------------------------------------------------------------------

// Pins connexions SPI per connectar amb el mòdul LoRa (SPI)
const byte SCK_GPIO = 5;
const byte MISO_GPIO = 19;
const byte MOSI_GPIO = 27;
const byte NSS_GPIO = 18;
const byte RESET_GPIO = 14;
const byte DIO0_GPIO = 26;
const byte DIO1_GPIO = 33;
const byte DIO2_GPIO = 32;


// ------------------------------------------------------------------------------------------------------------------
// Display OLED (I2C)
// ------------------------------------------------------------------------------------------------------------------

const byte pinSDA = 4;
const byte pinSCL = 15;
const byte pinRESET_I2C = 16;


// -----------------------------------------------------------------------------
// LEDs
// -----------------------------------------------------------------------------
const byte pinLed = 2;

// ------------------------------------------------------------------------------------------------------------------
// Sensor DHT  
// ------------------------------------------------------------------------------------------------------------------

// Declarem una variable on guardarem el pin on es connectarà el sensor DHT22
// const byte pinDHT = 23;

// Especifiquem el tipus de sensor (DHT11 o DHT22)
// #define DHTTYPE DHT22

// Creem un objecte de la clase DHT
//DHT dht(pinDHT, DHTTYPE);
