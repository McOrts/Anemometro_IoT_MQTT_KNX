// Arxiu Credentials

#pragma once

// ATENCIÓ!!! Només s'ha de definir un d'aquests paràmetres alhora, comentar el que NO s'utilitzi
//#define USE_ABP
#define USE_OTAA                           // OTAA no funciona amb ATM

#ifdef USE_ABP

// LoRaWAN NwkSKey (network session key) (ATENCIÓ!!! en format "msb")
static const u1_t PROGMEM NWKSKEY[16] = { 0x83, 0xB6, 0x11, 0xAC, 0x0D, 0x05, 0x7F, 0x4E, 0x58, 0x54, 0xA9, 0x0D, 0x7B, 0xEE, 0x69, 0xD8 };

// LoRaWAN AppSKey (application session key) (ATENCIÓ!!! en format "msb")
static const u1_t PROGMEM APPSKEY[16] = { 0xC1, 0x0B, 0xD7, 0xEC, 0x82, 0x1E, 0xEF, 0x2D, 0xAF, 0x50, 0x3E, 0x7E, 0x9D, 0x35, 0x7C, 0xCB };

// Adreça LoRaWAN end-device (DevAddr) (ATENCIÓ!!! en format hexadecimal)
// Aquesta ha de ser única per a cada node
static const u4_t DEVADDR             = 0x260B20F3;

#endif

#ifdef USE_OTAA

// Aquesta EUI ha d'estar en format "little-endian" (lsb), de manera que el byte menys significatiu
// es posi primer. Quan copiem una EUI de la sortida de ttnctl, això significa revertir
// els bytes. Per a les EUI emeses amb TTN, els últims bytes haurien de ser 0x00, 0x00,
// 0x00.
static const u1_t PROGMEM APPEUI[8]  = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };        // (ATENCIÓ!!! en format "lsb")

// També hauria d'estar en format "little endian" (lsb), vegeu més amunt.
static const u1_t PROGMEM DEVEUI[8]  = { 0x24, 0x5F, 0x06, 0xD0, 0x7E, 0xD5, 0xB3, 0x70 };        // (ATENCIÓ!!! en format "lsb")

// Aquesta "key" hauria d'estar en format "big endian" (msb) (o, ja que no és realment a
// nombre però un bloc de memòria, l'endianness no s'aplica realment). En la
// pràctica, una "key" presa de ttnctl es pot copiar tal com està.
// La "key" que es mostra aquí és la clau per defecte de semtech.
static const u1_t PROGMEM APPKEY[16] = { 0xDE, 0xA4, 0x90, 0x47, 0x36, 0x82, 0x6E, 0xB9, 0x72, 0xBA, 0x68, 0xE4, 0xA3, 0x50, 0x9C, 0x96 };          // (ATENCIÓ!!! en format "msb")

#endif
