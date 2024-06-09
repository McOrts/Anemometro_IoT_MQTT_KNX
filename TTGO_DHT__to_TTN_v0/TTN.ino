/*

  Mòdul TTN per utilitzar amb la biblioteca LMIC

  Aquest codi requereix la biblioteca LMIC de Matthijs Kooijman
  https://github.com/matthijskooijman/arduino-lmic

*/

#include <hal/hal.h>
#include <SPI.h>
#include <vector>
#include "config.h"
#include "credentials.h"

#ifndef CFG_eu868
// IMPORTANT!!! A Europa solament podem interectuar amb una freqüència "EU868", editem el fitxer "lmic_project_config.h"
// de la llibreria per canviar la configuració en cas que estigui configurat diferent
#error "Aquest script està pensat per connectar-se a la xarxa TTN EU a 868MHz"
#endif

// -----------------------------------------------------------------------------
// Globals
// -----------------------------------------------------------------------------

// Configuració LMIC GPIO
const lmic_pinmap lmic_pins = {
  .nss = NSS_GPIO,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = RESET_GPIO,
  .dio = {DIO0_GPIO, DIO1_GPIO, DIO2_GPIO},
};

#ifdef USE_ABP
// Aquestes "callbacks" només s'utilitzen en l'activació a l'aire (OTAA), de manera que ho són
// deixat buit aquí (no podem deixar-los fora completament tret que
// DISABLE_JOIN està establert a config.h, en cas contrari l'enllaçador es queixarà).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }
#endif

#ifdef USE_OTAA
void os_getArtEui (u1_t* buf) {
  memcpy_P(buf, APPEUI, 8);
}
void os_getDevEui (u1_t* buf) {
  memcpy_P(buf, DEVEUI, 8);
}
void os_getDevKey (u1_t* buf) {
  memcpy_P(buf, APPKEY, 16);
}
#endif

std::vector<void(*)(uint8_t message)> _lmic_callbacks;

// -----------------------------------------------------------------------------
// Mètodes privats
// -----------------------------------------------------------------------------

void _ttn_callback(uint8_t message) {
  for (uint8_t i = 0; i < _lmic_callbacks.size(); i++) {
    (_lmic_callbacks[i])(message);
  }
}

// La biblioteca LMIC cridarà a aquest mètode quan es desencadeni un esdeveniment
void onEvent(ev_t event) {

  if (EV_TXCOMPLETE == event) {

    if (LMIC.txrxFlags & TXRX_ACK) {
      _ttn_callback(EV_ACK);
    }

    if (LMIC.dataLen) {
      _ttn_callback(EV_RESPONSE);
    }

  }

  // Envia devolucions de callbacks
  _ttn_callback(event);

}

// -----------------------------------------------------------------------------
// Mètodes públics
// -----------------------------------------------------------------------------

void ttn_register(void (*callback)(uint8_t message)) {
  _lmic_callbacks.push_back(callback);
}

size_t ttn_response_len() {
  return LMIC.dataLen;
}

void ttn_response(uint8_t * buffer, size_t len) {
  for (uint8_t i = 0; i < LMIC.dataLen; i++) {
    buffer[i] = LMIC.frame[LMIC.dataBeg + i];
  }
}

bool ttn_setup() {

  // SPI interface
  SPI.begin(SCK_GPIO, MISO_GPIO, MOSI_GPIO, NSS_GPIO);

  // LMIC init
  //return ( 1 == os_init_ex( (const void *) &lmic_pins ) );
  os_init();
  return true;

}

void ttn_join() {

  // Restableix l'estat MAC. Es descartaran les transferències de dades de sessió i pendents.
  LMIC_reset();

#ifdef USE_ABP

  /// Estableix els paràmetres de sessió estàtics. En lloc d'establir una sessió dinàmicament
  // en unir-se a la xarxa, es proporcionen paràmetres de sessió precalculats.
  uint8_t appskey[sizeof(APPSKEY)];
  uint8_t nwkskey[sizeof(NWKSKEY)];
  memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
  memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
  LMIC_setSession(0x1, DEVADDR, nwkskey, appskey);

  // Configurem els canals utilitzats per The Things Network, que correspongui
  // als valors predeterminats de la majoria de passarel·les. Sense això, només tres base
  // s'utilitzen canals de l'especificació LoRaWAN, que sens dubte
  // funciona, de manera que és bo per depurar, però pot sobrecarregar-los
  // freqüències, així que assegureu-vos de configurar el rang de freqüències complet de
  // la vostra xarxa aquí (tret que la vostra xarxa les configure automàticament).
  // La configuració dels canals hauria de passar després de LMIC_setSession, com això
  // configura el conjunt mínim de canals.
  LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
  LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band

  // Si utilitzem una gateway monocanal, desactiveu tots els canals
  // excepte el que està escoltant la gateway
  //LMIC_disableChannel(0);
  //LMIC_disableChannel(1);
  //LMIC_disableChannel(2);
  //LMIC_disableChannel(3);
  //LMIC_disableChannel(4);
  //LMIC_disableChannel(5);
  //LMIC_disableChannel(6);
  //LMIC_disableChannel(7);
  //LMIC_disableChannel(8);

  // TTN defineix un canal addicional a 869,525 Mhz utilitzant SF9 per a la classe B
  // ranures de ping dels dispositius. LMIC no té una manera fàcil de definir-ho
  // la freqüència i el suport per a la classe B són irregulars i sense provar, així que això
  // la freqüència no està configurada aquí.
  
  // Desactiva la validació de la comprovació d'enllaços
  LMIC_setLinkCheckMode(0);

  // TTN utilitza SF9 per a la seva finestra RX2.
  LMIC.dn2Dr = DR_SF9;

 // Estableix la velocitat de dades i la potència de transmissió per a l'enllaç ascendent (nota: txpow sembla que la biblioteca ignora)
  LMIC_setDrTxpow(DR_SF7, 14);

  // Activar una connexió falsa
  _ttn_callback(EV_JOINED);

#endif // TTN_ACTIVATION == ACTIVATION_ABP

}

void ttn_sf(unsigned char sf) {
  LMIC_setDrTxpow(sf, 14);
}

void ttn_adr(bool enabled) {
  LMIC_setAdrMode(enabled);
}

void ttn_send(uint8_t * data, size_t len, uint8_t port, bool confirmed) {

  // Comprovem si no s'està executant cap treball TX/RX actual
  if (LMIC.opmode & OP_TXRXPEND) {
    _ttn_callback(EV_PENDING);
    return;
  }

  // Preparem la transmissió de dades amunt en el proper moment possible.
  // Els paràmetres són port, dades, longitud, confirmat
  LMIC_setTxData2(port, data, len, confirmed ? 1 : 0);

  _ttn_callback(EV_QUEUED);
}

void ttn_loop() {
  os_runloop_once();
}
