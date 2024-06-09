// ******************************************************************************************************************************************************************************************************************
// HACKATÓ PARK IoT - 16 de Març del 2.024
// FAB Casa del Mig (Sants / Barcelona)
// v0 (16/02/2.024)
// Maker: David Pérez
// ******************************************************************************************************************************************************************************************************************
//
// Tipus de dispositiu: Sensor de temperatura i humitat ambient
// Model: DHT22 (https://es.aliexpress.com/i/32883872442.html)

// Tensió alimentació (VCC): 3V3 a 5V DC
// Pins: #1 - VCC
//       #2 - Senyal
//       #3 - <buit>
//       #4 - GND 
//
// Datasheet: https://cdn-shop.adafruit.com/datasheets/DHT22.pdf
//
// Descripció del Codi:
// El següent codi mesura la temperatura i la humitat ambient amb el sensor DHT, les mostra pel Monitor Serial del IDE d'Arduino i les envia a la plataforma TTN (The Things Network) mitjantçant comunicació LoRa a 868 MHz.
//
// Placa: TTGO LoRa32 OLED 0,96" v1.0 (https://www.aliexpress.com/item/2pcs-of-TTGO-LORA32-868-915Mhz-SX1276-ESP32-Oled-display-Bluetooth-WIFI-Lora-development-board/32841743946.html)
//        TTGO LoRa32 OLED 0,96" v1.3 (https://es.aliexpress.com/item/4000628100802.html)
// Afegir la següent URL al gestor de plaques de la pestanya "Fitxer/Preferències": https://dl.espressif.com/dl/package_esp32_index.json del IDE de Arduino
// Descarregar el paquet "esp32 by Espressif Systems" ATENCIÓ IMPORTANT!!! la versió 1.0.6 al Gestor de plaques de la pestanya "Eines/Placa/Gestor de plaques" del IDE de Arduino
// Pinout: https://i0.wp.com/randomnerdtutorials.com/wp-content/uploads/2019/10/TTGO-LoRa-Board-Pinout.jpg?w=936&quality=100&strip=all&ssl=1
// ATENCIÓ! Les versions d'aquesta placa diferents de la 1.x tenen errors interns de fabricació que es solucionen fent ponts en alguns pins. Buscar la solució a la xarxa.
//
// Connexions: 1) 5V (placa) - Vcc (breadboard)
//             2) Vcc (breadboard) - Pin #1 (DHT)
//             3) Pin #23 (placa) - Pin #2 (DHT)  
//             4) Resistència 4K7 entre Vcc(breadboard) i Pin #2 (DHT)
//             5) GND (placa) - Pin #4 (DHT) 
//
// Aquest codi requereix la biblioteca LMIC de Matthijs Kooijman (https://github.com/matthijskooijman/arduino-lmic)
// ******************************************************************************************************************************************************************************************************************


// Importem la llibreria que gestionarà el funcionament del sensor DHT
// #include <DHT.h>

// Importem la llibreria que ens permetrà comunicar-nos per I2C
#include <Wire.h>

// Cridem l'arxiu que conté dades necessaries per fer funcionar el codi principal
#include "config.h"

// Cridem l'arxiu necessari per guardar dades a la memòria de la placa quan s'utilitza el rellotge RTC
#include <rom/rtc.h>

// Declarem una variable que guarda el comptador de missatges que s'envien
// El comptador de missatges, emmagatzemat a la memòria RTC, es manté encara que utilitzem la funció "deep sleep"
RTC_DATA_ATTR uint32_t comptador = 0;


// -----------------------------------------------------------------------------------------------------------------------------------------------------------
// Funcions
// -----------------------------------------------------------------------------------------------------------------------------------------------------------

/* La funció “msb()” (big endian) transforma la variable float en un integer, per tant passem de 4 bytes a 2 bytes
   i després corre 8 bits el primer byte cap a la dreta per quedar-se solament amb aquest primer byte
   i fer desaparèixer el segon byte.
   Finalment es carrega el primer byte amb la instrucció “& 0xFF” per quedar-se solament amb un únic byte
*/
unsigned char msb(float value) {
  return (int(value) >> 8) & 0xFF;
}

/* La funció “lsb()” (little endian) transforma la variable float en un integer, per tant passem de 4 bytes a 2 bytes.
   Finalment es carrega el primer byte amb la instrucció “& 0xFF” per quedar-se solament amb el byte de la dreta.
*/
unsigned char lsb(float value) {
  return int(value) & 0xFF;
}


// Funció que envia el missatge a la plataforma TTN
void enviarMsg() {

  // Cridem les funcions que ens mesuren la temperatura i la humitat ambient i guardem el valor en una variable
  float temperatura = 22;
  float humitat = 33;

  // Comprovem si hi ha hagut algun error en la lectura (isnan = is not a number)
  if (isnan(temperatura) || isnan(humitat)) {
    Serial.println("Error obtenint les dades del sensor DHT22");
    return;
  }

  // https://www.quora.com/What-does-5-2f-means-in-C
  // %5.2fºC\n =
  // %: indica que comença a mostrar el valor que volem
  // 5: nº d'espais que ocuparà el valor a mostrar (ex. 23,05 = 5 espais)
  // 2: nº de decimals a mostrar
  // f: tipus de valor a mostrar (f: float, d: decimal)
  // ºC: text que es mostrarà després del valor mostrat
  // \n: salt de linia
  // NOTA: Perque surti el caracter "%" després del valor a mostrar hem d'escriure "%%"

  // Mostrem pel Monitor Serial el comptador del paquet que enviarem i les mesures realitzades de temperatura, humitat ambient, pressió baromètica i alçada

  // ATENCIÓ!!! La funció "printf" NO és compatible amb una placa d'ARDUINO, solament amb plaques ESP32,
  // S'utilitza per simplificar en una sola linia de codi el text i el valor que volem mostrar.
  // Si es vol utilitzar amb plaques Arduino s'ha de baixar la llibreria adequada.

  Serial.println("");
  Serial.printf("[SNS] Comptador: %d\n", comptador);
  Serial.printf("[SNS] Temperatura: %4.1fºC\n", temperatura);
  Serial.printf("[SNS] Humitat: %d%%\n", (int) humitat);
  Serial.println("");

  // Declarem i inicialitzem una variable que enmagatzemarà la longitud màxima que tindrà la cadena de caracters del missatge a enviar.
  // IMPORTANT! S'ha de declarar com una constant per poder utilizar-la en un array
  const int Tamany_Buffer_Msg = 64;

  // Declarem i inicialitzem la variable array de tipus "char" que enmagatzemarà la càrrega útil (payload) dels missatges a enviar al display OLED
  char buffer[Tamany_Buffer_Msg];

  // Enpaquetem dins la variable el missatge que enviarem al display OLED
  snprintf(buffer, sizeof(buffer), "[TTN] Enviant msg #%03d\n", (byte) (comptador & 0xFF));

  // Mostrem pel display OLED el comptador del missatge que estem enviant
  screenPrint(buffer);

  /*
    Les dades que enviarem a la plataforma TTN les codificarem d'aquesta manera:

    Hem de tenir em compte que si enviem un número comprés entre 0 i 255 utilitzarem un sol byte però que si el número està comprés entre 256 i 65.535 aleshores
    necessitarem 2 bytes, i a més a més aquests valors han de ser sempre positius, mai negatius.
    Si tenim un número amb decimals, aleshores hem de multiplicar per 100 per eliminar els decimals i convertir el valor en un número enter.
    Exemple: 23,58 -> 23,58 * 100 = 2358 > 255 -> 2 bytes

    Si volem enviar un número negatiu amb decimals hem de sumar-li un valor suficient per tal que quedi positiu i després multiplicar per 100 com s'ha explicat abans
    Exemple: -62,98 -> (-62,98 + 100) * 100 = 3702 > 255 -> 2 bytes
            -58 -> -58 + 100 = 42 < 255 -> 1 byte

    comptador (1 byte)
    MSB (temperatura + 40)*100 (1 byte)       (*)
    LSB (temperatura + 40)*100 (1 byte)       (*)
    humitat (1 byte)

    (*) El valor 40 és perque segons el datasheet del sensor de temperatura DHT (https://cdn-shop.adafruit.com/datasheets/DHT22.pdf)
    el rang de temperatures que pot mesurar està comprés entre -40 i 125ºC.
    Si sumem aquest valor a sigui quina sigui la mesura evitem que surti un valor negatiu

    La funció descodificadora dels missatges "Uplink" que hem enviat a la plataforma TTN s'ha d'anomenar SEMPRE "Decoder"
    i ha d'estar escrita en llenguatge Javascript!!!):

    function Decoder(bytes, port) {
    var descodificar = {};
    if (port == 20) {
     descodificar.comptador = bytes[0];
     descodificar.temperatura = (bytes[1] * 256 + bytes[2]) / 100.0 - 40.0;
     descodificar.humitat = bytes[3];
    }
    return descodificar;
    }
  */

  // Reescalem les mesures preses pel sensor per encapsular-les dins d'una variable del tipus array
  temperatura = (temperatura + 40.0) * 100;

  // Declarem una variable que guardarà el nombre d'elements (bytes) que volem enviar a la plataforma de TTN. IMPORTANT! S'ha de declarar com una constant
  // per poder utilizar-la com una variable del tipus array
  const byte numElements = 4;

  // Declarem una variable del tipus array on guardarem el comptador i totes les dades mesurades pel sensor DHT
  byte data[numElements] = {
    (byte) (comptador & 0xFF),
    msb(temperatura), lsb(temperatura),
    lsb(humitat),
  };

  // Enviem el paquet de dades a la plataforma TTN
#if LORAWAN_CONFIRMED_EVERY
  bool confirmed = (count % LORAWAN_CONFIRMED_EVERY == 0);
#else
  bool confirmed = false;
#endif
  ttn_send(data, sizeof(data), port_LORAWAN, confirmed);

  // Incrementem en una unitat el valor de comptador de missatges enviats
  comptador++;
}

// Funció que gestiona la connexió del microcontrolador amb la plataforma TTN
void callback(byte message) {

  if (EV_JOINING == message) screenPrint("[TTN] Connectant-se...\n");
  if (EV_JOINED == message) screenPrint("[TTN] Connectat!\n");
  if (EV_JOIN_FAILED == message) screenPrint("[TTN] Connexió fallida\n");
  if (EV_REJOIN_FAILED == message) screenPrint("[TTN] Reconnexió fallida\n");
  if (EV_RESET == message) screenPrint("[TTN] Reset\n");
  if (EV_LINK_DEAD == message) screenPrint("[TTN] Enllaç mort\n");
  if (EV_ACK == message) screenPrint("[TTN] ACK rebut\n");
  if (EV_PENDING == message) screenPrint("[TTN] Missatge descartat\n");
  //if (EV_QUEUED == message) screenPrint("[TTN] Missatge en cua\n");

  if (EV_TXCOMPLETE == message) {
    screenPrint("[TTN] Missatge enviat\n");
  }

  if (EV_RESPONSE == message) {

    screenPrint("[TTN] Resposta: ");

    size_t len = ttn_response_len();
    uint8_t data[len];
    ttn_response(data, len);

    char buffer[6];
    for (byte i = 0; i < len; i++) {
      snprintf(buffer, sizeof(buffer), "0x%02X ", data[i]);
      screenPrint(buffer);
    }
    screenPrint("\n");
  }
}


void setup() {
  // Inicialització del Monitor Serial
#ifdef DEBUG_PORT
  DEBUG_PORT.begin(velocitatTransmissio);
#endif

  // Configurem el pin de dades on tenim connectat el sensor DHT com una "entrada"
//  pinMode(pinDHT, INPUT);

  // Cridem la funció que ens gestionarà el funcionament del display OLED
  screenSetup();

  // Mostrem un missatge de presentació per la pantalla del display OLED
//  screenPrint("SENSOR DHT\n");

  // Test I2C
  //i2cSetup();                             // El comentem perque ja inicialitzem la comunicació I2C quan també inicialitzem el sensor DHT
  //i2cScan();

  // Cridem la funció que ens gestionarà funcionament del sensor DHT
 // sensorSetup();

  // Esperem un temps per poder veure el missatge de benvinguda al display OLED de la placa
  delay(2000);

  // Cridem la funció que ens gestionarà el mòdul LoRa de la placa
  if (!ttn_setup()) {
    screenPrint("No s'ha pogut trobar el mòdul de ràdio!\n");
    while (true);
  }

  // Cridem les funcions que ens permetran connectar-nos a la plataforma TTN
  ttn_register(callback);
  ttn_join();
  ttn_sf(LORAWAN_SF);
  ttn_adr(LORAWAN_ADR);
}


void loop() {
  // Enviem el missatge cada cert interval de temps
  static long ultimMsg = 0;
  if (0 == ultimMsg || millis() - ultimMsg > tempsEnviamentMsg) {

    // Cridem la funció que envia els missatges a la plataforma TTN
    enviarMsg();

    // Resetegem el temps a 0
    ultimMsg = millis();
  }

  // IMPORTANT!!! Cridar aquesta funció SEMPRE que volguem enviar dades a la plataforma TTN a la funció "loop" del codi principal
  ttn_loop();
}
