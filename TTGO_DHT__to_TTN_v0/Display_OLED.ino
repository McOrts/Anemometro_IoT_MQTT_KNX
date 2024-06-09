// -------------------------------------------------------------------------------------------------------------------------------------
// Mòdul Display OLED (I2C)
//--------------------------------------------------------------------------------------------------------------------------------------

// Importem la llibreria que ens permetrà comunicar-nos per I2C
#include <Wire.h>

// Importem la llibreria que ens permetrà gestionar el display OLED
#include "SSD1306.h"

// Cridem l'arxiu que conté dades necessaries per fer funcionar el display OLED
#include "config.h"

// Creem un objecte de la clase SSD1306 i li passem els arguments necessaris (adreça I2C, pin SDA i pin SLC)
SSD1306 display(0x3C, pinSDA, pinSCL);

// Funció que ens configura els display OLED
void _screenReset() {
    
    // Configurem el pin de RESET com una "sortida"
    pinMode(pinRESET_I2C, OUTPUT);
    
    // Apaguem el display
    digitalWrite(pinRESET_I2C, LOW);
    
    delay(50);
    
    // Encenem el display
    digitalWrite(pinRESET_I2C, HIGH);
}

// Funció que ens neteja la pantalal del display OLED
void screenClear() {
    display.clear();
}

// Funció que ens permet mostrar missatges per la pantalla del display OLED
void screenPrint(const char * str) {
    display.clear();
    display.print(str);
    display.drawLogBuffer(0, 0);
    display.display();
    DEBUG_MSG(str);
}

// Funció que ens inicialitza el display OLED
void screenSetup() {

    _screenReset();

    display.init();
    display.flipScreenVertically();
    display.setFont(ArialMT_Plain_10);
    display.setLogBuffer(5, 30);
}
