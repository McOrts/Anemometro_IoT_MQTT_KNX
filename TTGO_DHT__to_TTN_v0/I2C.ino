// Mòdul I2C

// Cridem la llibreria que gestiona les connecions I2C
#include "Wire.h"

// Funció que comprova que l'adreça del dispositiu que tenim connectat a la placa és la correcta
bool i2cCheck(unsigned char address) {
    Wire.beginTransmission(address);
    return Wire.endTransmission();
}

// Funció que ens busca els dispositius connectats a la placa i que es comuniquen a través del protocol I2C
void i2cScan() {
    unsigned char nDevices = 0;
    for (unsigned char address = 1; address < 127; address++) {
        unsigned char error = i2cCheck(address);
        if (error == 0) {
            DEBUG_MSG("[I2C] Dispositiu 0x%02X\n", address);
            nDevices++;
        }
    }
    if (nDevices == 0) DEBUG_MSG("[I2C] No s'ha trobat cap dispositiu\n");
}

// Inicialitzem la connexió I2C
void i2cSetup() {
    Wire.begin();
}
