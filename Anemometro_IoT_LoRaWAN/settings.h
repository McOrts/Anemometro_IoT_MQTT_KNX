// LoRaWAN Configuration

/* OTAA para*/
static uint8_t devEui[] = { 0x };
static uint8_t appEui[] = { 0x };
static uint8_t appKey[] = { 0x };

uint16_t userChannelsMask[6]={ 0x00FF,0x0000,0x0000,0x0000,0x0000,0x0000 };
static DeviceClass_t lorawanClass = LORAWAN_CLASS;

// Devise location
const float latitude = 39.573;
const float longitude = 2.732;
const int alt = 20;

// Other params
float SensorId= 10.01;// Sensor  identifcator number 
const int DutyCycle = 60000; // Transmision and reading period

