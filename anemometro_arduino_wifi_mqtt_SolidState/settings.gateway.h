// WiFi Configuration
// const char* ssid = "Domohome_GRX";
// const char* password = "Nomeolvides2022";
const char* ssid = "MiFibra-2D79";
const char* password = "oqnQYSp3";

// MQTT Configuration
const char* mqtt_server = "mcortshome.mooo.com";
const int   mqtt_port = 11884;
const char* mqttUser = "iotauser";
const char* mqttPassword = "Xx1tt&9&9";
const char* mqtt_id = "anemometer_solid_1";
const char* mqtt_sub_topic_healthcheck = "/promenade_espmi/meteo/wind/healthcheck";
const char* mqtt_sub_topic_operation = "/promenade_espmi/meteo/wind/operation";
const char* mqtt_sub_topic_ip = "/promenade_espmi/meteo/wind/ip";
const char* mqtt_sub_topic_cp = "/promenade_espmi/meteo/wind/cp";

const char* mqtt_pub_topic_windspeed = "/promenade_espmi/meteo/wind/speed";
const char* mqtt_pub_topic_windspeedpeak = "/promenade_espmi/meteo/wind/speed_peak";
const char* mqtt_pub_topic_windspeedmin = "/promenade_espmi/meteo/windspeed_min";
const char* mqtt_pub_topic_windtemperature = "/promenade_espmi/meteo/wind/temperature";


// Other params
const int update_time_sensors = 59000;
