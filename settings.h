// WiFi Configuration
const char* ssid = "MiFibra-2D79-terraza";
const char* password = "";

// MQTT Configuration
const char* mqtt_server = "192.168.1.114";
const int mqtt_port = 1883;
const char* mqtt_id = "anemometer_cups";
const char* mqtt_sub_topic_healthcheck = "/home/meteo/anemometer/healthcheck";
const char* mqtt_pub_topic_voltage = "/home/meteo/anemometer/wind_speed";
const char* mqtt_sub_topic_operation = "/home/meteo/anemometer/operation";

// Other params
const int update_time_sensors = 59000;
