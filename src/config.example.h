// config.example.h
#ifndef CONFIG_H
#define CONFIG_H

// Arquivo de exemplo para configurações
// Copie este arquivo para config.h e preencha com suas credenciais

// Configurações do WiFi
const char* ssid = "Wokwi-GUEST";  // WIFI SSID
const char* password = "";  // WIFI PASSWORD

// Configurações do MQTT
const char* mqtt_server = "io.adafruit.com";
const int mqtt_port = 1883;
const char* mqtt_user = "ADAFRUIT_USERNAME";
const char* mqtt_password = "ADAFRUIT_IO_KEY";

// Tópicos MQTT
const char* topic_data = "username/feeds/estufa.dados";
const char* topic_command = "username/feeds/estufa.comandos";
const char* topic_status = "username/feeds/estufa.status";

#endif
