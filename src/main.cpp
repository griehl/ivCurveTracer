#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <Ticker.h> // Para o timeout
#include "config.h"

bool isOccupied = false;
Ticker occupancyTimeout;

// Credenciais do Wi-Fi
const char* ssid = WIFI_SSID;
const char* password = WIFI_PASSWORD;

// Credenciais do MQTT Broker
const char* mqtt_broker = MQTT_BROKER;
const int mqtt_port = MQTT_PORT;
const char* mqtt_username = MQTT_USERNAME;
const char* mqtt_password = MQTT_PASSWORD;

// Tópicos MQTT
const char* topic_esp_occupied = TOPIC_ESP_OCCUPIED;
const char* topic_current_value = TOPIC_CURRENT_VALUE;
const char* topic_tension_value = TOPIC_TENSION_VALUE;
const char* topic_start_measure = TOPIC_START_MEASURE;
const char* topic_measure_time = TOPIC_MEASURE_TIME;

// Criando instâncias do cliente Wi-Fi e do cliente MQTT
WiFiClientSecure wifiClient;
PubSubClient mqttClient(wifiClient);

void callback(char* topic, byte* payload, unsigned int length) {
  // Converter payload para String
  String payloadStr;
  for (int i = 0; i < length; i++) {
    payloadStr += (char)payload[i];
  }

  if (strcmp(topic, topic_start_measure) == 0) {
    isOccupied = true;
    mqttClient.publish(topic_esp_occupied, "1", true);
    
    // Configura timeout de 5 segundos
    occupancyTimeout.once(5, resetOccupancy);
  }

  if (strcmp(topic, topic_measure_time) == 0 && isOccupied) {
    // Cancela o timeout pois recebemos o measureTime
    occupancyTimeout.detach();
    
    // Converte payload para inteiro
    int seconds = payloadStr.toInt();
    seconds = min(seconds, 60); // Limita a 60 segundos por segurança

    for (int i = 0; i < seconds; i++) {
      // Aqui você colocaria os valores reais das medições
      mqttClient.publish(topic_current_value, "VALOR_CORRENTE", true);
      mqttClient.publish(topic_tension_value, "VALOR_TENSAO", true);
      
      if (i < seconds - 1) { // Não espera depois do último envio
        delay(1000);
      }
    }
    
    // Libera o ESP32 após completar as medições
    resetOccupancy();
  }
}

void resetOccupancy() {
  isOccupied = false;
  mqttClient.publish(topic_esp_occupied, "0", true);
}

void reconnect() {
  Serial.println("Connecting to MQTT Broker...");
  while (!mqttClient.connected()) {
    Serial.println("Reconnecting to MQTT Broker...");
    String clientId = "ESP32Client-";
    clientId += String(random(0xffff), HEX);
    
    if (mqttClient.connect(clientId.c_str(), mqtt_username, mqtt_password)) {
      Serial.println("Connected to MQTT Broker.");
      mqttClient.subscribe(topic_start_measure, 1); //QOS = 1
      mqttClient.subscribe(topic_measure_time, 1); //QOS = 1
    } else {
      Serial.print("Failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(115200);
  
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("Connected to Wi-Fi");

  // Inicializa o WifiClientSecure
  wifiClient.setInsecure(); // Use somente para testes, permite conectar sem um certificado raiz
  
  mqttClient.setServer(mqtt_broker, mqtt_port);
  mqttClient.setCallback(callback);
  
}

void loop() {

  if (!mqttClient.connected()) {
    reconnect();
  }
  mqttClient.loop();

}