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
const char* topic_power_value = TOPIC_POWER_VALUE;
const char* topic_start_measure = TOPIC_START_MEASURE;
const char* topic_measure_period = TOPIC_MEASURE_PERIOD;
const int wifi_connected_led = 2;

// Criando instâncias do cliente Wi-Fi e do cliente MQTT
WiFiClientSecure wifiClient;
PubSubClient mqttClient(wifiClient);

void resetOccupancy() {
  isOccupied = false;
  mqttClient.publish(topic_esp_occupied, "0", false);
}

void generateAndPublishIVCurve(int seconds) {
  const float maxCurrent = 3.0;
  const float maxVoltage = 24.0;
  const int numPoints = seconds * 3;
  const float kneeVoltage = 18.0;

  for (int i = 0; i < numPoints; i++) {
    float voltage = (maxVoltage * i) / (numPoints - 1);
    float current;
    
    if (voltage < kneeVoltage) {
      current = maxCurrent - (maxCurrent * 0.01 * (voltage / kneeVoltage));
    } else {
      float fallRatio = (voltage - kneeVoltage) / (maxVoltage - kneeVoltage);
      current = maxCurrent * (1 - fallRatio * 1.2);
      current = (current > 0.0) ? current : 0.0;  // Versão alternativa ao max()
    }

    float power = voltage * current;

    mqttClient.publish(topic_tension_value, String(voltage, 2).c_str(), false);
    mqttClient.publish(topic_current_value, String(current, 3).c_str(), false);
    mqttClient.publish(topic_power_value, String(power, 2).c_str(), false);

    delay(100);
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
  // Converter payload para String
  String payloadStr;
  for (int i = 0; i < length; i++) {
    payloadStr += (char)payload[i];
  }

  Serial.println(payloadStr);

  if (strcmp(topic, topic_start_measure) == 0) {
    isOccupied = true;
    mqttClient.publish(topic_esp_occupied, "1", false);
    Serial.println("Medição solicitada");
    // Configura timeout de 5 segundos
    occupancyTimeout.once(5, resetOccupancy);
  }

  if (strcmp(topic, topic_measure_period) == 0 && isOccupied) {
    // Cancela o timeout pois recebemos o measureTime
    occupancyTimeout.detach();
    Serial.println("Período de medição recebido: " + payloadStr);
    // Converte payload para inteiro
    int seconds = payloadStr.toInt();
    seconds = min(seconds, 60); // Limita a 60 segundos por segurança

    generateAndPublishIVCurve(seconds);

    // Libera o ESP32 após completar as medições
    resetOccupancy();
  }
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
      mqttClient.subscribe(topic_measure_period, 1); //QOS = 1
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

  pinMode(wifi_connected_led, OUTPUT);
  digitalWrite(wifi_connected_led, LOW);
  
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("Connected to Wi-Fi");
  digitalWrite(wifi_connected_led, HIGH);

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