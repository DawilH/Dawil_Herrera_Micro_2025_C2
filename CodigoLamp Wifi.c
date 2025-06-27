#include <WiFi.h>
#include <PubSubClient.h>
#include "esp_camera.h"

// Configuración WiFi y MQTT
const char* ssid = "tu_SSID";
const char* password = "tu_PASSWORD";
const char* mqtt_server = "broker.mqtt-dashboard.com";

WiFiClient espClient;
PubSubClient client(espClient);

// Pines ESP32-CAM
#define BUTTON_PIN 12
#define RELAY_PIN 13
#define OPEN_SENSOR_PIN 14
#define CLOSED_SENSOR_PIN 15
#define BUZZER_PIN 2
#define LAMP_PIN 4

// Estados
enum DoorState {UNKNOWN, OPEN, CLOSED, OPENING, CLOSING};
DoorState currentState = UNKNOWN;

 
enum ErrorCodes {
  NO_ERROR = 0,
  ERROR_WIFI_CONNECTION = 1,
  ERROR_MQTT_CONNECTION = 2,
  ERROR_DOOR_TIMEOUT = 3,
  ERROR_SENSOR_CONFLICT = 4
};

// Variables de tiempo
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;
unsigned long lastLampUpdate = 0;
unsigned long lampInterval = 500;
unsigned long doorOperationStart = 0;
const unsigned long DOOR_TIMEOUT = 30000; // 30 segundos timeout

void setup() {
  Serial.begin(115200);
  
  // Pines
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(OPEN_SENSOR_PIN, INPUT_PULLUP);
  pinMode(CLOSED_SENSOR_PIN, INPUT_PULLUP);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LAMP_PIN, OUTPUT);
  
  digitalWrite(RELAY_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(LAMP_PIN, LOW);
  
 
  setup_camera();
  
  // Conectar WiFi y MQTT
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  
  // Determinar estado inicial
  updateDoorState();
}

void setup_camera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = 5;
  config.pin_d1 = 18;
  config.pin_d2 = 19;
  config.pin_d3 = 21;
  config.pin_d4 = 36;
  config.pin_d5 = 39;
  config.pin_d6 = 34;
  config.pin_d7 = 35;
  config.pin_xclk = 0;
  config.pin_pclk = 22;
  config.pin_vsync = 25;
  config.pin_href = 23;
  config.pin_sscb_sda = 26;
  config.pin_sscb_scl = 27;
  config.pin_pwdn = 32;
  config.pin_reset = -1;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  
  if(psramFound()){
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }
  
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Error inicializando cámara: 0x%x", err);
  }
}

void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Conectando a ");
  Serial.println(ssid);
  
  WiFi.begin(ssid, password);
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 10) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if(WiFi.status() != WL_CONNECTED) {
    handleError(ERROR_WIFI_CONNECTION);
    return;
  }
  
  Serial.println("");
  Serial.println("WiFi conectado");
  Serial.println("Dirección IP: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* message, unsigned int length) {
  String messageTemp;
  
  for (int i = 0; i < length; i++) {
    messageTemp += (char)message[i];
  }
  
  if (String(topic) == "puerta/comando") {
    if(messageTemp == "abrir") {
      operateDoor();
    } else if(messageTemp == "estado") {
      publishStatus();
    } else if(messageTemp == "foto") {
      takeAndSendPhoto();
    }
  }
}

void takeAndSendPhoto() {
  camera_fb_t * fb = NULL;
  fb = esp_camera_fb_get();
  if(!fb) {
    Serial.println("Error al capturar foto");
    return;
  }
  
  client.publish("puerta/foto", fb->buf, fb->len);
  esp_camera_fb_return(fb);
}

void reconnect() {
  int attempts = 0;
  while (!client.connected() && attempts < 5) {
    Serial.print("Intentando conexión MQTT...");
    if (client.connect("ESP32CAMClient")) {
      Serial.println("conectado");
      client.subscribe("puerta/comando");
    } else {
      Serial.print("falló, rc=");
      Serial.print(client.state());
      Serial.println(" intentando de nuevo en 5 segundos");
      delay(5000);
      attempts++;
    }
  }
  if(!client.connected()) {
    handleError(ERROR_MQTT_CONNECTION);
  }
}

void updateDoorState() {
  bool isOpen = digitalRead(OPEN_SENSOR_PIN) == LOW;
  bool isClosed = digitalRead(CLOSED_SENSOR_PIN) == LOW;
  
  if (isOpen && isClosed) {
    handleError(ERROR_SENSOR_CONFLICT);
    currentState = UNKNOWN;
    return;
  }
  
  if (isOpen && !isClosed) {
    currentState = OPEN;
    doorOperationStart = 0;
  } else if (!isOpen && isClosed) {
    currentState = CLOSED;
    doorOperationStart = 0;
  } else if (!isOpen && !isClosed) {
    if (currentState == OPEN) {
      currentState = CLOSING;
      doorOperationStart = millis();
    } else if (currentState == CLOSED) {
      currentState = OPENING;
      doorOperationStart = millis();
    } else {
      currentState = UNKNOWN;
    }
  }
  
  // Verificar timeout
  if ((currentState == OPENING || currentState == CLOSING) && 
      doorOperationStart != 0 && 
      (millis() - doorOperationStart) > DOOR_TIMEOUT) {
    handleError(ERROR_DOOR_TIMEOUT);
    currentState = UNKNOWN;
    doorOperationStart = 0;
  }
}

void operateDoor() {
   
  if (currentState == OPEN || currentState == CLOSING || currentState == UNKNOWN) {
    digitalWrite(RELAY_PIN, HIGH);
    delay(500);  
    digitalWrite(RELAY_PIN, LOW);
    currentState = CLOSING;
    doorOperationStart = millis();
  } 
   
  else if (currentState == CLOSED || currentState == OPENING) {
    digitalWrite(RELAY_PIN, HIGH);
    delay(500);  
    digitalWrite(RELAY_PIN, LOW);
    currentState = OPENING;
    doorOperationStart = millis();
  }
  
  publishStatus();
}

void handleLamp() {
  unsigned long currentMillis = millis();
  
  switch(currentState) {
    case OPEN:
      digitalWrite(LAMP_PIN, HIGH);
      break;
    case OPENING:
    case CLOSING:
      if (currentMillis - lastLampUpdate >= lampInterval) {
        lastLampUpdate = currentMillis;
        digitalWrite(LAMP_PIN, !digitalRead(LAMP_PIN));
        lampInterval = (currentState == CLOSING) ? 250 : 500;
      }
      break;
    case CLOSED:
    case UNKNOWN:
      digitalWrite(LAMP_PIN, LOW);
      break;
  }
}

void publishStatus() {
  String status;
  switch(currentState) {
    case OPEN: status = "abierta"; break;
    case CLOSED: status = "cerrada"; break;
    case OPENING: status = "abriendo"; break;
    case CLOSING: status = "cerrando"; break;
    case UNKNOWN: status = "desconocido"; break;
  }
  
  client.publish("puerta/estado", status.c_str());
}

void handleError(ErrorCodes error) {
 
  digitalWrite(RELAY_PIN, LOW);
  
   
  for(int i = 0; i < error; i++) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(300);
    digitalWrite(BUZZER_PIN, LOW);
    delay(300);
  }
  
  
  String errorMsg = "Error: " + String(error);
  client.publish("puerta/error", errorMsg.c_str());
  
  
  for(int i = 0; i < 10; i++) {
    digitalWrite(LAMP_PIN, HIGH);
    delay(100);
    digitalWrite(LAMP_PIN, LOW);
    delay(100);
  }
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  
 
  int reading = digitalRead(BUTTON_PIN);
  if (reading == LOW && (millis() - lastDebounceTime) > debounceDelay) {
    lastDebounceTime = millis();
    operateDoor();
  }
  
   
  updateDoorState();
  
  
  handleLamp();
  
  / 
  static unsigned long lastStatusUpdate = 0;
  if (millis() - lastStatusUpdate > 10000) {
    lastStatusUpdate = millis();
    publishStatus();
  }
}