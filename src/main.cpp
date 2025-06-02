#include <WiFi.h>
#include <libiot.h>
#include <libwifi.h>
#include <libdisplay.h>
#include <libota.h>
#include "HealthMonitor.h"
SensorData data;  // Estructura para almacenar los datos de temperatura y humedad del SHT21
time_t hora;      // Timestamp de la hora actual
HealthMonitor monitor;//objeto monitor con los otros sensores
/**
 * Configura el dispositivo para conectarse a la red WiFi y ajusta parametros IoT
 */
void setup() {
  Serial.begin(115200);     // Paso 1. Inicializa el puerto serie
  listWiFiNetworks();       // Paso 2. Lista las redes WiFi disponibles
  delay(1000);              // -- Espera 1 segundo para ver las redes disponibles
  startDisplay();           // Paso 3. Inicializa la pantalla OLED
  displayConnecting(ssid);  // Paso 4. Muestra en la pantalla el mensaje de "Conectandose a :" y luego el nombre de la red a la que se conecta
  startWiFi("");            // Paso 5. Inicializa el servicio de WiFi
  setupIoT();               // Paso 6. Inicializa el servicio de IoT
  hora = setTime();         // Paso 7. Ajusta el tiempo del dispositivo con servidores SNTP
  monitor.begin();          //libreria con todos los codigos existentes de los otros sensores
}

// Función loop
void loop() {
  checkWiFi();                                                   // Paso 1. Verifica la conexión a la red WiFi y si no está conectado, intenta reconectar
  checkMQTT();                                                   // Paso 2. Verifica la conexión al servidor MQTT y si no está conectado, intenta reconectar
  String message = checkAlert();                                 // Paso 3. Verifica si hay alertas y las retorna en caso de haberlas
  if(measure(&data)){   
    monitor.update();                                         // Paso 4. Realiza una medición de temperatura y humedad
    displayLoop(message, hora, data.temperature, data.humidity); // Paso 5. Muestra en la pantalla el mensaje recibido, las medidas de temperatura y humedad
    sendSensorData(data.temperature, data.humidity);             // Paso 6. Envía los datos de temperatura y humedad al servidor MQTT xdxdxd
  }  
  static uint32_t lastSend = 0;
  if (millis() - lastSend > 5000) {
    lastSend = millis();

    // Variables donde se guardarán los datos actuales
    float temperatura, humedad, bpm;
    uint8_t spo2;
    double lat, lon;
    char timestamp[20];

    // Obtenemos todos los valores
    monitor.getCurrentReadings(temperatura, humedad, bpm, spo2, lat, lon, timestamp, sizeof(timestamp));

    // Llamamos a la función ya existente en libiot.cpp
    sendAllSensorData(temperatura, humedad, bpm, spo2, lat, lon, timestamp);
  } 


}
