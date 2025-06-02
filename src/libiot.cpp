/*
 * The MIT License
 *
 * Copyright 2024 Alvaro Salazar <alvaro@denkitronik.com>.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <libiot.h>
#include <SHTSensor.h>
#include <libota.h>

time_t now0;
//#define PRINT
#ifdef PRINT
#define PRINTD(x, y) Serial.println(x, y)
#define PRINTLN(x) Serial.println(x)
#define PRINT(x) Serial.print(x)
#else
#define PRINTD(x, y)
#define PRINTLN(x)
#define PRINT(x)
#endif

SHTSensor sht;     //Sensor SHT21
String alert = ""; //Mensaje de alerta
extern const char * client_id;  //ID del cliente MQTT


/**
 * Consulta y guarda el tiempo actual con servidores SNTP.
 */
time_t setTime() {
  //Sincroniza la hora del dispositivo con el servidor SNTP (Simple Network Time Protocol)
  Serial.print("Ajustando el tiempo usando SNTP");
  configTime(-5 * 3600, 0, "pool.ntp.org", "time.nist.gov"); //Configura la zona horaria y los servidores SNTP
  now0 = time(nullptr);              //Obtiene la hora actual
  while (now0 < 1700000000) {        //Espera a que el tiempo sea mayor a 1700000000 (1 de enero de 2024)
    delay(500);                     //Espera 500ms antes de volver a intentar obtener la hora
    Serial.print(".");
    now0 = time(nullptr);            //Obtiene la hora actual
  }
  Serial.println(" hecho!");
  struct tm timeinfo;               //Estructura que almacena la información de la hora
  gmtime_r(&now0, &timeinfo);        //Obtiene la hora actual
  Serial.print("Tiempo actual: ");  //Una vez obtiene la hora, imprime en el monitor el tiempo actual
  Serial.print(asctime(&timeinfo));
  return now0;
}

/**
 * Conecta el dispositivo con el bróker MQTT usando
 * las credenciales establecidas.
 * Si ocurre un error lo imprime en la consola.
 */
void checkMQTT() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
}

/**
 * Adquiere la dirección MAC del dispositivo y la retorna en formato de cadena.
 */
String getMacAddress() {
  uint8_t mac[6];
  char macStr[18];
  WiFi.macAddress(mac);
  snprintf(macStr, sizeof(macStr), "ESP32-%02X%02X%02X%02X%02X%02X", 
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  return String(macStr);
}


/**
 * Función que se ejecuta cuando se establece conexión con el servidor MQTT
 */
void reconnect() {
  while (!client.connected()) { //Mientras no esté conectado al servidor MQTT
    Serial.print("Conectando al servidor MQTT...");
    if (client.connect(client_id, mqtt_user, mqtt_password)) { //Intenta conectarse al servidor MQTT
      Serial.println("Conectado");
      client.subscribe(MQTT_TOPIC_SUB); //Se suscribe al tópico de suscripción
      setupOTA(client); //Configura la funcionalidad OTA
      Serial.println("Suscripción a " + String(MQTT_TOPIC_SUB));
    } else {
      Serial.println("Problema con la conexión, revise los valores de las constantes MQTT");
      int state = client.state();
      Serial.print("Código de error = ");
      alert = "MQTT error: " + String(state);
      Serial.println(state);
      if ( client.state() == MQTT_CONNECT_UNAUTHORIZED ) ESP.deepSleep(0);
      delay(5000); // Espera 5 segundos antes de volver a intentar
    }
  }
}


/**
 * Función setupIoT que configura el certificado raíz, el servidor MQTT y el puerto
 */
void setupIoT() {
  Wire.begin();                 //Inicializa el bus I2C
  espClient.setCACert(root_ca); //Configura el certificado raíz de la autoridad de certificación
  client.setServer(mqtt_server, mqtt_port);   //Configura el servidor MQTT y el puerto seguro
  client.setCallback(receivedCallback);       //Configura la función que se ejecutará cuando lleguen mensajes a la suscripción
  setTime();                    //Ajusta el tiempo del dispositivo con servidores SNTP
  setupSHT();                   //Configura el sensor SHT21
}


/**
 * Configura el sensor SHT21
 */
void setupSHT() {
  if (sht.init()) Serial.print("SHT init(): Exitoso\n");
  else Serial.print("SHT init(): Fallido\n");
  sht.setAccuracy(SHTSensor::SHT_ACCURACY_MEDIUM); // soportado solo por el SHT3x
}


/**
 * Verifica si ya es momento de hacer las mediciones de las variables.
 * si ya es tiempo, mide y envía las mediciones.
 */
bool measure(SensorData * data) {
  if ((millis() - measureTime) >= MEASURE_INTERVAL * 1000 ) {
    PRINTLN("\nMidiendo variables...");
    measureTime = millis();    
    if (sht.readSample()) {
        data->temperature = sht.getTemperature();
        data->humidity = sht.getHumidity();
        PRINT(" %RH ❖ Temperatura: ");
        PRINTD(data->humidity, 2);
        PRINT(" %RH ❖ Temperatura: ");
        PRINTD(data->temperature, 2);
        PRINTLN(" °C");
        return true;
    } else {
        Serial.print("Error leyendo la muestra\n");
        return false;
    }
  }
  return false;
}

/**
 * Verifica si ha llegdo alguna alerta al dispositivo.
 * Si no ha llegado devuelve OK, de lo contrario retorna la alerta.
 * También asigna el tiempo en el que se dispara la alerta.
 */
String checkAlert() {
  if (alert.length() != 0) {
    if ((millis() - alertTime) >= ALERT_DURATION * 1000 ) {
      alert = "";
      alertTime = millis();
    }
    return alert;
  } else return "OK";
}

/**
 * Publica la temperatura y humedad dadas al tópico configurado usando el cliente MQTT.
 */
void sendSensorData(float temperatura, float humedad) {
  String data = "{";
  data += "\"temperatura\": "+ String(temperatura, 1) +", ";
  data += "\"humedad\": "+ String(humedad, 1);
  data += "}";
  char payload[data.length()+1];
  data.toCharArray(payload,data.length()+1);
  PRINTLN("client id: " + String(client_id) + "\ntopic: " + String(MQTT_TOPIC_PUB) + "\npayload: " + data);
  client.publish(MQTT_TOPIC_PUB, payload);
}
/**
 * Publica todos los datos relevantes al tópico configurado usando el cliente MQTT.
 */
void sendAllSensorData(float temperatura, float humedad, float bpm, uint8_t spo2, double lat, double lon, const char* timestamp) {
  String data = "{";
  data += "\"temperatura\": " + String(temperatura, 1) + ", ";
  data += "\"humedad\": " + String(humedad, 1) + ", ";
  data += "\"bpm\": " + String(bpm, 1) + ", ";
  data += "\"spo2\": " + String(spo2) + ", ";
  data += "\"lat\": " + String(lat, 6) + ", ";
  data += "\"lon\": " + String(lon, 6) + ", ";
  data += "\"timestamp\": \"" + String(timestamp) + "\"";
  data += "}";

  char payload[data.length() + 1];
  data.toCharArray(payload, data.length() + 1);
  PRINTLN("client id: " + String(client_id) + "\ntopic: " + String(MQTT_TOPIC_PUB) + "\npayload: " + data);
  client.publish(MQTT_TOPIC_PUB, payload);
}

/**
 * Función que se ejecuta cuando llega un mensaje a la suscripción MQTT.
 * Construye el mensaje que llegó y si contiene ALERT lo asgina a la variable 
 * alert que es la que se lee para mostrar los mensajes.
 * También verifica si el mensaje es para actualización OTA.
 */
void receivedCallback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Recibido [");
  Serial.print(topic);
  Serial.print("]: ");
  String data = ""; // Se inicializa con un string vacío
  for (int i = 0; i < length; i++) data += String((char)payload[i]); // Se recorre el payload y se va concatenando al string data
  Serial.println(data);
  
  // Verifica si el mensaje es para actualización OTA
  if (String(topic) == OTA_TOPIC) {
    checkOTAUpdate(data.c_str());
    return;
  }
  
  // Verifica si el mensaje contiene una alerta
  if (data.indexOf("ALERT") >= 0) alert = data; // Si el mensaje contiene la palabra ALERT, se asigna a la variable alert
}