/*************************************************************
 *                  LIBRERÍAS                                *
 *************************************************************/
#include <WiFi.h>
#include <PubSubClient.h>

// Librerías para I2C y sensor ambiental BME280
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

/*************************************************************
 *              CONFIGURACIÓN WIFI                           *
 *************************************************************/
const char* ssid = "Latinas";
const char* password = "Latinas9";

/*************************************************************
 *              CONFIGURACIÓN SERVIDOR MQTT                  *
 *************************************************************/
const char* mqtt_server = "192.168.0.69";
const int mqtt_port = 1883;
const char* mqtt_user = "latina1";
const char* mqtt_password = "latina1";

/*************************************************************
 *              TOPICS MQTT (PUBLICACIÓN)                    *
 *************************************************************/
const char* topic_distancia   = "sensor ultrasonico";
const char* topic_dac         = "casa/sensor/dac";
const char* topic_voltaje     = "casa/sensor/voltaje";
const char* topic_boton       = "casa/boton";

// Topics para sensor BME280
const char* topic_temperatura = "temperatura";
const char* topic_presion     = "presion";
const char* topic_humedad     = "humedad";

/*************************************************************
 *          TOPICS MQTT (SUSCRIPCIÓN - PLC)                  *
 *************************************************************/
const char* mqtt_subscribe_topic_I2 = "entradaI2";  
const char* mqtt_subscribe_topic_I4 = "stopI4";     // 

/*************************************************************
 *              OBJETOS GLOBALES                             *
 *************************************************************/
WiFiClient espClient;
PubSubClient client(espClient);

// Sensor BME280
Adafruit_BME280 bme;

/*************************************************************
 *                  PINES ESP32                              *
 *************************************************************/
const int pinReleManual = 4;    // Relé Manual (I2)
const int pinRelePID    = 32;   // Relé PID (I4)
const int PIN_TRIG      = 14;
const int PIN_ECHO      = 12;
const int PIN_DAC       = 25;

// Pines I2C
#define SDA_PIN 21
#define SCL_PIN 22

/*************************************************************
 *              PARÁMETROS DEL SISTEMA                       *
 *************************************************************/
const unsigned long tiempoPulso = 150;     // Duración del pulso al PLC (ms)
const unsigned long intervaloSensor = 1000; // Periodo de envío de sensores (ms)
unsigned long ultimoEnvioSensor = 0;

/*************************************************************
 *              CONEXIÓN WIFI                                *
 *************************************************************/
void setup_wifi() {
  Serial.print("Conectando a ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA); //se conecta a la SSID que se le ajusto anteriormente con su contraseña
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nWiFi conectado");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
}

/*************************************************************
 *          FUNCIÓN DE PULSO PARA RELÉS                      *
 *************************************************************/
void pulsoRele(int pin) {  //active el relé, espera un tiempo y lo vuelve a desactivar
  digitalWrite(pin, HIGH);
  delay(tiempoPulso);
  digitalWrite(pin, LOW);
}

/*************************************************************
 *              CALLBACK MQTT                                *
 *************************************************************/
void callback(char* topic, byte* message, unsigned int length) { // recibe los mensajes desde MQTT y los pasa a una string 

  String msg = "";
  for (unsigned int i = 0; i < length; i++) {
    msg += (char)message[i];
  }

  // Entrada I2 → modo Manual → SOLO responde cuando le llega "ON"
  if (String(topic) == mqtt_subscribe_topic_I2 && msg == "ON") {
    Serial.println("Acción: Pulso relé MANUAL (I2)");
    pulsoRele(pinReleManual);
  }

  // Entrada I4 → modo PID → SOLO responde cuando le llega "OFF"
  else if (String(topic) == mqtt_subscribe_topic_I4 && msg == "OFF") {
    Serial.println("Acción: Pulso relé PID (I4)");
    pulsoRele(pinRelePID);
  }
}

/*************************************************************
 *              RECONEXIÓN MQTT                              *
 *************************************************************/
 //Reconectar automáticamente el ESP32 al broker MQTT cuando se pierde la conexión.
void reconnect_mqtt() { 
  while (!client.connected()) {

    Serial.print("Intentando conexión MQTT...");
    String clientId = "ESP32_" + String(random(0xffff), HEX);

    if (client.connect(clientId.c_str(), mqtt_user, mqtt_password)) {
      Serial.println(" CONECTADO");

      client.subscribe(mqtt_subscribe_topic_I2);
      client.subscribe(mqtt_subscribe_topic_I4);

      Serial.println("Suscrito a entradaI2 y stopI4");
    } 
    else {
      Serial.print("Error rc=");
      Serial.print(client.state());
      Serial.println(" -> Reintento en 5s");
      delay(5000);
    }
  }
}

/*************************************************************
 *                  SETUP                                    *
 *************************************************************/
void setup() {
  Serial.begin(115200);

  //pines para cada parte del sistema
  pinMode(pinReleManual, OUTPUT);
  pinMode(pinRelePID, OUTPUT);
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);

  digitalWrite(pinReleManual, LOW);
  digitalWrite(pinRelePID, LOW);
  digitalWrite(PIN_TRIG, LOW);

  // Inicialización I2C y BME280
  Wire.begin(SDA_PIN, SCL_PIN);

  if (!bme.begin(0x76)) {
    Serial.println("ERROR: No se detectó BME280");
    while (1);
  }
  Serial.println("Sensor BME280 inicializado");

  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
}

/*************************************************************
 *                  LOOP PRINCIPAL                           *
 *************************************************************/
void loop() {

  if (!client.connected()) {
    reconnect_mqtt();
  }
  client.loop();

  unsigned long ahora = millis();

  // ===== BLOQUE DE SENSORES (1s) =====
  if (ahora - ultimoEnvioSensor >= intervaloSensor) {
    ultimoEnvioSensor = ahora;

    /*********** SENSOR ULTRASÓNICO ***********/
    digitalWrite(PIN_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(PIN_TRIG, LOW);

    long duracion = pulseIn(PIN_ECHO, HIGH);
    float distancia_cm = (duracion * 0.034) / 2.0;

    // Rango físico real del sistema
    distancia_cm = constrain(distancia_cm, 10, 50);

    // Inversión SOLO para visualización MQTT (Node-RED)
    // Cerca (10 cm)  -> 50
    // Lejos (50 cm)  -> 10
    float distancia_mqtt = 60.0 - distancia_cm;

    /*********** DAC ***********/
    int valorDAC = map(distancia_cm, 10, 50, 0, 255);
    float voltaje = (valorDAC * 3.3) / 255.0;
    dacWrite(PIN_DAC, valorDAC);

    /*********** PUBLICACIÓN MQTT ***********/
    client.publish(topic_distancia, String(distancia_mqtt).c_str());
    client.publish(topic_dac, String(valorDAC).c_str());
    client.publish(topic_voltaje, String(voltaje).c_str());

    /*********** SENSOR BME280 ***********/
    float temperatura = bme.readTemperature();
    float presion     = bme.readPressure() / 100.0F;
    float humedad     = bme.readHumidity();

    client.publish(topic_temperatura, String(temperatura).c_str());
    client.publish(topic_presion, String(presion).c_str());
    client.publish(topic_humedad, String(humedad).c_str());

    /*********** MONITOR SERIAL ***********/
    Serial.print("Dist real: ");
    Serial.print(distancia_cm);
    Serial.print(" cm | Dist MQTT: ");
    Serial.print(distancia_mqtt);
    Serial.println(" cm");

    Serial.print("DAC: "); Serial.print(valorDAC);
    Serial.print(" | V: "); Serial.print(voltaje); Serial.println(" V");

    Serial.print("Temp: "); Serial.print(temperatura); Serial.print(" °C | ");
    Serial.print("Presión: "); Serial.print(presion); Serial.print(" hPa | ");
    Serial.print("Humedad: "); Serial.print(humedad); Serial.println(" %");
    Serial.println("------------------------------------------------");
  }
}
