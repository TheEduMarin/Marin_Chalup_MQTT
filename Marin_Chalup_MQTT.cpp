#include <WiFi.h>
#include <PubSubClient.h>
#include <ESP32Servo.h>

// Credenciales WiFi
const char* WIFI_SSID = "UCB-IoT";
const char* WIFI_PASS = "sistemasyteleco";

// Datos del broker MQTT
const char* MQTT_BROKER = "broker.hivemq.com";
const int MQTT_PORT = 1883;
const char* CLIENT_ID = "111112083"; // Identificador único

WiFiClient wiFiClient;
PubSubClient client(wiFiClient);

// Clase para el servo MG90S
class MG90S {
private:
  Servo servo;
  int pin;

  const int velocidadAbierta = 1200;
  const int velocidadCerrada = 1800;
  const int neutro = 1500;
  const int tiempo90 = 300;

public:
  MG90S(int servoPin) : pin(servoPin) {}

  void begin() {
    servo.setPeriodHertz(50);
    servo.attach(pin, 1000, 2000);
    stop();
  }

  void open() {
    Serial.println("Servo: OPEN");
    servo.writeMicroseconds(velocidadAbierta);
    delay(tiempo90);
    stop();
  }

  void close() {
    Serial.println("Servo: CLOSE");
    servo.writeMicroseconds(velocidadCerrada);
    delay(tiempo90);
    stop();
  }

  void stop() {
    servo.writeMicroseconds(neutro);
  }
};

// Clase para el sensor ultrasónico
class ultrasonicsens {
private:
  int echopin; 
  int trigger;

public:
  ultrasonicsens(int echopin, int trigger) {
    this->echopin = echopin;
    this->trigger = trigger;
  }

  long readUltrasonicDistance(int triggerPin, int echoPin) {
    pinMode(triggerPin, OUTPUT);
    digitalWrite(triggerPin, LOW);
    delayMicroseconds(2);
    digitalWrite(triggerPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(triggerPin, LOW);       
    pinMode(echoPin, INPUT);
    return pulseIn(echoPin, HIGH, 20000); 
  }

  long getDistance() {
    return 0.01723 * readUltrasonicDistance(trigger, echopin);  
  }
};

// Instancias de objetos
MG90S servoMotor(21);  // Servo en GPIO 21
ultrasonicsens sensor(22, 23);  // Echo en GPIO 22, Trigger en GPIO 23

// Callback para manejar mensajes entrantes
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Mensaje recibido en el tópico: ");
  Serial.print(topic);
  Serial.print(" -> ");

  String message;
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  Serial.println(message);

  if (message.equalsIgnoreCase("ON")) {
    servoMotor.open();  // ON = abrir
  } 
  else if (message.equalsIgnoreCase("OFF")) {
    servoMotor.close(); // OFF = cerrar
  }
  else {
    //Serial.println("Comando no reconocido");
  }
}

void setup() {
  Serial.begin(115200);

  servoMotor.begin(); // Iniciar el servo

  setupWiFi();

  client.setServer(MQTT_BROKER, MQTT_PORT);
  client.setCallback(callback);
}

void setupWiFi() {
  delay(10);
  Serial.println();
  Serial.print("Conectando a ");
  Serial.println(WIFI_SSID);

  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println();
  Serial.print("Conectado a WiFi. IP: ");
  Serial.println(WiFi.localIP());
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Intentando conexión MQTT...");
    if (client.connect(CLIENT_ID)) {
      Serial.println("¡Conectado!");
      client.subscribe("9516767"); // Suscripción al tópico deseado
    } else {
      Serial.print("Fallo, rc=");
      Serial.print(client.state());
      Serial.println(" intentando de nuevo en 5s");
      delay(5000);
    }
  }
}

unsigned long lastMsg = 0;

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  // Publicar la distancia cada 10 segundos
  unsigned long now = millis();
  if (now - lastMsg > 1000) {
    lastMsg = now;
    long distancia = sensor.getDistance();
    Serial.print("Distancia medida: ");
    Serial.print(distancia);
    Serial.println(" cm");

    String msg = String(distancia);
    client.publish("9516767", msg.c_str());
  }
}