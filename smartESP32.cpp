#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <ESP32Servo.h>


const char* ssid = "UCB-IoT";
const char* password = "sistemasyteleco";


const char* aws_endpoint = "a3ovfyx2pmf1sk-ats.iot.us-east-1.amazonaws.com";
const int aws_port = 8883;
const char* thingName = "ESP32-thing";


const char* ca_cert = R"EOF(
-----BEGIN CERTIFICATE-----
MIIDQTCCAimgAwIBAgITBmyfz5m/jAo54vB4ikPmljZbyjANBgkqhkiG9w0BAQsF
ADA5MQswCQYDVQQGEwJVUzEPMA0GA1UEChMGQW1hem9uMRkwFwYDVQQDExBBbWF6
b24gUm9vdCBDQSAxMB4XDTE1MDUyNjAwMDAwMFoXDTM4MDExNzAwMDAwMFowOTEL
MAkGA1UEBhMCVVMxDzANBgNVBAoTBkFtYXpvbjEZMBcGA1UEAxMQQW1hem9uIFJv
b3QgQ0EgMTCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBALJ4gHHKeNXj
ca9HgFB0fW7Y14h29Jlo91ghYPl0hAEvrAIthtOgQ3pOsqTQNroBvo3bSMgHFzZM
9O6II8c+6zf1tRn4SWiw3te5djgdYZ6k/oI2peVKVuRF4fn9tBb6dNqcmzU5L/qw
IFAGbHrQgLKm+a/sRxmPUDgH3KKHOVj4utWp+UhnMJbulHheb4mjUcAwhmahRWa6
VOujw5H5SNz/0egwLX0tdHA114gk957EWW67c4cX8jJGKLhD+rcdqsq08p8kDi1L
93FcXmn/6pUCyziKrlA4b9v7LWIbxcceVOF34GfID5yHI9Y/QCB/IIDEgEw+OyQm
jgSubJrIqg0CAwEAAaNCMEAwDwYDVR0TAQH/BAUwAwEB/zAOBgNVHQ8BAf8EBAMC
AYYwHQYDVR0OBBYEFIQYzIU07LwMlJQuCFmcx7IQTgoIMA0GCSqGSIb3DQEBCwUA
A4IBAQCY8jdaQZChGsV2USggNiMOruYou6r4lK5IpDB/G/wkjUu0yKGX9rbxenDI
U5PMCCjjmCXPI6T53iHTfIUJrU6adTrCC2qJeHZERxhlbI1Bjjt/msv0tadQ1wUs
N+gDS63pYaACbvXy8MWy7Vu33PqUXHeeE6V/Uq2V8viTO96LXFvKWlJbYK8U90vv
o/ufQJVtMVT8QtPHRh8jrdkPSHCa2XV4cdFyQzR1bldZwgJcJmApzyMZFo6IQ6XU
5MsI+yMRQ+hDKXJioaldXgjUkK642M4UwtBV8ob2xJNDd2ZhwLnoQdeXeGADbkpy
rqXRfboQnoZsG4q5WTP468SQvvG5
-----END CERTIFICATE-----
)EOF";

const char* client_cert = R"KEY(
-----BEGIN CERTIFICATE-----
MIIDWTCCAkGgAwIBAgIUVupxhqB1tcffXqI7Ji6zkvxvZeowDQYJKoZIhvcNAQEL
BQAwTTFLMEkGA1UECwxCQW1hem9uIFdlYiBTZXJ2aWNlcyBPPUFtYXpvbi5jb20g
SW5jLiBMPVNlYXR0bGUgU1Q9V2FzaGluZ3RvbiBDPVVTMB4XDTI1MDUxMTIxNTUx
MFoXDTQ5MTIzMTIzNTk1OVowHjEcMBoGA1UEAwwTQVdTIElvVCBDZXJ0aWZpY2F0
ZTCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBAMwKmiq1LLkq5nriGPYG
aGFoSIwLRNcygApM+N0mk+JducgKE+S7lDk9TbNlWwgkOmD4Mfw7YSMeAhJ6U3el
idVUO+np1GlnDJtFS+Wwu+HT6rynxvOWBO+3/8CnmM7G3Xiz1vUBDN4V7JiVJw8K
q0ANSNHnaaP+fPLQiyChMqYlj4Nt4vSl6mTvHYU7e4PdRx0TK4kjd0T68xDMiihX
58iTmwOsiBmILST2uBkCqiRM5MaKHXmxkjtbDD/SHMV79PvuoqpeDAs7bDZQCkHZ
8S7EMwU4Y++vtUFbOLJgfoBRZyjWtrA9hygazTU1ry2Qh5t4mxs02ap53A41GTdh
xGMCAwEAAaNgMF4wHwYDVR0jBBgwFoAU+chjdmLsonEznnYqzNi9Hrc+6swwHQYD
VR0OBBYEFOj2YidYOhXkHBGWxBAxKqK4ZUv2MAwGA1UdEwEB/wQCMAAwDgYDVR0P
AQH/BAQDAgeAMA0GCSqGSIb3DQEBCwUAA4IBAQBMeAc1i8Pk446wM3bVMDbLU1Ft
iGVNts2nePtTUE926AF0qwTLZGwS7/y2+UPkWcYvSYzzFzMOBNAGq8sZsLL/Xr0a
IrFQHIo7NtlMWlYZbqK0v08KQr9Rg636ckMxTquW809RL9BuozKtlQv+I/z6w3ks
/3abuz+2ylQLvZen0WKaLmOBFoN9vGK2O7lnD9sQF/0Y9DkQI61V/vZm8etGQ5f0
g3GUIlgGvwBdjoncbB0kalnwpJsKnaLKocz6AJ1ZBHbWMxQltdPYA2JRHD2/tfqs
ebQ1kIScc7ozK2Kx3rpuPEuMiAUxdYgvKo/Hcpx7gBwLIUNB/UErfL6LSKNh
-----END CERTIFICATE-----
)KEY";

const char* client_key = R"KEY(
-----BEGIN RSA PRIVATE KEY-----
MIIEowIBAAKCAQEAzAqaKrUsuSrmeuIY9gZoYWhIjAtE1zKACkz43SaT4l25yAoT
5LuUOT1Ns2VbCCQ6YPgx/DthIx4CEnpTd6WJ1VQ76enUaWcMm0VL5bC74dPqvKfG
85YE77f/wKeYzsbdeLPW9QEM3hXsmJUnDwqrQA1I0edpo/588tCLIKEypiWPg23i
9KXqZO8dhTt7g91HHRMriSN3RPrzEMyKKFfnyJObA6yIGYgtJPa4GQKqJEzkxood
ebGSO1sMP9IcxXv0++6iql4MCztsNlAKQdnxLsQzBThj76+1QVs4smB+gFFnKNa2
sD2HKBrNNTWvLZCHm3ibGzTZqnncDjUZN2HEYwIDAQABAoIBAQC27sZ7hcWcGfdb
srwYT3Rj6IAqdqv4mAEtuwbgF4c8yzUWQNrHFtn0W57/ON7dtMVWTQ9bic00E+Jd
uKh+u76QsZ2pwQe8nDQ0PXOnccVw2XM6Qt3fY9/3ICs/CcO5l5fWcL5HGPL1Zgii
YA8NsE4K9h4Uevo+mpphhFFIJt3nEGigQ62thQkad4fZ5WEz22wKNpLyCdciaWAI
RtPvh/Po/gYrKTXlC3GgZbUaWVij6SBzag68gsATpG8zLcnCWGR9fNOcbJjLBKNj
PiKTg91cKoEUM8Un0ACrnWvwjTuCYgFmCGRbf+v/faFaFNIXPrwYzcIYI+MsrkQU
BQ46yjQBAoGBAPggq6emZ8W1pOZput17hD75Q9YY2pAXarsM05g2NXR3lkplTXN8
/YzzsQ4/TotuUQkRuLDnT+/KyHDQn14F/pWTK/QAf0ju6AUq3R7/+IxaEHorOjOy
OMoEeLfnYoXSpzRxIRlJsb/wLKAyZxk3tlIkUC0JgnMfc/2c73sSkfIBAoGBANKD
22Jr2E97CxCHgnf01bFSK1aahZKPnSF8w7cw7DjixuaKKqu4wWgmZm8hwIziJYsy
V+Hx6MVS7ia5AS9xl3DfT6Q7YuOvSZWn2OXkIWknHkdEiJWuDjljw/5URzKxSrrl
x87D2TCelee4OoubptN8Cc9WhfnYDS5O3V01dS5jAoGAX7Fh4i9IGDTcmswt+BsZ
vputcNV2/Vk6nVbuB+boKLmyKZD2FNMgigq1vxPlO9rYv9Go0/jxmD4xnWin/b/8
Fj2DFwBWFo+kxdFpI/p+JcraTfN759IVGrwsvmgrVAcQ2yXCKRmFxsteuo2Vu2EE
HJvKnAPt3m5D8oxiw6k+xgECgYATPaYBmuXMbeyW+Cu+lwwpBt+bLMhkmue7tdGV
ns0YbfHMBHjd9ScIQ+o84hLdTZYs4M8Tga7KykemFma7VWCIGoEG+76QHOdW3F6A
tJcxKzgjP2pUc/Is4Kd7SWywf2/bIUFf7pqdTQFaioYfWHyo+79RyhC4wzyBLQgi
is5JDQKBgEpXd3SdF/mWmEERm1oG5vl51T6r5em4Cy+yhJidPkS3VtqvdPP47Vrb
7CcAPVoIjPPG3PHF+p3WlIqg6outb71Vct8oaWjqDVINIJYSKL0qlQFkZBmtiYHr
MzpmiCFBPCQxwY+f/GxI45wiRwE/76UI/bOEeLVfiwxc4g17otb0
-----END RSA PRIVATE KEY-----
)KEY";


class ultrasonicsens {
private:
    int trigger;
    int echo;

public:
    ultrasonicsens(int t, int e) {
        trigger = t;
        echo = e;
        pinMode(trigger, OUTPUT);
        pinMode(echo, INPUT);
    }

    float distancia() {
        digitalWrite(trigger, LOW);
        delayMicroseconds(2);
        digitalWrite(trigger, HIGH);
        delayMicroseconds(10);
        digitalWrite(trigger, LOW);
        long duracion = pulseIn(echo, HIGH);
        float d = duracion * 0.034 / 2;
        return d;
    }
};

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
    Serial.println(" Servo: OPEN");
    servo.writeMicroseconds(velocidadAbierta);
    delay(tiempo90);
    stop();
  }

  void close() {
    Serial.println(" Servo: CLOSE");
    servo.writeMicroseconds(velocidadCerrada);
    delay(tiempo90);
    stop();
  }

  void stop() {
    servo.writeMicroseconds(neutro);
  }
};



ultrasonicsens sensor(23, 22);  // Trigger: GPIO 5, Echo: GPIO 18
MG90S servo(21);               // Servo: GPIO 19

WiFiClientSecure net;
PubSubClient client(net);


String shadowUpdate = "$aws/things/ESP32/shadow/update";
String shadowDelta = "$aws/things/ESP32/shadow/update/delta";


void conectarAWS() {
    net.setCACert(ca_cert);
    net.setCertificate(client_cert);
    net.setPrivateKey(client_key);

    client.setServer(aws_endpoint, aws_port);
    client.setCallback(callbackMQTT);

    Serial.print("Conectando a AWS IoT...");
    while (!client.connected()) {
        if (client.connect(thingName)) {
            Serial.println("¡Conectado!");
            client.subscribe(shadowDelta.c_str());
        } else {
            Serial.print(".");
            delay(1000);
        }
    }
}
void callbackMQTT(char* topic, byte* payload, unsigned int length) {
    StaticJsonDocument<512> doc;
    DeserializationError error = deserializeJson(doc, payload, length);

    if (error) {
        Serial.println("Error al parsear JSON de delta");
        return;
    }


    if (doc["state"]["servoState"].is<const char*>()) {
        String accion = doc["state"]["servoState"].as<String>();

        if (accion == "abrir") {
            servo.open();
            Serial.println("Servo abierto");
        } else if (accion == "cerrar") {
            servo.close();
            Serial.println("Servo cerrado");
        }


        StaticJsonDocument<256> report;
        report["state"]["reported"]["servoState"] = accion;
        String jsonStr;
        serializeJson(report, jsonStr);
        client.publish(shadowUpdate.c_str(), jsonStr.c_str());
    }
}


void setup() {
    Serial.begin(115200);
    delay(100);

    WiFi.begin(ssid, password);
    Serial.print("Conectando a WiFi...");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println(" conectado.");

    conectarAWS();
}

void loop() {
    if (!client.connected()) {
        conectarAWS();
    }

    float distancia = sensor.distancia();
    Serial.print("Distancia: ");
    Serial.print(distancia);
    Serial.println(" cm");


    StaticJsonDocument<256> doc;
    doc["state"]["reported"]["distance"] = distancia;
    String payload;
    serializeJson(doc, payload);
    client.publish(shadowUpdate.c_str(), payload.c_str());

    client.loop();
    delay(5000);  
}