/*
 *
 * vim: sta:et:sw=4:ts=4:sts=4
 * https://arduinojson.org/v6/how-to/use-arduinojson-with-arduinomqttclient/
 * https://github.com/arduino-libraries/ArduinoMqttClient/
 *
 * https://github.com/maxgerhardt/platform-raspberrypi
 * https://github.com/earlephilhower/arduino-pico
 * https://components101.com/development-boards/raspberry-pi-pico-w
 *******************************************************************************/

#include <Arduino.h>
#include <WiFi.h>
#include <time.h>

#include <ArduinoMqttClient.h>
#include <ArduinoJson.h>
#include <ArduinoOTA.h>
#include <EEPROM.h>

//#include <PicoOTA.h>
#include <LittleFS.h>
#include "hardware/gpio.h"

#ifndef SECRET_SSID
#include "arduino_secrets.h"
#endif

const int waterAutoOn = 60UL; // in sec. Auto water/ day

const char *ssid = SECRET_SSID;
const char *password = SECRET_PASS;
#define PIN_D21              21
#define PS_PIN 23 // Power Save Pin, H to disable, L default

const int SENSE_PIN = A0; // PICO pin 31, Measure external Voltage
const int AGND_PIN = A1; // PICO pin 32, Measure analog ground level
const int relayPin = 22; // PICO pin 29
unsigned int irqPin = PIN_D21;

const int msgPeriod = 1 * 1000U;
const int wifiPeriod = 300 * 1000U;
int ledPeriod = 2 * 1000UL;

const char* mqttBroker = "192.168.168.112";
const int mqtt_port = 1883;  // Sets the server details.
// const char* mqtt_server = "91.121.93.94";
const char* ntpServer = "ntp1.tecnico.ulisboa.pt";
// int        port        = 1883;
const char willTopic[] = "iot/voltagesensor/will";
const char inTopic[]   = "iot/voltagesensor/in";
const char outTopicVoltage[]  = "iot/weidemann/voltage";
const char outTopicTemperature[]  = "iot/weidemann/cputemp";
const char outTopicRSSI[]  = "iot/weidemann/rssi";
String willPayload = "oh no!, weidemann lost Mqtt Connection";
const bool willRetain = true;
const int willQos = 1;
const int subscribeQos = 1;

// By default 'pool.ntp.org' is used with 60 seconds update interval and
WiFiClient wClient;
MqttClient mqttClient(wClient);

bool ledState = false;
bool led_blink = false;
// int count = 0;
int irqCount, irqNext = 0;

#define WIFI_RETRY 20
#define MSG_BUFFER_SIZE 100

int wifiOK = 0;
int wifiRetries = 0 ;
bool ntpOK = false;
bool relayState = false;

void onMqttMessage(int messageSize);

int setupMqtt(){
    mqttClient.stop();
    delay(20);
    mqttClient.beginWill(willTopic, willPayload.length(), willRetain, willQos);
    mqttClient.print(willPayload);
    mqttClient.endWill();

    Serial.print("Attempting to connect to the MQTT broker: ");
    Serial.println(mqttBroker);

    if (!mqttClient.connect(mqttBroker, mqtt_port)) {
        Serial.print("MQTT connection failed! Error code = ");
        Serial.println(mqttClient.connectError());
        return -1;
    }
    else
        Serial.println("You're connected to the MQTT broker!");

    // set the message receive callback
    mqttClient.onMessage(onMqttMessage);

    Serial.print("Subscribing to topic: ");
    Serial.println(inTopic);
    //Serial.println();

    // subscribe to a topic
    // the second parameter sets the QoS of the subscription,
    // the the library supports subscribing at QoS 0, 1, or 2

    mqttClient.subscribe(inTopic, subscribeQos);
    // topics can be unsubscribed using:
    // mqttClient.unsubscribe(inTopic);
    return 0;
}
void reconnectWifi() {
    delay(10);
    if (WiFi.status() == WL_CONNECTED && mqttClient.connected() == 1)
        return;
    Serial.print("No link. Wifi "); Serial.print(WiFi.status());
    Serial.print(" MQTT "); Serial.println(mqttClient.connected());
    mqttClient.unsubscribe(inTopic);
    if(wifiRetries++ > WIFI_RETRY){
        Serial.println("Rebooting....");
        rp2040.reboot();
    }
    wifiOK = 0;
    // We start by connecting to a WiFi network
    Serial.println();
    Serial.print("Re-connecting to ");
    Serial.println(ssid);

    WiFi.disconnect();
    WiFi.begin(ssid, password);

    led_blink = true;
    ledPeriod = 500UL;
    for(int i = 0; i < WIFI_RETRY; i++){
        if (WiFi.status() != WL_CONNECTED) {
            Serial.print(".");
            ledState = false;
            digitalWrite(LED_BUILTIN, ledState);
            delay(100);
            ledState = true;
            digitalWrite(LED_BUILTIN, ledState);
            delay(500);
            //continue;
        }
        else {
            wifiOK = 1;
            ledState = true;
            //led_blink = true;
            ledPeriod = 2000UL;
            Serial.println("");
            Serial.print("WiFi connected, IP address: ");
            Serial.println(WiFi.localIP());
            digitalWrite(LED_BUILTIN, ledState);
            if(setupMqtt() == 0)
                wifiRetries = 0;
            break;
        }
    }
}

void setClockNtp(uint32_t timeout) {
    NTP.begin(ntpServer, "pool.ntp.org");
    NTP.waitSet(timeout);
    time_t now = time(nullptr);
    struct tm timeinfo;
    if(now < 8 * 3600 * 2){
        Serial.println("Could not get ntp time");
    }
    else{
        gmtime_r(&now, &timeinfo);
        Serial.print("Current ntp time: ");
        Serial.print(asctime(&timeinfo));
    }
}

void onMqttMessage(int messageSize) {
    StaticJsonDocument<256> doc;

    // we received a message, print out the topic and contents
    Serial.print("Received a message with topic '");
    Serial.print(mqttClient.messageTopic());
    Serial.print("', duplicate = ");
    Serial.print(mqttClient.messageDup() ? "true" : "false");
    Serial.print(", QoS = ");
    Serial.print(mqttClient.messageQoS());
    Serial.print(", retained = ");
    Serial.print(mqttClient.messageRetain() ? "true" : "false");
    Serial.print("', length ");
    Serial.print(messageSize);
    //Serial.print(", msg:");

    deserializeJson(doc, mqttClient);

    //Serial.print("Msg: ");
    //Serial.println((char)mqttClient.read());
    int pump = doc["pump"];
    unsigned long now = millis();
    time_t nowTime = time(nullptr);
    unsigned long ntpTime = nowTime;
    if (pump == 1){
    }
    Serial.print("\"pump\":");
    Serial.println(pump);
    //Serial.println();

    // use the Stream interface to print the contents
    /*
       while (mqttClient.available()) {
       Serial.print((char)mqttClient.read());
       }
       */

}

void gpio_callback(uint gpio, uint32_t events) {
    // Put the GPIO event(s) that just happened into event_str
    // so we can print it
    //gpio_event_string(event_str, events);
    gpio_acknowledge_irq(irqPin, GPIO_IRQ_EDGE_FALL);
    irqCount++;
    gpio_set_irq_enabled(irqPin, GPIO_IRQ_EDGE_FALL, false);
    irqNext = millis() + 2000UL;  // Debounce Reed switch
}

// The normal, core0 setup
void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);
    pinMode(relayPin, OUTPUT);
    pinMode(irqPin, INPUT_PULLUP);
    analogReadResolution(12);

    relayState = false;
    digitalWrite(relayPin, relayState);

    Serial.begin(115200);
    delay(500);
    //while (!Serial);
    gpio_set_irq_enabled_with_callback(irqPin, GPIO_IRQ_EDGE_FALL, true, &gpio_callback);

    // Set in station mode
    WiFi.mode(WIFI_STA);
    reconnectWifi();

    setClockNtp(10000);
    time_t nowTime = time(nullptr);
    unsigned long ntpTime = nowTime;

    ArduinoOTA.setHostname("PicoW");  // Set the network port name.  设置网络端口名称
    ArduinoOTA.setPassword("666666");  // Set the network port connection
    ArduinoOTA.begin();            // Initialize the OTA.  初始化OTA
    Serial.println("OTA ready!");  // M5.lcd port output format str§
    delay(20);
}

void loop() {
    static  unsigned long lastMsg = 0;
    static  unsigned long lastWifiCheck = 0;
    static  unsigned long lastLed = 0;
    char msg[MSG_BUFFER_SIZE];

    StaticJsonDocument<256> doc;
    struct tm timeinfo;

    // call poll() regularly to allow the library to receive MQTT messages and
    // send MQTT keep alives which avoids being disconnected by the broker
    mqttClient.poll();

    unsigned long now = millis();
    time_t nowTime = time(nullptr);
    unsigned long ntpTime = nowTime;
    gmtime_r(&nowTime, &timeinfo);

    if (now - lastWifiCheck > wifiPeriod) {
        lastWifiCheck = now;
        reconnectWifi();
    }

    if(led_blink)
        if (now - lastLed > ledPeriod) {
            lastLed = now;
            ledState = not ledState;
            digitalWrite(LED_BUILTIN, ledState);
        }

    long rawADC_voltage, rawADC_agnd;
    if (now - lastMsg > msgPeriod) {
        lastMsg = now;

        //digitalWrite(PS_PIN, HIGH);
        const int AVERAGE_CNT = 100;
        int v=0l;
        int g=0l;
        for(int i=0; i<AVERAGE_CNT; i++){
            v += analogRead(SENSE_PIN);
            g += analogRead(AGND_PIN);
            delayMicroseconds(100);
        }
        rawADC_voltage = v/AVERAGE_CNT;
        rawADC_agnd = g/AVERAGE_CNT;
        //digitalWrite(PS_PIN, LOW);
        auto voltage = (rawADC_voltage - rawADC_agnd) * 0.000805664 * (10560.0/560.0) * 1.011739876; //substract ground offset, multiplicate with resolution and voltage divider ratio and calib factor
        auto rssi = WiFi.RSSI();
        auto coreTemp = analogReadTemp();

        snprintf(msg, MSG_BUFFER_SIZE, ", Voltage raw: %u, AGnd: %u, Voltage: %.3f, RSSI: %i, Temp: %.1f\n", rawADC_voltage, rawADC_agnd, voltage, rssi, coreTemp);
        Serial.print("Wifi:"); Serial.print(wifiOK);
        Serial.print(" mqttClient: "); Serial.print(mqttClient.connected());
        Serial.print(msg);

        bool retained = false;
        int qos = 1;

        mqttClient.beginMessage(outTopicVoltage, retained, qos);
        mqttClient.print(voltage, 3);
        mqttClient.endMessage();

        mqttClient.beginMessage(outTopicTemperature);
        mqttClient.print(coreTemp);
        mqttClient.endMessage();

        mqttClient.beginMessage(outTopicRSSI);
        mqttClient.print(rssi);
        mqttClient.endMessage();
    }

}

/*
// Running on core1
void setup1() {
delay(5000);
Serial.printf("C1: Red leader standing by...\n");
}
void loop1() {
static  unsigned long lastMsg = 0;
unsigned long now = millis();
//time_t t_now;
//char buff[80];


if (now - lastMsg > 5000) {
lastMsg = now;

//time(&t_now);
//strftime(buff, sizeof(buff), "%c", localtime(&t_now));

float coreTemp =analogReadTemp();
//Serial.printf("Core temperature %2.1fC ", coreTemp);
//Serial.println(buff);
snprintf(msg, MSG_BUFFER_SIZE, "Core temperature: %2.1fC", coreTemp);

PmqttClient.publish("ipfn/rega/temp", msg);
Serial.println(msg);

}
//delay(500);
}
*/
/*
   void callback(char* topic, byte* payload, unsigned int length) {
   StaticJsonDocument<256> doc;
   deserializeJson(doc, payload, length);
   Serial.print("Message arrived [");
   Serial.print(topic);
   Serial.print("] ");
   for (int i = 0; i < length; i++) {
   Serial.print((char)payload[i]);
   }
   ledState = not led_state;
   Serial.print(F(", LED:"));
   Serial.println(led_state);
   }

   void reConnect() {
   while (!PmqttClient.connected()) {
   Serial.print(F("Attempting MQTT connection..."));
// Create a random client ID.
String clientId = "ipfn/rega-";
clientId += String(random(0xffff), HEX);
// Attempt to connect.
if (PmqttClient.connect(clientId.c_str())) {
Serial.print("\nSuccess\n");
// Once connected, publish an announcement to the topic.
PmqttClient.publish("ipfn/rega/rp", "hello world");
// ... and resubscribe.
PmqttClient.subscribe("ipfn/rega/led");
} else {
Serial.print("failed, rc=");
Serial.print(PmqttClient.state());
Serial.println("try again in 5 seconds");
delay(5000);
}
}
}
*/
