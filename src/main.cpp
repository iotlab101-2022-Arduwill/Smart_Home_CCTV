#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <DHTesp.h>
#include <ConfigPortal8266.h>
#include <SSD1306.h>

char*               ssid_pfix = (char*)"MQTTSensor_JIHOON";
String              user_config_html = ""
    "<p><input type='text' name='mqttServer' placeholder='mqtt Broker'>";



DHTesp              dht;
unsigned long       interval = 2000;
unsigned long       lastDHTReadMillis = 0;
float               humidity = 0;
float               temperature = 0;
 
const char*         mqttServer;
const int           mqttPort = 1883;

unsigned long       pubInterval = 5000;
unsigned long       lastPublished = - pubInterval;

////////
const int trigPin = 13;
const int echoPin = 12;
long duration;
float distance;




// 엔코더에 대한 변수 선언
const int           pulseA = 12;
const int           pulseB = 13;
const int           pushSW = 2;
volatile int        lastEncoded = 0;
volatile long       encoderValue = 0;
int val = 0;




WiFiClient espClient;
PubSubClient client(espClient);
void readDHT22();
void callback(char* topic, byte* payload, unsigned int length);



SSD1306 display(0x3c, 4, 5, GEOMETRY_128_32);




IRAM_ATTR void handleRotary() {
    // Never put any long instruction
    int MSB = digitalRead(pulseA); //MSB = most significant bit
    int LSB = digitalRead(pulseB); //LSB = least significant bit

    int encoded = (MSB << 1) |LSB; //converting the 2 pin value to single number
    int sum  = (lastEncoded << 2) | encoded; //adding it to the previous encoded value
    if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue ++;
    if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue --;
    lastEncoded = encoded; //store this value for next time
    if (encoderValue > 1023) {
        encoderValue = 1023;
    } else if (encoderValue < 0 ) {
        encoderValue = 0;
    }
}

IRAM_ATTR void buttonClicked() {
    Serial.println("pushed");
}



void setup() {

    display.init();
    display.flipScreenVertically();
    display.drawString(10, 10, "hellow world");
    display.display();

    pinMode(pushSW, INPUT_PULLUP);
    pinMode(pulseA, INPUT_PULLUP);
    pinMode(pulseB, INPUT_PULLUP);
    attachInterrupt(pushSW, buttonClicked, FALLING);
    attachInterrupt(pulseA, handleRotary, CHANGE);
    attachInterrupt(pulseB, handleRotary, CHANGE);



    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);





    Serial.begin(115200);
    WiFi.mode(WIFI_STA); 
    loadConfig();
    // *** If no "config" is found or "config" is not "done", run configDevice ***
    if(!cfg.containsKey("config") || strcmp((const char*)cfg["config"], "done")) {
        configDevice();
    }
    WiFi.begin((const char*)cfg["ssid"], (const char*)cfg["w_pw"]);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    mqttServer = cfg["mqttServer"];
    client.setServer(mqttServer, mqttPort);
    client.setCallback(callback);
    while (!client.connected()) {
        Serial.println("Connecting to MQTT...");
 
        if (client.connect("hang_ESP8266Client")) {
            Serial.println("connected");  
        } else {
            Serial.print("failed with state "); Serial.println(client.state());
            delay(2000);
        }
    }
    dht.setup(14, DHTesp::DHT22); // Connect DHT sensor to GPIO 14
    client.subscribe("id/jihoon/sensor/cmd");
}

void loop() {
    client.loop();

    unsigned long currentMillis = millis();
    if(currentMillis - lastPublished >= pubInterval) {
        lastPublished = currentMillis;
        /*
        readDHT22();
        Serial.printf("%.1f\t %.1f\n", temperature, humidity);
        char buf[10];
        sprintf(buf, "%.1f", temperature);
        client.publish("id/jihoon/sensor/evt/temperature", buf);
        sprintf(buf, "%.1f", humidity);
        client.publish("id/jihoon/sensor/evt/humidity", buf);
        */
       
        /*
        int light = analogRead(0);
        sprintf(buf, "%d", light);
        client.publish("id/jihoon/sensor/evt/light", buf);
        */
        char buf[10];
        digitalWrite(trigPin, LOW);
        delayMicroseconds(2);
        digitalWrite(trigPin, HIGH);
        delayMicroseconds(10);
        digitalWrite(trigPin, LOW);

        duration = pulseIn(echoPin, HIGH);
        distance = duration * 0.017; // distance = duration / 29 / 2;
        sprintf(buf, "%f", distance);
        client.publish("id/jihoon/sensor/evt/distance", buf);

        Serial.printf("Duration = %6ld, Distance = %6.2fcm\n", duration, distance);
        //delay(1000);
    }


    
}



void readDHT22() {
    unsigned long currentMillis = millis();

    if(currentMillis - lastDHTReadMillis >= interval) {
        lastDHTReadMillis = currentMillis;

        humidity = dht.getHumidity();              // Read humidity (percent)
        temperature = dht.getTemperature();        // Read temperature as Fahrenheit

       



        display.clear();
    
        String buff1 = String(humidity);
        display.drawString(0, 0, buff1);
    
  
        String buff2 = String(temperature);
        display.drawString(0, 10, buff2);
    

    

        String buff3 = String(encoderValue);
        display.drawString(0, 20, buff3);


       


        display.display();





        
    }
}




void callback(char* topic, byte* payload, unsigned int length) {
 
    char msgBuffer[20];
    if(!strcmp(topic, "id/jihoon/relay/cmd")) {
        int i;
        for(i = 0; i < (int)length; i++) {
            msgBuffer[i] = payload[i];
        } 
        msgBuffer[i] = '\0';
        Serial.printf("\n%s -> %s", topic, msgBuffer);
        if(!strcmp(msgBuffer, "status")) {
            lastPublished -= pubInterval;
        }
    }
}