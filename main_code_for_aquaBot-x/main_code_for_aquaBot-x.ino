/*------------------------------------------------------------------------------------------------------------------
   Project: AquaBot + Line Following Robot + Firebase Realtime Monitoring
   Board  : ESP32
   Author : Mahmudul Hasan
   Note   : Non-blocking version, delay removed. Original behavior unchanged.
-------------------------------------------------------------------------------------------------------------------*/

// ----- Feature Enable Flags -----
#define ENABLE_USER_AUTH
#define ENABLE_DATABASE

// ----- Libraries -----
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <FirebaseClient.h>
#include "DHT.h"
#include <ESP32Servo.h>

/*-------------------------------- Pin Configuration & Global Variables --------------------------------*/

// ----- DHT11 Sensor -----
#define DHTPIN 4              
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

// ----- Soil & Water Sensors -----
#define soilSensor 32         
#define water_sensor 34       

// ----- Voltage Sensor -----
#define voltagePin 35         
float R1 = 30000.0;           
float R2 = 7500.0;            

// ----- Timers -----
unsigned long previousMillis = 0;    
const long interval = 5000;          
unsigned long lastMarkMillis = 0;    
const long debounceDelay = 150;      

// ----- IR Sensor (Black mark counter) -----
#define IRSENSOR_PIN 27       
int markCount = 0;
bool lastState = HIGH;        

// ----- Pump & UV LED -----
#define AWG 2 
int pump = 25;
int uv_led = 19;

// ----- Tree count (from Firebase)-----
int treeCount = -1; 

/*-------------------------------- Robot Section --------------------------------*/

// ----- Line Following IR Sensors -----
int irLeft   = 13;
int irCenter = 12;
int irRight  = 11;

// ----- Motors (PWM pins) -----
int lmotorf = 9;   
int lmotorb = 6;   
int rmotorf = 5;   
int rmotorb = 3;   

// ----- Motor Speed -----
int fullSpeed = 200;  
int slowSpeed = 120;  

// ----- Ultrasonic Sensor -----
int trigPin = 8;
int echoPin = 7;

// ----- Servo Motors -----
Servo myServo;             
Servo s1, s2, s3, s4;      

int ulraservoPin = 10;
int s1Pin = 16;
int s2Pin = 17;
int s3Pin = 18;
int s4Pin = 23;

// ----- Safe Distance -----
int safeDistance = 15; // cm 

// -------robot status from webdashboard using firebase---------
int robotStatus = 0;   // 0 = Sleep, 1 = Active

/*-------------------------------- Firebase Configuration --------------------------------*/

void asyncCB(AsyncResult &aResult);
void processData(AsyncResult &aResult);

UserAuth user_auth("AIzaSyDqU-w8lG5XBNcje1LcGv3k9gQL44UsGDw",
                   "mdmahmudulhasan1511@gmail.com",
                   "mahmudul@robo1511");

FirebaseApp app;

WiFiClientSecure ssl_client1, ssl_client2;

using AsyncClient = AsyncClientClass;
AsyncClient async_client1(ssl_client1), async_client2(ssl_client2);

RealtimeDatabase Database;

bool onetimeTest = false;
AsyncResult dbResult;

/*-------------------------------- PWM (ESP32 ledc) setup --------------------------------*/
const int CH_LMOTORF = 0;
const int CH_LMOTORB = 1;
const int CH_RMOTORF = 2;
const int CH_RMOTORB = 3;

void analogWritePWM(int pin, int value) {
    if(value < 0) value = 0;
    if(value > 255) value = 255;

    if(pin == lmotorf) ledcWrite(CH_LMOTORF, value);
    else if(pin == lmotorb) ledcWrite(CH_LMOTORB, value);
    else if(pin == rmotorf) ledcWrite(CH_RMOTORF, value);
    else if(pin == rmotorb) ledcWrite(CH_RMOTORB, value);
}

/*-------------------------------- Non-blocking Servo & Pump --------------------------------*/
struct ServoAction {
    Servo* servo;
    int targetPos;
    unsigned long duration;
    unsigned long startMillis;
    bool active;
};

ServoAction sActions[4];

void startServoAction(Servo* s, int pos, unsigned long durationMs) {
    for(int i=0; i<4; i++){
        if(!sActions[i].active){
            sActions[i].servo = s;
            sActions[i].targetPos = pos;
            sActions[i].duration = durationMs;
            sActions[i].startMillis = millis();
            sActions[i].active = true;
            return;
        }
    }
}

void updateServos(){
    unsigned long currentMillis = millis();
    for(int i=0; i<4; i++){
        if(sActions[i].active){
            sActions[i].servo->write(sActions[i].targetPos);
            if(currentMillis - sActions[i].startMillis >= sActions[i].duration){
                sActions[i].active = false;
            }
        }
    }
}

unsigned long lastPumpMillis = 0;
unsigned long pumpDuration = 1000; // 1 sec
bool pumpOn = false;

void updatePump(){
    unsigned long currentMillis = millis();
    if(pumpOn && currentMillis - lastPumpMillis >= pumpDuration){
        digitalWrite(pump, LOW);
        pumpOn = false;
    }
}

/*-------------------------------- Setup Function --------------------------------*/
void setup()
{
    Serial.begin(115200);
    dht.begin();

    pinMode(IRSENSOR_PIN, INPUT);

    pinMode(lmotorf, OUTPUT);
    pinMode(lmotorb, OUTPUT);
    pinMode(rmotorf, OUTPUT);
    pinMode(rmotorb, OUTPUT);

    // ----- ESP32 LEDC PWM Setup (3.3.0 compatible) -----
    ledcAttach(lmotorf, 5000, 8);
    ledcAttach(lmotorb, 5000, 8);
    ledcAttach(rmotorf, 5000, 8);
    ledcAttach(rmotorb, 5000, 8);

    pinMode(irLeft, INPUT);
    pinMode(irCenter, INPUT);
    pinMode(irRight, INPUT);

    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);

    myServo.attach(ulraservoPin);
    myServo.write(90); 

    s1.attach(s1Pin);
    s2.attach(s2Pin);
    s3.attach(s3Pin);
    s4.attach(s4Pin);

    pinMode(pump, OUTPUT);
    pinMode(uv_led, OUTPUT);
    digitalWrite(pump, LOW);
    digitalWrite(uv_led, LOW);

    WiFi.begin("mahmudul", "@mahmuduL");
    Serial.print("Connecting to Wi-Fi");
    while (WiFi.status() != WL_CONNECTED)
    {
        Serial.print(".");
        delay(300);
    }
    Serial.println();
    Serial.print("Connected with IP: ");
    Serial.println(WiFi.localIP());
    Serial.println();

    ssl_client1.setInsecure();
    ssl_client2.setInsecure();
    ssl_client1.setConnectionTimeout(1000);
    ssl_client1.setHandshakeTimeout(5);
    ssl_client2.setConnectionTimeout(1000);
    ssl_client2.setHandshakeTimeout(5);

    initializeApp(async_client1, app, getAuth(user_auth), processData, "🔐 authTask");
    app.getApp<RealtimeDatabase>(Database);
    Database.url("https://aquabot-database-default-rtdb.asia-southeast1.firebasedatabase.app/");
}

/*-------------------------------- Main Loop --------------------------------*/
void loop()
{
    AsyncResult treeResult;
    AsyncResult statusResult;

    // TreeCount পড়া
    Database.get(async_client1, "/garden/treeCount", treeResult);
    if (treeResult.available()) {
        const char* payload = treeResult.c_str();
        if(payload) {
            treeCount = atoi(payload);
            Serial.print("TreeCount = "); Serial.println(treeCount);
        }
    }

    // Robot Status পড়া
    Database.get(async_client1, "/control/robotStatus", statusResult);
    if (statusResult.available()) {
        const char* payload = statusResult.c_str();
        if(payload) {
            int val = atoi(payload);
            if(val == 0 || val == 1){
                robotStatus = val;
                Serial.print("Robot Status = "); Serial.println(robotStatus);
            }
        }
    }

    //robot status based condition
    if (robotStatus == 0) {
        stopMotors();
        digitalWrite(pump, LOW);
        Serial.println("Robot Sleeping... Waiting for status=1");

        app.loop();   // Firebase sync
        delay(50);    // minimal delay
        return;
    }

    int irValue = digitalRead(IRSENSOR_PIN);
    unsigned long currentMillis = millis();

    if (lastState == HIGH && irValue == LOW) {
        if (currentMillis - lastMarkMillis > debounceDelay) {
            markCount++;
            Serial.print("Black mark detected! Count = ");
            Serial.println(markCount);
            Database.set<int>(async_client1, "/irSensor/markCount", markCount, dbResult);
            lastMarkMillis = currentMillis;
        }
    }
    lastState = irValue;

    float temperature = dht.readTemperature();
    float humidity    = dht.readHumidity();
    if (isnan(temperature) || isnan(humidity)) {
        Serial.println("Failed to read from DHT sensor!");
        return;
    }

    int waterLevel   = analogRead(water_sensor);
    int waterPercent = map(waterLevel, 0, 4095, 0, 100);

    int sensorValue      = analogRead(soilSensor);
    int moisturePercent  = map(sensorValue, 4095, 0, 0, 100);

    int frontDist = getDistance(); 
    if (frontDist < safeDistance && frontDist > 0) {
        avoidObstacle();
    } else {
        int leftVal   = digitalRead(irLeft);
        int centerVal = digitalRead(irCenter);
        int rightVal  = digitalRead(irRight);

        Serial.print("Left: "); Serial.print(leftVal);
        Serial.print(" | Center: "); Serial.print(centerVal);
        Serial.print(" | Right: "); Serial.println(rightVal);

        if(centerVal == HIGH && leftVal == LOW && rightVal == LOW) {
            if(irValue == HIGH) {
                stopMotors();
                Serial.println(" -> Mark detected");

                startServoAction(&s1, 90, 100);
                startServoAction(&s2, 40, 100);
                startServoAction(&s3, 50, 100);
                startServoAction(&s4, 30, 100);

                if(moisturePercent <= 50){
                    digitalWrite(pump, HIGH);
                    pumpOn = true;
                    lastPumpMillis = millis();
                } else {
                    startServoAction(&s1, 20, 100);
                    startServoAction(&s2, 20, 100);
                    startServoAction(&s3, 20, 100);
                    startServoAction(&s4, 20, 100);
                    Serial.print(" -> Has moisture. Going forward!");
                    forward(fullSpeed, fullSpeed);
                }
            } else {
                forward(fullSpeed, fullSpeed);
                Serial.println(" -> Forward");
            }
        }
        else if(leftVal == HIGH && centerVal == HIGH && rightVal == LOW) {
            forward(slowSpeed, fullSpeed);
            Serial.println(" -> Slight Left");
        }
        else if(rightVal == HIGH && centerVal == HIGH && leftVal == LOW) {
            forward(fullSpeed, slowSpeed);
            Serial.println(" -> Slight Right");
        }
        else if(leftVal == HIGH && centerVal == LOW && rightVal == LOW) {
            forward(0, fullSpeed);
            Serial.println(" -> Hard Left");
        }
        else if(rightVal == HIGH && centerVal == LOW && leftVal == LOW) {
            forward(fullSpeed, 0);
            Serial.println(" -> Hard Right");
        }
        else if(markCount == treeCount && treeCount >= 0) {
            stopMotors();
            Serial.println(" -> sleeping.... (markCount == treeCount)");
        }
        else {
            stopMotors();
            Serial.println(" -> Stop / Line Lost");
        }
    }

    // Update non-blocking operations
    updateServos();
    updatePump();

    // Firebase loop
    app.loop();

    if (app.ready() && (currentMillis - previousMillis >= interval)) {
        previousMillis = currentMillis;

        int adcValue = analogRead(voltagePin);
        float voltage = ((float)adcValue / 4095.0) * 3.3 * ((R1 + R2) / R2);
        int voltagePercent = map((int)(voltage*100), 0, 1200, 0, 100);

        Serial.printf("Temp: %.1f, Humidity: %.1f, Water: %d%%, Moisture: %d%%, Charge: %d%%\n",
                      temperature, humidity, waterPercent, moisturePercent, voltagePercent);

        Database.set<float>(async_client1, "/dht11/temperature", temperature, dbResult);
        Database.set<float>(async_client1, "/dht11/humidity", humidity, dbResult);
        Database.set<int>(async_client1, "/waterSensor/percent", waterPercent, dbResult);
        Database.set<int>(async_client1, "/moistureSensor/percent", moisturePercent, dbResult);
        Database.set<int>(async_client1, "/voltageSensor/charge", voltagePercent, dbResult);

        processData(dbResult);
    }

    if (temperature >= 30 && humidity >= 65 && waterPercent <= 90) {
        digitalWrite(AWG, HIGH);
        if(waterPercent > 5) digitalWrite(uv_led, HIGH);
        else digitalWrite(uv_led, LOW);
    } else {
        digitalWrite(AWG, LOW);
    }
}

/*-------------------------------- Firebase Data Processing --------------------------------*/
void processData(AsyncResult &aResult)
{
    if (!aResult.isResult()) return;

    if (aResult.isEvent())
        Firebase.printf("Event task: %s, msg: %s, code: %d\n",
                         aResult.uid().c_str(), aResult.eventLog().message().c_str(), aResult.eventLog().code());

    if (aResult.isDebug())
        Firebase.printf("Debug task: %s, msg: %s\n",
                         aResult.uid().c_str(), aResult.debug().c_str());

    if (aResult.isError())
        Firebase.printf("Error task: %s, msg: %s, code: %d\n",
                         aResult.uid().c_str(), aResult.error().message().c_str(), aResult.error().code());

    if (aResult.available()) {
        Firebase.printf("task: %s, payload: %s\n", aResult.uid().c_str(), aResult.c_str());

        const char* payload = aResult.c_str();
        if (payload != nullptr) {
            int val = atoi(payload);
            if (val != 0 || (payload[0] == '0' && payload[1] == '\0')) {
                treeCount = val;
                Serial.print("Parsed treeCount from DB: ");
                Serial.println(treeCount);
            }
        }
    }
}

/*-------------------------------- Movement Functions --------------------------------*/
void forward(int leftSpeed, int rightSpeed) {
    analogWritePWM(lmotorf, leftSpeed);
    digitalWrite(lmotorb, LOW);
    analogWritePWM(rmotorf, rightSpeed);
    digitalWrite(rmotorb, LOW);
}

void stopMotors() {
    analogWritePWM(lmotorf, 0);
    analogWritePWM(rmotorf, 0);
    analogWritePWM(lmotorb, 0);
    analogWritePWM(rmotorb, 0);
    digitalWrite(lmotorb, LOW);
    digitalWrite(rmotorb, LOW);
}

void backward(int speedVal) {
    analogWritePWM(lmotorb, speedVal);
    digitalWrite(lmotorf, LOW);
    analogWritePWM(rmotorb, speedVal);
    digitalWrite(rmotorf, LOW);
}

void turnLeft() {
    analogWritePWM(lmotorf, 0);
    analogWritePWM(rmotorf, fullSpeed);
    digitalWrite(lmotorb, LOW);
    digitalWrite(rmotorb, LOW);
}

void turnRight() {
    analogWritePWM(rmotorf, 0);
    analogWritePWM(lmotorf, fullSpeed);
    digitalWrite(lmotorb, LOW);
    digitalWrite(rmotorb, LOW);
}

/*-------------------------------- Ultrasonic Distance --------------------------------*/
int getDistance() {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    unsigned long duration = pulseIn(echoPin, HIGH, 30000UL);
    if (duration == 0) return -1;
    int distance = (int)(duration * 0.034 / 2);
    return distance;
}

/*-------------------------------- Obstacle Avoiding --------------------------------*/
void avoidObstacle() {
    stopMotors();

    myServo.write(150);
    int leftDist = getDistance();

    myServo.write(30);
    int rightDist = getDistance();

    myServo.write(90);

    if (leftDist < 0 && rightDist < 0) {
        backward(180);
        turnRight();
        forward(fullSpeed, fullSpeed);
    } else if (leftDist > rightDist) {
        Serial.println("Obstacle! Turning LEFT");
        turnLeft();
        forward(fullSpeed, fullSpeed);
    } else {
        Serial.println("Obstacle! Turning RIGHT");
        turnRight();
        forward(fullSpeed, fullSpeed);
    }

    stopMotors();
}
