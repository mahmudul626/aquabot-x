#define ENABLE_USER_AUTH
#define ENABLE_DATABASE

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <FirebaseClient.h>
#include "DHT.h"
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 16, 2);

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

// ----- IR Sensor (Black mark counter) -----
#define IRSENSOR_PIN 27       
int markCount = 0;
bool lastState = HIGH;
unsigned long lastMarkMillis = 0;     // শেষ mark detection এর সময়
const unsigned long debounceDelay = 100; // debounce delay (100 ms)


// Step 2
void asyncCB(AsyncResult &aResult);
void processData(AsyncResult &aResult);

// Step 3
UserAuth user_auth("AIzaSyDqU-w8lG5XBNcje1LcGv3k9gQL44UsGDw", "mdmahmudulhasan1511@gmail.com", "mahmudul@robo1511");

// Step 4
FirebaseApp app;

// Step 5
WiFiClientSecure ssl_client1, ssl_client2;

// Step 6
using AsyncClient = AsyncClientClass;
AsyncClient async_client1(ssl_client1), async_client2(ssl_client2);

// Step 7
RealtimeDatabase Database;

bool onetimeTest = false;
AsyncResult dbResult;

// Non-blocking timer variables
unsigned long previousMillis = 0;
const unsigned long interval = 7000; // 10 seconds

void setup()
{
    pinMode(IRSENSOR_PIN, INPUT);

    Serial.begin(115200);
    dht.begin();

    lcd.init();
    lcd.backlight();

    WiFi.begin("UIU-STUDENT", "12345678");
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

    // Step 8
    initializeApp(async_client1, app, getAuth(user_auth), processData, "🔐 authTask");

    // Step 9
    app.getApp<RealtimeDatabase>(Database);

    // Step 10
    Database.url("https://aquabot-database-default-rtdb.asia-southeast1.firebasedatabase.app/");
}

void loop()
{
    app.loop();

    int irValue = digitalRead(IRSENSOR_PIN);
    unsigned long currentMillisir = millis();

    if (lastState == HIGH && irValue == LOW) {
        if (currentMillisir - lastMarkMillis > debounceDelay) {
            markCount++;
            Serial.print("Black mark detected! Count = ");
            Serial.println(markCount);
            Database.set<int>(async_client1, "/irSensor/markCount", markCount, dbResult);
            lastMarkMillis = currentMillisir;
        }
    }
    lastState = irValue;

    unsigned long currentMillis = millis();
    if (app.ready() && (currentMillis - previousMillis >= interval))
    {
        previousMillis = currentMillis;

        // Read DHT11 data
        float temperature = dht.readTemperature();
        float humidity = dht.readHumidity();

        if (isnan(temperature) || isnan(humidity))
        {
            Serial.println("Failed to read from DHT sensor!");
            return;
        }

        int waterLevel   = analogRead(water_sensor);
        int waterPercent = map(waterLevel, 0, 4095, 0, 100);

        int sensorValue      = analogRead(soilSensor);
        int moisturePercent  = map(sensorValue, 4095, 0, 0, 100);

        int adcValue = analogRead(voltagePin);
        float voltage = ((float)adcValue / 4095.0) * 3.3 * ((R1 + R2) / R2);
        int voltagePercent = map((int)(voltage*100), 0, 1200, 0, 100);

        lcd.setCursor(0, 0);  
        lcd.print("T:");  
        lcd.print(temperature);  
        lcd.print("C ");  
        lcd.print("H:");  
        lcd.print(humidity);  
        lcd.print("%");

        lcd.setCursor(0, 1);  
        lcd.print("W:");  
        lcd.print(waterPercent);  
        lcd.print("% S:");  
        lcd.print(moisturePercent);  
        lcd.print("%");
        lcd.print("C:");
        lcd.print(voltagePercent);



        // Async update to Firebase
        Database.set<float>(async_client1, "/dht11/temperature", temperature, dbResult);
        Database.set<float>(async_client1, "/dht11/humidity", humidity, dbResult);
        Database.set<float>(async_client1, "/moistureSensor/percent", moisturePercent, dbResult);
        Database.set<float>(async_client1, "/waterSensor/percent", waterPercent, dbResult);
        Database.set<float>(async_client1, "/voltageSensor/Charge", voltagePercent, dbResult);

        // Process async results
        processData(dbResult);
    }
}

void processData(AsyncResult &aResult)
{
    if (!aResult.isResult())
        return;

    if (aResult.isEvent())
        Firebase.printf("Event task: %s, msg: %s, code: %d\n", aResult.uid().c_str(), aResult.eventLog().message().c_str(), aResult.eventLog().code());

    if (aResult.isDebug())
        Firebase.printf("Debug task: %s, msg: %s\n", aResult.uid().c_str(), aResult.debug().c_str());

    if (aResult.isError())
        Firebase.printf("Error task: %s, msg: %s, code: %d\n", aResult.uid().c_str(), aResult.error().message().c_str(), aResult.error().code());

    if (aResult.available())
        Firebase.printf("task: %s, payload: %s\n", aResult.uid().c_str(), aResult.c_str());
}
