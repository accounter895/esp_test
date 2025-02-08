// put function declarations here:
#include <Arduino.h>  // 平时写记得包含这个
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <MQ135.h>
#include <MQUnifiedsensor.h>

#define DHTPIN 2     // Digital pin connected to the DHT sensor 
// Feather HUZZAH ESP8266 note: use pins 3, 4, 5, 12, 13 or 14 --
// Pin 15 can work but DHT must be disconnected during program upload.

// Uncomment the type of sensor in use:
#define DHTTYPE    DHT11     // DHT 11
//#define DHTTYPE    DHT22     // DHT 22 (AM2302)
//#define DHTTYPE    DHT21     // DHT 21 (AM2301)
#define PIN_MQ135  A2
#define LED_Yellow 21
#define Key_1      48
#define Soil_val   A0

MQ135 mq135_sensor(PIN_MQ135);
DHT_Unified dht(DHTPIN, DHTTYPE);
float temperature = 21.0; // Assume current temperature. Recommended to measure with DHT22
float humidity = 25.0; // Assume current humidity. Recommended to measure with DHT22
uint32_t delayMS;
uint8_t LED_state = 0;
float Soil_wet = 0;

//  按键中断服务函数
void Key_state_Service(void) {
  LED_state = digitalRead(Key_1);
  if(LED_state){
    delay(5);
    digitalWrite(LED_Yellow, LED_state);
  } else {
    delay(5);
    digitalWrite(LED_Yellow, LED_state);
  }
}

// 外设初始化
void setup() {
  Serial.begin(115200);
  while (!Serial); // 拔掉数据线运行，这行记得注释掉，否则不连接串口，会卡在这里，后面代码就无法运行
  pinMode(LED_Yellow, OUTPUT);
  attachInterrupt(Key_1, Key_state_Service, CHANGE);    // 为按键引脚设置中断服务函数，在setup函数中调用  dht.begin();       // DHT11初始化 
  dht.begin();       // DHT11初始化 
  Serial.println(F("DHT11 Initialized Completed"));

  // 设置ADC分辨率位12位
  analogReadResolution(12);

  // Print temperature sensor details.
  sensor_t sensor;
  dht.temperature().getSensor(&sensor);
  dht.humidity().getSensor(&sensor);
  // Set delay between sensor readings based on sensor details.
  delayMS = sensor.min_delay / 1000;  //延迟1s
}

void loop() {
  delay(delayMS);
  // 获取温湿度并打印值
  sensors_event_t event;
  dht.temperature().getEvent(&event);   // 获取DHT11传感器标志位
  /*if (isnan(event.temperature)) {
    Serial.println(F("Error reading temperature!"));
  }
  else {
    Serial.print(F("Temperature: "));
    Serial.print(event.temperature);
    Serial.println(F("°C"));
  }
  dht.humidity().getEvent(&event);    // 获取湿度数据
  if (isnan(event.relative_humidity)) {
    Serial.println(F("Error reading humidity!"));
  }
  else {
    Serial.print(F("Humidity: "));
    Serial.print(event.relative_humidity);
    Serial.println(F("%"));
  }*/
  Soil_wet = analogRead(Soil_val);
  Serial.print("Soil_wet: ");
  Serial.println(Soil_wet);
  float rzero = mq135_sensor.getRZero();
  float correctedRZero = mq135_sensor.getCorrectedRZero(temperature, humidity);
  float resistance = mq135_sensor.getResistance();
  float ppm = mq135_sensor.getPPM();
  float correctedPPM = mq135_sensor.getCorrectedPPM(temperature, humidity);
  Serial.print("\t Corrected PPM: ");
  Serial.print(correctedPPM);
  Serial.println("ppm");
}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}