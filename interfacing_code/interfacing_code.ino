#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <MQUnifiedsensor.h>
#include <SoftwareSerial.h>
SoftwareSerial pmsSerial(2, 3);

#define DHTPIN 7    
#define DHTTYPE    DHT11     // DHT 11
DHT_Unified dht(DHTPIN, DHTTYPE);

uint32_t delayMS;
#define placa "Arduino UNO"
#define Voltage_Resolution 5
#define pin A0 
#define type "MQ-135" //MQ135
#define ADC_Bit_Resolution 10 
#define RatioMQ135CleanAir 3.6 
MQUnifiedsensor MQ135(placa, Voltage_Resolution, ADC_Bit_Resolution, pin, type);

#define pin1 A1 //Analog input 0 of your arduino
#define type1 "MQ-7" //MQ7
#define RatioMQ7CleanAir 27.5 //RS / R0 = 27.5 ppm 
MQUnifiedsensor MQ7(placa, Voltage_Resolution, ADC_Bit_Resolution, pin1, type1);

void setup() {
  Serial.begin(9600);
  pmsSerial.begin(9600);
  dht.begin();
  sensor_t sensor;
  dht.temperature().getSensor(&sensor);
  dht.humidity().getSensor(&sensor);
  delayMS = 300000;

  MQ135.setRegressionMethod(1); //_PPM =  a*ratio^b
  MQ135.init(); 
  float calcR0 = 0;
  
  
  for(int i = 1; i<=10; i ++)
  {
    MQ135.update();
    calcR0 += MQ135.calibrate(RatioMQ135CleanAir);
  }
  MQ135.setR0(calcR0/10);

  MQ7.setRegressionMethod(1); //_PPM =  a*ratio^b
  MQ7.setA(99.042); MQ7.setB(-1.518);
  MQ7.init(); 
  calcR0 = 0;
  for(int i = 1; i<=10; i ++)
  {
    MQ7.update(); // Update data, the arduino will be read the voltage on the analog pin
    calcR0 += MQ7.calibrate(RatioMQ7CleanAir);
  }
  MQ7.setR0(calcR0/10);



  
   Serial.println("Temp,Humidity,CO,CO2,NOx,PM1,PM2.5,PM10");   

}

struct pms5003data {
  uint16_t framelen;
  uint16_t pm10_standard, pm25_standard, pm100_standard;
  uint16_t pm10_env, pm25_env, pm100_env;
  uint16_t particles_03um, particles_05um, particles_10um, particles_25um, particles_50um, particles_100um;
  uint16_t unused;
  uint16_t checksum;
};
 
struct pms5003data data;

  
void loop() {
  delay(delayMS);
  sensors_event_t event;
  dht.temperature().getEvent(&event);
 // Serial.print(event.temperature);
  float temp = event.temperature;
  dht.humidity().getEvent(&event);
 // Serial.print(",");
 // Serial.print(event.relative_humidity);
  float humi= event.relative_humidity;
  MQ135.update();
  MQ135.setA(102.2 ); MQ135.setB(-2.473); 
  float NOx = MQ135.readSensor();

  MQ135.setA(110.47); MQ135.setB(-2.862);
  float CO2 = MQ135.readSensor();

  MQ7.update();
  MQ7.setA(99.042); MQ135.setB(-1.518);
  float CO = MQ7.readSensor();
 readPMSdata(&pmsSerial);
  //if (readPMSdata(&pmsSerial)) {
     Serial.print(temp); 
     Serial.print(",");
     Serial.print(humi); 
     Serial.print(",");
     Serial.print(CO); 
     Serial.print(",");
     Serial.print(CO2); 
     Serial.print(",");
     Serial.print(NOx); 
     Serial.print(",");
     Serial.print(data.pm10_env);
     Serial.print(",");
     Serial.print(data.pm25_env);
     Serial.print(",");
     Serial.print(data.pm100_env);
  
  //}
  Serial.println("");
  
  
}
  

 
boolean readPMSdata(Stream *s) {
  if (! s->available()) {
    return false;
  }
  
  // Read a byte at a time until we get to the special '0x42' start-byte
  if (s->peek() != 0x42) {
    s->read();
    return false;
  }
 
  // Now read all 32 bytes
  if (s->available() < 32) {
    return false;
  }
    
  uint8_t buffer[32];    
  uint16_t sum = 0;
  s->readBytes(buffer, 32);
 
  // get checksum ready
  for (uint8_t i=0; i<30; i++) {
    sum += buffer[i];
  }
 
  /* debugging
  for (uint8_t i=2; i<32; i++) {
    Serial.print("0x"); Serial.print(buffer[i], HEX); Serial.print(", ");
  }
  Serial.println();
  */
  
  // The data comes in endian'd, this solves it so it works on all platforms
  uint16_t buffer_u16[15];
  for (uint8_t i=0; i<15; i++) {
    buffer_u16[i] = buffer[2 + i*2 + 1];
    buffer_u16[i] += (buffer[2 + i*2] << 8);
  }
 
  // put it into a nice struct :)
  memcpy((void *)&data, (void *)buffer_u16, 30);
 
 // if (sum != data.checksum) {
   // Serial.println("Checksum failure");
   // return false;
  //}
  // success!
  return true;
}
