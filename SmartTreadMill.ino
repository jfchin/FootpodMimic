/* Based on code from https://github.com/imwitti/FootpodMimic
 * Modified by jfchin - https://github.com/jfchin/SmartTreadMill to fit my setup
*/

#include <Arduino.h>

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>

bool        is_inst_stride_len_present = 1;                                 /**< True if Instantaneous Stride Length is present in the measurement. */
bool        is_total_distance_present = 1;                                  /**< True if Total Distance is present in the measurement. */
bool        is_running = 1;                                                 /**< True if running, False if walking. */
uint16_t    inst_speed = 40;                                                 /**< Instantaneous Speed. */
uint8_t     inst_cadence = 1;                                               /**< Instantaneous Cadence. */
uint16_t    inst_stride_length = 1;                                        /**< Instantaneous Stride Length. */
uint32_t    total_distance = 0;
float       total_distance_float = 0;

//extra
unsigned long msec_between_steps = 0;
volatile static unsigned long last_interrupt_time = 0;
volatile static unsigned long last_step_time = 0;
volatile static unsigned long stepCount = 0;

int encoder = 2;
volatile unsigned int counter;
int rpm;

//extra
float distance_per_mark = 0.14;

float kmph;
float mps;

//extra
byte rscmArray[12] = {0b000001,10,10,10};

byte fakePos[1] = {1};

bool _BLEClientConnected = false;

#define RSCService BLEUUID((uint16_t)0x1814)
BLECharacteristic RSCMeasurementCharacteristics(BLEUUID((uint16_t)0x2A53), BLECharacteristic::PROPERTY_NOTIFY);
BLECharacteristic sensorPositionCharacteristic(BLEUUID((uint16_t)0x2A5D), BLECharacteristic::PROPERTY_READ);

//BLECharacteristic sensorFeatureCharacteristic(BLEUUID((uint16_t)0x2A54), BLECharacteristic::PROPERTY_READ);

BLEDescriptor RSCDescriptor(BLEUUID((uint16_t)0x2901));
BLEDescriptor sensorPositionDescriptor(BLEUUID((uint16_t)0x2901));

class MyServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      _BLEClientConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      _BLEClientConnected = false;
    }
};

void InitBLE() {
  //Device Name
  BLEDevice::init("Footpodmimic");
  // Create the BLE Server
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pRSC = pServer->createService(RSCService);

  pRSC->addCharacteristic(&RSCMeasurementCharacteristics);

  RSCDescriptor.setValue("Rate from 0 to 200");
  
  RSCMeasurementCharacteristics.addDescriptor(&RSCDescriptor);
  RSCMeasurementCharacteristics.addDescriptor(new BLE2902());

  pRSC->addCharacteristic(&sensorPositionCharacteristic);

  //extra
  sensorPositionDescriptor.setValue("Position 0 - 6");
  sensorPositionCharacteristic.addDescriptor(&sensorPositionDescriptor);

  pServer->getAdvertising()->addServiceUUID(RSCService);

  pRSC->start();
  
  // Start advertising
  pServer->getAdvertising()->start();
}

void poop() {
  //this gets called when the speed sensor rises
  counter++;
  Serial.print(counter);
}

void setup() {

  //some relationships
  //kmph = 8;
  //mps = kmph/3.6;

  
  
  Serial.begin(115200);
  //Serial.println("Start");
  Serial.print("Started");
  
  //Init speed sensor  
  pinMode(15, INPUT);
  
  attachInterrupt(digitalPinToInterrupt(15),poop,RISING);
  //Init BLE
  InitBLE();

}

void loop() {
  
  //Determine wheel revolutions
  static uint32_t previousMillis;
  if (millis() - previousMillis >= 1000) {
            rpm = (counter/20)*60;
            counter = 0;
            previousMillis += 1000;
  }

  //extra
  uint16_t msSinceLast = millis() - previousMillis;
  float sSinceLast = msSinceLast/1000;

  if (counter > 0) {
    is_running = 1;
  } else {
    is_running = 0;  
  }
  
  float distance_since_last = counter * distance_per_mark;
  total_distance_float = total_distance_float + distance_since_last;
  total_distance = (uint32_t) total_distance_float;
  mps = distance_since_last/sSinceLast;
  counter = 0;
  previousMillis = millis();
  //extra
  
  Serial.print("rpm: ");
  Serial.print(rpm);
  
  //Serial.print("Total: ");
  //Serial.print(total_distance);
  
  mps = (rpm * 0.21) / 60;
  
  Serial.print(", Speed(mps): ");
  Serial.print(mps);

  kmph = mps * 3600 / 1000;
  
  Serial.print(", Speed(kmph): ");
  Serial.print(kmph);

  inst_speed = (256 / 3600 / 1000) * kmph;
  
  Serial.print(", inst_speed: ");
  Serial.println(inst_speed);


  //Create the bytearray to send via BLE (Zwift)
  byte charArray[10] = {
      3,
      (unsigned byte)inst_speed, (unsigned byte)(inst_speed >> 8),
      (unsigned byte)inst_cadence,
      (unsigned byte)inst_stride_length, (unsigned byte)(inst_stride_length >> 8),
      (unsigned byte)total_distance, (unsigned byte)(total_distance >> 8), (unsigned byte)(total_distance >> 16), (unsigned byte)(total_distance >> 24)};

  RSCMeasurementCharacteristics.setValue(charArray,10);

  RSCMeasurementCharacteristics.notify();

  sensorPositionCharacteristic.setValue(fakePos, 1);

  delay(1000);
}
