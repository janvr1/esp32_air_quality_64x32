#include <Arduino.h>
#include <esp32_ledmatrix64.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <NTPClient.h>
#include <TimeLib.h>
#include <jan_mhz19b.h>
#include <Ticker.h>
#include <zrak_api.h>

#define SSID "wifi2"
#define PASSWORD "vsegrepozraku"

#define NTPSERVER "ntp.siq.si"

#define SENSOR_BME280

#ifdef SENSOR_BME280
#include <jan_bme280.h>
bme280 bme(0x76);
#endif

#ifdef SENSOR_SHT30
#include <jan_sht30.h>
sht30 sht(0x44, 0, 4);
#endif

ledMatrix lm;
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, NTPSERVER);
MHZ19B mhz;
HardwareSerial co2_serial(2);
Ticker animation;
TaskHandle_t taskMeasure;
TaskHandle_t taskDisplay;
QueueHandle_t co2_q, t_q, hum_q;
zrak_client zrak("jan", "jan");
#define DEVICE_ID 5

unsigned long t_5 = millis();
unsigned long t_1 = millis();
unsigned long t_now;
int co2 = 5678, temp = 12, hum = 34;

void refreshDisplay(void * pvParameters);
void measure(void * pv);
void showFrame();
void startUpAnimation();

void setup() {
  lm.init();
  animation.attach_ms(10, startUpAnimation);

  WiFi.begin(SSID, PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }

  timeClient.begin();
  timeClient.update();
  delay(2000);

  mhz.begin(&co2_serial, 19, 21);
  #ifdef SENSOR_BME280
  bme.begin(0, 4);
  bme.setMode(bme280::MODE_FORCED);
  bme.setIIRFilter(bme280::FILTER_X4);
  bme.setTemperatureOverSampling(bme280::SAMPLING_X16);
  bme.setHumidityOverSampling(bme280::SAMPLING_X16);
  bme.setPressureOverSampling(bme280::SAMPLING_X16);
  bme.setMode(bme280::MODE_FORCED);
  #endif

  co2_q = xQueueCreate(1, sizeof(int));
  t_q = xQueueCreate(1, sizeof(int));
  hum_q = xQueueCreate(1, sizeof(int));

  xTaskCreatePinnedToCore(
                      measure,   /* Function to implement the task */
                      "task_measure", /* Name of the task */
                      5000,      /* Stack size in words */
                      NULL,       /* Task input parameter */
                      0,          /* Priority of the task */
                      &taskMeasure,       /* Task handle. */
                      0);
  setTime(timeClient.getEpochTime() + 3600);
  timeClient.end();
  animation.detach();

  xTaskCreatePinnedToCore(
                      refreshDisplay,   /* Function to implement the task */
                      "task_display", /* Name of the task */
                      5000,      /* Stack size in words */
                      NULL,       /* Task input parameter */
                      0,          /* Priority of the task */
                      &taskDisplay,       /* Task handle. */
                      1);
}

void loop() {
  showFrame();
}

void showFrame() {
  if (lm.ready) {
    lm.showFrame();
  }
  delay(2);
}

void refreshDisplay(void * pvParameters) {
  for (;;) {
    if (uxQueueMessagesWaiting(co2_q) > 0 ) {
      xQueueReceive(co2_q, &co2, portMAX_DELAY);
    }
    if (uxQueueMessagesWaiting(t_q) > 0 ) {
      xQueueReceive(t_q, &temp, portMAX_DELAY);
    }
    if (uxQueueMessagesWaiting(hum_q) > 0 ) {
      xQueueReceive(hum_q, &hum, portMAX_DELAY);
    }
    lm.drawGUI(hour(), minute(), second(), day(), month(), year(), co2, hum, temp);
    delay(1000);
  }
}

void measure(void * pv) {
  unsigned long t60 = millis();
  for (;;) {
    #ifdef SENSOR_SHT30
    sht.singleMeasurement();
    float HUM = sht.humidity;
    float T = sht.temperature;
    #endif
    #ifdef SENSOR_BME280
    bme.measurement();
    float HUM = bme.humidity;
    float T = bme.temperature;
    float p = bme.pressure;
    #endif
    int CO2 = mhz.getCO2();
    int t = round(T);
    int hum = round(HUM);
    xQueueSend(co2_q, &CO2, portMAX_DELAY);
    xQueueSend(t_q, &t, portMAX_DELAY);
    xQueueSend(hum_q, &hum, portMAX_DELAY);
    delay(5000);
    if ((millis() - t60) > 60000) {
      zrak.addVariable("CO2", CO2);
      zrak.addVariable("T", T);
      zrak.addVariable("RH", HUM);
      #ifdef SENSOR_BME280
      zrak.addVariable("p", p);
      #endif
      zrak.send(DEVICE_ID);
      t60 = millis();
    }
  }
}

void startUpAnimation() {
  static int i = 0;
  if (i<2048) {
    lm.setPixel(1, i%64, i/64, lm.RED);
    i++;
  }
  lm.showFrame();
}