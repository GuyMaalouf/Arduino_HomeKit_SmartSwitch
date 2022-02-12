/* Arduino Homekit with EEPROM - The Workshop by GM - Feb 2, 2022
 *  This code allows you to control a light in your room using the apple homekit app 
 *  as well as reading the temperature and humidity of the room
 *  
 *  Note: CHANGE THE WIFI ID and PASS in wifi_info.h
 */

#include <Arduino.h>
#include <arduino_homekit_server.h>
#include "wifi_info.h"
#include <DHT.h>
#include <EEPROM.h>

#define DHTPIN 2     // Digital pin connected to the DHT sensor 
// Uncomment the type of sensor in use:
#define DHTTYPE    DHT11     // DHT 11
//#define DHTTYPE    DHT22     // DHT 22 (AM2302)
//#define DHTTYPE    DHT21     // DHT 21 (AM2301)

DHT dht(DHTPIN, DHTTYPE);

#define LOG_D(fmt, ...)   printf_P(PSTR(fmt "\n") , ##__VA_ARGS__);

// access your homekit characteristics defined in my_accessory.c
extern "C" homekit_server_config_t config;
extern "C" homekit_characteristic_t cha_current_temperature;
extern "C" homekit_characteristic_t cha_humidity;
extern "C" homekit_characteristic_t led_on;
extern "C" void accessory_init();
extern "C" bool led_power;
extern "C" void led_update();
extern "C" void led_toggle();

#define PIN_Switch 13//D7
int eeprom_add = 1500;
int previous_switch_state;

static uint32_t next_heap_millis = 0;
static uint32_t next_report_millis = 0;
static uint32_t next_switch_millis = 0;

void setup() {
  accessory_init();
  EEPROM.begin(eeprom_add+2); 
  led_power = EEPROM.read(eeprom_add);
  led_update();
  previous_switch_state = EEPROM.read(eeprom_add+1);
  Serial.begin(115200);
  Serial.print("EEPROM Read! Led_Power: ");
  Serial.println(led_power);
  Serial.println("LED Updated");
  homekit_storage_reset(); //If you wish to connect your arduino with a new apple device uncomment this section to reset device.
  dht.begin();
  wifi_connect(); // in wifi_info.h
  my_homekit_setup();
}

void loop() {
  my_homekit_loop();
  delay(10);
}

//==============================
// Homekit setup and loop
//==============================

void my_homekit_setup() {
  arduino_homekit_setup(&config);
}

void my_homekit_loop() {
  arduino_homekit_loop();
  const uint32_t t = millis();
  if (t > next_report_millis) {
    // report sensor values every 10 seconds
    next_report_millis = t + 10 * 1000;
    my_homekit_report();
  }
  if (t > next_heap_millis) {
    // show heap info every 5 seconds
    next_heap_millis = t + 5 * 1000;
    LOG_D("Free heap: %d, HomeKit clients: %d",
        ESP.getFreeHeap(), arduino_homekit_connected_clients_count());

  }
  if (t > next_switch_millis) {
    // show heap info every 1 second
    next_switch_millis = t + 0.5 * 1000;
    //UPDATE SWITCH STATE
    int current_switch_state = digitalRead(PIN_Switch);
    if(current_switch_state != previous_switch_state){
      EEPROM.begin(eeprom_add+2);
      EEPROM.write(eeprom_add+1,!led_power);
      EEPROM.commit();
      Serial.print("EEPROM 2 Updated from: ");
      Serial.print(previous_switch_state);
      previous_switch_state = current_switch_state;
      Serial.print("  -  to: ");
      Serial.println(previous_switch_state);
      Serial.print("LED_ON Updated from: ");
      Serial.print(led_on.value.bool_value);
      led_on.value.bool_value = !led_on.value.bool_value;
      Serial.print("  -  to: ");
      Serial.println(led_on.value.bool_value);
      led_on.setter(led_on.value);
      homekit_characteristic_notify(&led_on, led_on.value);
    }
    //UPDATE LED_POWER
    EEPROM.begin(eeprom_add+1);
    int eeprom_temp = EEPROM.read(eeprom_add);
    if (eeprom_temp != led_power){
      EEPROM.write(eeprom_add,led_power);
      EEPROM.commit();
      Serial.print("EEPROM Updated from: ");
      Serial.print(eeprom_temp);
      eeprom_temp = EEPROM.read(eeprom_add);
      Serial.print("  -  to: ");
      Serial.println(eeprom_temp);
    }
    Serial.print("EEPROM Not Updated, current state: ");
    Serial.println(eeprom_temp);
  }
}

void my_homekit_report() {
  float temperature_value = dht.readTemperature();  
  cha_current_temperature.value.float_value = temperature_value;
  float humidity_value = dht.readHumidity();
  cha_humidity.value.float_value = humidity_value;
  LOG_D("Current temperature: %.1f", temperature_value);
  LOG_D("Current Humidity: %.1f", humidity_value);
  homekit_characteristic_notify(&cha_current_temperature, cha_current_temperature.value);
  homekit_characteristic_notify(&cha_humidity, cha_humidity.value);
}
