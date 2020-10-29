/*
  Door and lock sensor
  Communication via ESP-Now
  Written by W. Hoogervorst
  April 2020
*/

#include <ESP8266WiFi.h>
#include <credentials.h>  // WiFi credentials

// for HTTP update
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <ESP8266HTTPUpdateServer.h>

extern "C" {
#include "user_interface.h"
}
extern "C" {
#include <espnow.h>
}
// ESP-Now - MAC address of the ESP with which it is paired (slave), global defined since used in setup and loop
uint8_t mac_addr[6] = {0x6A, 0xC6, 0x3A, 0xC4, 0xA7, 0x5C}; // MAC address of access point for ESP-Now communication

// for HTTPupdate
const char* host = "shed doorsensor";
ESP8266WebServer httpServer(80);
ESP8266HTTPUpdateServer httpUpdater;
const char* software_version = "version 7";

// MQTT
const char* mqtt_id = "doorsensor4";
char* doorsensor_topic = "sensor/doorsensor4";
char* locksensor_topic = "sensor/locksensor4";
char* debug_topic = "sensor/doorsensor4/debug";
char* voltage_topic = "sensor/doorsensor4/voltage";
char* payload_open = "OPEN";                // MQTT payload
char* payload_closed = "CLOSED";            // MQTT payload

uint8_t result = 1;

#define MINUTE 60e6  // 60e6 is 60 seconds = 1 minute

#define CALLBACKMAXTIME 800
#define LENGTH 40 //message length
#define VOLTAGEINTERVAL 31  // publish measured voltage every # times = # - 1 wait time
#define WIFI_CONNECT_TIMEOUT_S 15

#define DOORDETECTPIN 12
#define LOCKDETECTPIN 13
#define VOLTAGESENSORPIN 14
#define OTAPIN 4

WiFiClient client;

// RTC-MEM Adresses
#define RTC_CHECK 64
//#define RTC_CHECK1 65
#define RTC_STATE 66
#define RTC_COUNT 67
#define RTC_DOOR 68
#define RTC_LOCK 69

#define OPEN 0
#define CLOSED 1

#define STATE_CHECK 0 //wake up with Radio off, just check if something has changed
#define STATE_INIT 1  //wake up with Radio on and transmit door and lock states
#define STATE_DOOR 2  //wake up with Radio on, publish doorstate next time
#define STATE_LOCK 3  //wake up with Radio on, publish lockstate next time
#define STATE_VOLTAGE 4  //wake up with Radio on, publish voltage next time
#define STATE_OTA 5  //wake up with Radio on, go to OTA modus


#define CHECK_VALUE_1 55 // change values for a fresh start
#define CHECK_VALUE_2 88 // change values for a fresh start

// global variables
int counter = 0;
uint32_t time1;//, time2;

int doorstate, lockstate, prev_doorstate, prev_lockstate;
int state, prev_state;
#define SERIAL_DEBUG 1

void setup() {
  pinMode(DOORDETECTPIN, INPUT_PULLUP);
  pinMode(LOCKDETECTPIN, INPUT_PULLUP);
  pinMode(OTAPIN, INPUT_PULLUP);
  pinMode(VOLTAGESENSORPIN, OUTPUT);
  digitalWrite(VOLTAGESENSORPIN, HIGH);

  // if serial is not initialized all following calls to serial end dead.
  if (SERIAL_DEBUG)
  {
    Serial.begin(115200);
    delay(10);
    Serial.println("");
    Serial.println("");
    Serial.println("");
    Serial.println(F("Started from reset"));
  }

  // read door and lock state
  doorstate = digitalRead(DOORDETECTPIN);
  lockstate = digitalRead(LOCKDETECTPIN);
  // read state value
  system_rtc_mem_read(RTC_STATE, &prev_state, sizeof(prev_state)); // read 2 bytes from RTC-MEMORY
  Serial.print("previous state: ");
  Serial.println(prev_state);

  system_rtc_mem_read(RTC_DOOR, &prev_doorstate, sizeof(prev_doorstate)); // read previous doorstate from RTC-MEMORY
  system_rtc_mem_read(RTC_LOCK, &prev_lockstate, sizeof(prev_lockstate)); // read previous lockstate from RTC-MEMORY
  system_rtc_mem_read(RTC_COUNT, &counter, sizeof(counter)); // read counter from RTC-MEMORY
  Serial.print("Program has run times: ");
  Serial.println(counter);
  counter++;
  Serial.println("Increase and write");
  system_rtc_mem_write(RTC_COUNT, &counter, sizeof(counter));

  Serial.print("previous door and lock state values:\t");
  Serial.print(prev_doorstate);
  Serial.print("\t");
  Serial.println(prev_lockstate);
  /*
  Serial.print("Size of prev_lockstate: ");
  Serial.println(sizeof(prev_lockstate));
  */
  Serial.print("current door and lock state values:\t");
  Serial.print(doorstate);
  Serial.print("\t");
  Serial.println(lockstate);

  switch (prev_state)
  {
    case STATE_CHECK:
      {
        if (digitalRead(OTAPIN) == LOW)
        {
          state = STATE_OTA;
          break;
        }
        if (doorstate == prev_doorstate)
        {
          Serial.println("doorstate has not changed since previous measurement, change nothing");
          state = prev_state;
        }
        else
        {
          Serial.println("doorstate has CHANGED since previous measurement, write new state to RTC memory");
          system_rtc_mem_write(RTC_DOOR, &doorstate, sizeof(doorstate));     // write doorstate to RTC memory
          state = STATE_DOOR;
          break;
        }
        if (lockstate == prev_lockstate)
        {
          Serial.println("lockstate has not changed since previous measurement, change nothing");
          state = prev_state;
        }
        else
        {
          Serial.println("lockstate has CHANGED since previous measurement, write new state to RTC memory");
          system_rtc_mem_write(RTC_LOCK, &lockstate, sizeof(lockstate));     // write lockstate to RTC memory
          state = STATE_LOCK;
          break;
        }
        if (counter % VOLTAGEINTERVAL == 0)
          state = STATE_VOLTAGE;
        break;
      }
    case STATE_INIT:  // first send door state, then send lock state
      {
        init_esp_now();
        if (digitalRead(DOORDETECTPIN) == OPEN)
        {
          senddata(doorsensor_topic, payload_open);
        }
        else
        {
          senddata(doorsensor_topic, payload_closed);
        }
        state = STATE_LOCK;
        break;
      }
    case STATE_DOOR:
      {
        init_esp_now();
        if (digitalRead(DOORDETECTPIN) == OPEN)
        {
          senddata(doorsensor_topic, payload_open);
        }
        else
        {
          senddata(doorsensor_topic, payload_closed);
        }
        if (lockstate == prev_lockstate)
        {
          Serial.println("lockstate has not changed since previous measurement, change nothing");
          state = STATE_CHECK;
        }
        else
        {
          Serial.println("lockstate has CHANGED since previous measurement, write new state to RTC memory");
          system_rtc_mem_write(RTC_LOCK, &lockstate, sizeof(lockstate));     // write lockstate to RTC memory
          state = STATE_LOCK;
        }
        break;
      }
    case STATE_LOCK:
      {
        init_esp_now();
        if (digitalRead(LOCKDETECTPIN) == OPEN)
        {
          senddata(locksensor_topic, payload_open);
        }
        else
        {
          senddata(locksensor_topic, payload_closed);
        }
        if (doorstate == prev_doorstate)
        {
          Serial.println("doorstate has not changed since previous measurement, change nothing");
          state = STATE_CHECK;
        }
        else
        {
          Serial.println("doorstate has CHANGED since previous measurement, write new state to RTC memory");
          system_rtc_mem_write(RTC_DOOR, &doorstate, sizeof(doorstate));     // write doorstate to RTC memory
          state = STATE_DOOR;

        }
        break;
      }

    case STATE_VOLTAGE:
      {
        init_esp_now();
        digitalWrite(VOLTAGESENSORPIN, LOW);
        delay(50);
        Serial.print("ADC reading: ");
        float voltage = 0;
        // A0 can read 0 - 1.0V as 0 - 1023, battery voltage is max 4.2V, so use a voltage divider.
        for (int i = 0; i < 10; i++) //take 10 measurements
          //voltage divider 10k and 47k resistors, then 1 volt at ADC pin is 5,7 V.
          voltage += map(analogRead(A0), 0, 1024, 0, 570);
        voltage = voltage / (float)1000;      // divide by 100 for converting to value to a float with 2 digits, divide by 10 for averaging measurements


        Serial.println(analogRead(A0));
        Serial.print("Voltage is: ");
        Serial.println(voltage, 2);
        Serial.print("corrected Voltage is: ");
        float correctedvoltage = voltage * 1.154; // correction since MOSFET apparently is not fully on at battery voltagelevel
        Serial.println(correctedvoltage, 2);

        digitalWrite(VOLTAGESENSORPIN, HIGH);
        int volt2 = round(correctedvoltage * 100);
        float volt_round = volt2 / (float)100;
        String tmp_str = String(volt_round); //converting voltage to a string
        char voltagebuf[5];
        tmp_str.toCharArray(voltagebuf, tmp_str.length() + 1);

        senddata(voltage_topic, voltagebuf);
        state = STATE_CHECK;
        break;
      }
    case STATE_OTA:
      {
        Serial.println("OTA mode in setup, start wifi");
        WiFi.mode(WIFI_STA);
        delay(10);
        // We start by connecting to a WiFi network
        Serial.println();
        Serial.print("Connecting to ");
        Serial.println(mySSID);
        WiFi.begin(mySSID, myPASSWORD);
        time1 = millis();
        while (WiFi.status() != WL_CONNECTED) {
          delay(500);
          Serial.print(".");
          if (millis() > time1 + (WIFI_CONNECT_TIMEOUT_S * 1000))
            ESP.restart();
        }
        Serial.println("");
        Serial.println("WiFi connected");
        Serial.println("IP address: ");
        Serial.println(WiFi.localIP());
        // for HTTPudate
        MDNS.begin(host);
        httpUpdater.setup(&httpServer);
        httpServer.begin();
        MDNS.addService("http", "tcp", 80);
        httpServer.on("/", handleRoot);
        state = STATE_CHECK;
        Serial.print("state: ");
        Serial.println(state);
        system_rtc_mem_write(RTC_STATE, &state, sizeof(state)); // read 2 bytes from RTC-MEMORY
        return;
        break;
      }
    default:
      {
        Serial.println("started up, no matching state, next time STATE_INIT");
        counter = 1;
        //system_rtc_mem_write(RTC_CHECK, buf, sizeof(buf));
        system_rtc_mem_write(RTC_COUNT, &counter, sizeof(counter));     // set counter to 1
        system_rtc_mem_write(RTC_DOOR, &doorstate, sizeof(doorstate));     // write doorstate to RTC memory
        system_rtc_mem_write(RTC_LOCK, &lockstate, sizeof(lockstate));     // write lockstate to RTC memory
        state = STATE_INIT;
        break;
      }
  }
  Serial.print("state: ");
  Serial.println(state);
  system_rtc_mem_write(RTC_STATE, &state, sizeof(state)); // read 2 bytes from RTC-MEMORY

  if (state == STATE_CHECK)
  {
    Serial.println("Going for a normal sleep, wake up with no WiFi");
    //WiFi.begin(mySSID, myPASSWORD);
    ESP.deepSleep(1 * MINUTE, WAKE_RF_DISABLED);
    delay(100);
  }
  else
  {
    Serial.println("Going to a short sleep, wake up with WiFi ENABLED");
    //WiFi.begin(mySSID, myPASSWORD);
    ESP.deepSleep(10, WAKE_RF_DEFAULT);
    delay(100);

  }
}

void loop() {
  httpServer.handleClient();    // for HTTPupdate
  yield();
}

void init_esp_now() {
  // Initialize the ESP-NOW protocol
  esp_now_init();
  /*
    if (esp_now_init() != 0) {
    ESP.restart();    // no restart when esp_now_init fails
    }
  */
  // *** DECLARATION OF THE ROLE OF THE ESP DEVICE IN THE COMMUNICATION *** //
  // 0 = LOOSE, 1 = MASTER, 2 = SLAVE and 3 = MASTER + SLAVE
  esp_now_set_self_role(1);   // sender

  // *** PAIRING WITH THE SLAVE *** //
  uint8_t role = 2;   // role of receiver = slave
  uint8_t channel = 1;  // WiFi channel of receiver access point
  esp_now_add_peer(mac_addr, role, channel, NULL, 0);   // NULL means there is no key, length of key is 0

  // set up the call-back function for the confirmation of the sent data. This is executed is ESP-NOW is used further on in the program
  esp_now_register_send_cb([](uint8_t *mac,  uint8_t result2) {
    char MACslave[6];
    // in this call back function the result is stored in the "result" variable, which is important in the program
    result = result2;
    sprintf(MACslave, " % 02X: % 02X: % 02X: % 02X: % 02X: % 02X", mac [0], mac [1], mac [2], mac [3], mac [4], mac [5]);

    // display result on serial if serial is initialized
    Serial.print("Data sent to ESP MAC: "); Serial.print(MACslave);
    Serial.print(". Reception(0 = 0K - 1 = ERROR): "); Serial.println(result);
  });
  Serial.println("ESP - now initialized");
}


uint8_t senddata (char * topic_data, char * payload_data)
{
  // prepare the data to send
  time1 = millis();
  char DATA[LENGTH];
  memset(DATA, '\0', LENGTH);    // Initialice or clear the string
  strcat(DATA, topic_data);   // Copy the topic to the array.
  strcat(DATA, "&");   // Copy "&" symbol to the array.
  strcat(DATA, payload_data);   // Copy payload to the array.

  uint8_t data[sizeof(DATA)];
  memcpy(data, &DATA, sizeof(DATA));
  uint8_t len = sizeof(data);

  Serial.print("DATA: "); Serial.println(DATA);
  Serial.print("data: "); Serial.println((char*) data);

  while ((result == 1) && (millis() - time1 < CALLBACKMAXTIME))
  {
    esp_now_send(mac_addr, data, len);
    delay(200);
  }
  return result;
}

void handleRoot() {
  String message = "WimIOT\nDevice: ";
  message += host;
  message += "\nSoftware version: ";
  message += software_version;
  message += "\nUpdatepath at http://[IP]/update";
  httpServer.send(200, "text/plain", message);
}
