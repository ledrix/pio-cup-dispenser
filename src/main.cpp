#include "pcb.h"
#include "image.h"
#include "config.h"
#include "version.h"

//Arduino -------------------
#include <Preferences.h>
//WIFI ----------------------
#include <ArduinoOTA.h>
#include <WiFiManager.h>
//Ethernet ------------------
#include <EthernetESP32.h>
//Network -------------------
#ifdef KUKA_ROBOT
#include <KukaVar.h>
#endif //KUKA_ROBOT
#ifdef NEUROMEKA_ROBOT
#include <ModbusEthernet.h>
#endif //NEUROMEKA_ROBOT
#include <ESPmDNS.h>
#include <WebServer.h>
#include <PubSubClient.h>
//Display -------------------
#include <SSD1306Wire.h>
//Current Sensor ------------
#include <Adafruit_INA219.h>

//Objects ----------------------------------------
WiFiManager wifi;
WebServer server(80);
Preferences settings;
W5500Driver w5500(ETH_CS);
Adafruit_INA219 ina219(0x45);
NetworkClient mqtt_client;
PubSubClient mqtt(mqtt_client);
SSD1306Wire OLED(0x3C, SDA, SCL, GEOMETRY_128_32);
#ifdef KUKA_ROBOT
NetworkClient kuka_client;
KukaVar kuka(kuka_IP, 7000, kuka_client);
#endif //KUKA_ROBOT
#ifdef NEUROMEKA_ROBOT
ModbusEthernet modbus;
#endif //NEUROMEKA_ROBOT
//------------------------------------------------

//Global Variables -------------------------------
hw_timer_t *timer = nullptr;

volatile uint16_t steps = 0;
volatile int32_t position = 0;

volatile uint8_t motor[] = { 0, 0, 0, 0};

double speed[] = { 4, 4, 4, 4};
double target[] = { 5.0, 5.0, 5.0, 5.0};
uint16_t timeout[] = { 5000,
                       5000,
                       5000,
                       5000 };

bool sensor_type = NPN;
uint8_t step_mode = 32;
uint8_t threshold = 50;
uint16_t wait_time = 1000;

double i_hold[] = { 0, 0, 0, 0};

volatile double current = 0;
volatile double voltage = 0;
bool current_sensor = false;

bool mqtt_connected = false;
bool robot_connected = false;
bool ethernet_connected = false;

TaskHandle_t kuka_taskhandle = nullptr;
TaskHandle_t mqtt_taskhandle = nullptr;
TaskHandle_t motor_taskhandle = nullptr;
TaskHandle_t ina219_taskhandle = nullptr;
TaskHandle_t statusLED_taskhandle = nullptr;
TaskHandle_t neuromeka_taskhandle = nullptr;
//------------------------------------------------

//Function Prototypes ----------------------------
void boot_screen();
void dashboard_screen();
void setMicroStep(uint8_t mode);
void kuka_task(void *parameters);
void mqtt_task(void *parameters);
void motor_task(void *parameters);
void ina219_task(void *parameters);
void statusLED_task(void *parameters);
void neuromeka_task(void *parameters);
//------------------------------------------------

//Interrupt Handlers -----------------------------
void IRAM_ATTR timer_ISR() 
{
  digitalWrite(stepPin, !digitalRead(stepPin));

  steps = steps + 1;

  if(digitalRead(dirPin) == COUNTERCLOCKWISE)
    position = position + 1;
  else
    position = position - 1;
}
//------------------------------------------------


void setup() 
{
  Serial.begin(115200);
  Serial.printf("\nHi there! My name is %s.\n", name);
  Serial.printf("Firmware @ %s\n", VERSION_STR);

  // Saved Settings
  settings.begin(name);
  for(int m = 0; m < 4; m++)
  {
    speed[m]   = settings.getDouble((String("speed-")   + m).c_str(),   speed[m]);
    target[m]  = settings.getDouble((String("target-")  + m).c_str(),  target[m]);
    timeout[m] = settings.getUShort((String("timeout-") + m).c_str(), timeout[m]);
  }
  wait_time   = settings.getUShort("wait_time"  ,   wait_time);
  step_mode   = settings.getUChar ("step_mode"  ,   step_mode);
  threshold   = settings.getUChar ("threshold"  ,   threshold);
  sensor_type = settings.getBool  ("sensor_type", sensor_type);

  // IO Pins
  pinMode(dirPin, OUTPUT);
  pinMode(stepPin, OUTPUT);
  pinMode(pullerPin, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  for(int i = 0; i < 4; i++)
  {
    pinMode(sensorPin[i], INPUT);
    pinMode(enablePin[i], OUTPUT);
    digitalWrite(enablePin[i], HIGH);
  }
  for(int i = 0; i < 3; i++)
    pinMode(modePin[i], OUTPUT);

  digitalWrite(pullerPin, !sensor_type);

  setMicroStep(step_mode);
  
  // Initialize Ethernet
  Ethernet.setHostname(name);
  Ethernet.init(w5500);
  Ethernet.begin();
  if(Ethernet.hardwareStatus() == EthernetNoHardware)
    Serial.println("[Ethernet]: Hardware not found!");
  else if(Ethernet.linkStatus() == LinkOFF)
    Serial.println("[Ethernet]: Cable not connected!");
  else
  {
    ethernet_connected = true;
    Serial.print("[Ethernet]: IP = ");
    Serial.println(Ethernet.localIP());
  }
  
  // Timer
  timer = timerBegin(1E+6);
  timerAttachInterrupt(timer, &timer_ISR);

  // FreeRTOS
  xTaskCreatePinnedToCore(  motor_task,
                            "Stepper Motors",
                            8192,
                            nullptr,
                            1,
                            &motor_taskhandle,
                            ARDUINO_RUNNING_CORE );
  
  xTaskCreatePinnedToCore(  ina219_task,
                            "INA219 Sensor",
                            8192,
                            nullptr,
                            1,
                            &ina219_taskhandle,
                            ARDUINO_RUNNING_CORE );

  xTaskCreatePinnedToCore(  mqtt_task,
                            "MQTT Communication",
                            8192,
                            nullptr,
                            1,
                            &mqtt_taskhandle,
                            ARDUINO_RUNNING_CORE );

#ifdef KUKA_ROBOT
  xTaskCreatePinnedToCore(  kuka_task,
                            "KUKA Robot",
                            8192,
                            nullptr,
                            1,
                            &kuka_taskhandle,
                            ARDUINO_RUNNING_CORE );
#endif //KUKA_ROBOT

#ifdef NEUROMEKA_ROBOT
  xTaskCreatePinnedToCore(  neuromeka_task,
                            "Neuromeka Robot",
                            8192,
                            nullptr,
                            1,
                            &neuromeka_taskhandle,
                            ARDUINO_RUNNING_CORE );
#endif //NEUROMEKA_ROBOT
  
  xTaskCreatePinnedToCore(  statusLED_task,
                            "Status LED",
                            1024,
                            nullptr,
                            1,
                            &statusLED_taskhandle,
                            ARDUINO_RUNNING_CORE );

  // OLED Display
  OLED.init();
  OLED.flipScreenVertically();

  // Boot Animation
  boot_screen();
  
  // WiFi
  wifi.setHostname(name);
  wifi.setDarkMode(true);
  wifi.setTitle("Robotine");
  wifi.setAPCallback( [](WiFiManager *myWiFiManager)
  {
    OLED.clear();
    OLED.drawString(64,  0, "Configure");
    OLED.drawString(64, 16,   "WiFi!"  );
    OLED.display();
  });
#ifdef RELEASE
  wifi.setConnectTimeout(5 * 60);
  wifi.setConfigPortalTimeout(5 * 60);
#endif //RELEASE
  wifi.setConfigPortalBlocking(false);
  wifi.autoConnect(name);

  // WebServer
  server.on("/settings", []()
  {
    String str = "";
    str += "\nVersion = " + String(VERSION_STR) + "\n";
    for(int i = 0; i < 4; i++)
    {
      str += "\nMOTOR " + String(i+1);
      str += "\n└> Speed   = " + String(speed[i]) + " RPS";
      str += "\n└> Target  = " + String(target[i]) + " revs.";
      str += "\n└> Timeout = " + String(timeout[i]) + " ms";
      str += "\n";
    }
    str += "\n-> Wait Time = " + String(wait_time) + "ms";
    str += "\n-> Step Mode = 1/" + String(step_mode);
    str += "\n-> Sensor Type = " + String(sensor_type == NPN ? "NPN" : "PNP");
    str += "\n-> Stall Threshold = " + String(threshold) + "%";
    server.send(200, "text/html", "[" + String(name) + "]: Saved Settings...\n" + str + "\n");
  });
  server.on("/target", []()
  {
    int m = -1;
    double r = -1.0;
    for(int i = 0; i < server.args(); i++)
    {
      String argName = server.argName(i);
      argName.toLowerCase();
      if(argName.indexOf("motor") != -1)
        m = server.arg(i).toInt();
      if(argName.indexOf("revs") != -1)
        r = server.arg(i).toDouble();
    }
    if(m > 0 && m <= 4 && r > 0 && r <= 1000)
    {
      m = m - 1;
      target[m] = r;
      settings.putDouble((String("target-") + m).c_str(), target[m]);
      server.send(200, "text/plain", "[" + String(name) + "]: OK\n");
    }
    else
      server.send(200, "text/plain", "[" + String(name) + "]: Error!\nArgs:\nMotor -> [1, 4]\nRevs -> ]0.0, 1000.0] (revs)\n");
  });
  server.on("/timeout", []()
  {
    int m = -1;
    int t = -1.0;
    for(int i = 0; i < server.args(); i++)
    {
      String argName = server.argName(i);
      argName.toLowerCase();
      if(argName.indexOf("motor") != -1)
        m = server.arg(i).toInt();
      if(argName.indexOf("time") != -1)
        t = server.arg(i).toInt();
    }
    if(m > 0 && m <= 4 && t >= 1 && t <= 60E+3)
    {
      m = m - 1;
      timeout[m] = t;
      settings.putUShort((String("timeout-") + m).c_str(), timeout[m]);
      server.send(200, "text/plain", "[" + String(name) + "]: OK\n");
    }
    else
      server.send(200, "text/plain", "[" + String(name) + "]: Error!\nArgs:\nMotor -> [1, 4]\nTime -> [1, 60000] (ms)\n");
  });
  server.on("/step_mode", []()
  {
    int md = -1;
    for(int i = 0; i < server.args(); i++)
    {
      String argName = server.argName(i);
      argName.toLowerCase();
      if(argName.indexOf("mode") != -1)
        md = server.arg(i).toInt();
    }
    if((md == 1 || md%2 == 0) && md <= 64)
    {
      server.send(200, "text/plain", "[" + String(name) + "]: OK\n");
      setMicroStep(md);
    }
    else
      server.send(200, "text/plain", "[" + String(name) + "]: Error!\nArgs:\nMode -> [1, 64]\n");
  });
  server.on("/threshold", []()
  {
    int value = -1;
    for(int i = 0; i < server.args(); i++)
    {
      String argName = server.argName(i);
      argName.toLowerCase();
      if(argName.indexOf("value") != -1)
        value = server.arg(i).toInt();
    }
    if(value >= 0 && value <= 255)
    {
      if(value == 0)
        server.send(200, "text/plain", "[" + String(name) + "]: OK\nWARNING: Stall Monitoring Disabled!\n");
      else
        server.send(200, "text/plain", "[" + String(name) + "]: OK\n");
      threshold = value;
      settings.putUChar("threshold", threshold);
    }
    else
      server.send(200, "text/plain", "[" + String(name) + "]: Error!\nArgs:\nValue -> [0, 255] (%)\n");
  });
  server.on("/speed", []()
  {
    int m = -1;
    double spd = -1;
    for(int i = 0; i < server.args(); i++)
    {
      String argName = server.argName(i);
      argName.toLowerCase();
      if(argName.indexOf("motor") != -1)
        m = server.arg(i).toInt();
      if(argName.indexOf("rps") != -1)
        spd = server.arg(i).toDouble();
      if(argName.indexOf("rpm") != -1)
        spd = server.arg(i).toDouble() / 60.0;
    }
    if(spd > 0 && spd <= 6.0 && m > 0 && m <= 4)
    {
      m = m - 1;
      speed[m] = spd;
      settings.putDouble((String("speed-") + m).c_str(), speed[m]);
      server.send(200, "text/plain", "[" + String(name) + "]: OK\n");
    }
    else
      server.send(200, "text/plain", "[" + String(name) + "]: Error!\nArgs:\nMotor -> [1, 4]\nRPS -> ]0.0, 6.0]\nor\nRPM -> ]0.0, 360.0]\n");
  });
  server.on("/wait", []()
  {
    int time = -1;
    for(int i = 0; i < server.args(); i++)
    {
      String argName = server.argName(i);
      argName.toLowerCase();
      if(argName.indexOf("time") != -1)
        time = server.arg(i).toInt();
    }
    if(time >= 0 && time <= 60E+3)
    {
      wait_time = time;
      settings.putUShort("wait_time", wait_time);
      server.send(200, "text/plain", "[" + String(name) + "]: OK\n");
    }
    else
      server.send(200, "text/plain", "[" + String(name) + "]: Error!\nArgs:\nTime -> [0, 60000] (ms)\n");
  });
  server.on("/sensor", []()
  {
    int type = -1;
    for(int i = 0; i < server.args(); i++)
    {
      String argName = server.argName(i);
      argName.toLowerCase();
      if(argName.indexOf("type") != -1)
      {
        if(server.arg(i).indexOf("NPN"))
          type = NPN;
        if(server.arg(i).indexOf("PNP"))
          type = PNP;
      }
    }
    if(type != -1)
    {
      sensor_type = type;
      digitalWrite(pullerPin, !sensor_type);
      settings.putBool("sensor_type", sensor_type);
      server.send(200, "text/plain", "[" + String(name) + "]: OK\n");
    }
    else
      server.send(200, "text/plain", "[" + String(name) + "]: Error!\nArgs:\nType -> NPN / PNP\n");
  });
  server.on("/manual", []()
  {
    int m = -1;
    for(int i = 0; i < server.args(); i++)
    {
      String argName = server.argName(i);
      argName.toLowerCase();
      if(argName.indexOf("motor") != -1)
        m = server.arg(i).toInt();
    }
    if(m > 0 && m <= 4)
    {
      motor[m-1] = true;
      server.send(200, "text/plain", "[" + String(name) + "]: OK\n");
    }
    else
      server.send(200, "text/plain", "[" + String(name) + "]: Error!\nArgs:\nMotor -> [1, 4]\n");
  });
  server.on("/wr", []()
  {    
    server.send(200, "text/plain", "Credenciais Wifi Resetadas\n");
    wifi.resetSettings();
  });
  server.onNotFound([](){ server.send(200, "text/plain", "[" + String(name) + "]: Not Found!\n"); });
  server.begin();

  // mDNS Service
  MDNS.begin(name);
  MDNS.addService("http", "tcp", 80);

  // OTA Updates
  ArduinoOTA.setHostname(name);
  ArduinoOTA.onStart([]() 
  {
    Serial.println("[OTA]: Updating...");
    OLED.clear();
    OLED.setFont(ArialMT_Plain_16);
    OLED.setTextAlignment(TEXT_ALIGN_CENTER);
    OLED.drawString(64, 1, "Updating...");
    OLED.display();
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) 
  {
    OLED.drawProgressBar(14, 24, 100, 6, (progress / (total / 100)));
    OLED.display();
  });
  ArduinoOTA.onEnd([]() 
  {    
    Serial.println("[OTA]: Updating...OK");
    OLED.clear();
    OLED.drawString(64, 8, "Update OK!");
    OLED.display();
    delay(1000);
  });
  ArduinoOTA.onError([](ota_error_t error) 
  {
    Serial.println("[OTA]: Updating...FAIL");
    (void)error;
    OLED.clear();
    OLED.drawString(64, 8, "Update Error!");
    OLED.display();
    delay(2000);
  });
  ArduinoOTA.begin();
}


void loop() 
{
  ArduinoOTA.handle();

  server.handleClient();

  if(!WiFi.isConnected())
  {
    wifi.process();

    // static uint32_t then = millis();
    // if(millis() - then >= 1000)
    // {
    //   WiFi.reconnect();
    //   then = millis();
    // }
  }

  dashboard_screen();
}


void ina219_task(void *parameters)
{
  Serial.println("[INA219]: Initializing...");
  for(int i = 0; i < 100; i++)
  {
    if(ina219.begin()) break;
    else vTaskDelay(pdMS_TO_TICKS(100));
  }
  if(ina219.success())
  {
    ina219.setCalibration_32V_2A();
    Serial.println("[INA219]: Initializing...OK");
    current_sensor = true;

    // vTaskSuspend(motor_taskhandle);
    // for(int i = 0; i < 4; i++)
    //   digitalWrite(enablePin[i], HIGH);
    // for(int i = 0; i < 4; i++)
    // {
    //   digitalWrite(enablePin[i], LOW);
    //   delay(1000);
    //   double total = 0;
    //   for(int m = 0; m < 100; m++)
    //   {
    //     current = ina219.getCurrent_mA();
    //     total += current;
    //     vTaskDelay(pdMS_TO_TICKS(10));
    //   }
    //   i_hold[i] = total/100;
    //   digitalWrite(enablePin[i], HIGH);
    // }
    // vTaskResume(motor_taskhandle);
  }
  else
  {
    Serial.println("[INA219]: Initializing...FAIL");
    current_sensor = false;
    vTaskSuspend(nullptr);
  }  
  
  for(;;)
  {
    current = ina219.getCurrent_mA();
    voltage = ina219.getBusVoltage_V();
    vTaskDelay(pdMS_TO_TICKS(2));
  }
}


#ifdef KUKA_ROBOT
void kuka_task(void *parameters)
{
  uint8_t i = 0;
  String response;
  uint16_t robot[] = {0, 0, 0, 0};
  uint16_t pRobot[] = {0, 0, 0, 0};

  for(;;)
  {
    vTaskDelay(pdMS_TO_TICKS(1000/kuka_refresh));
    
    robot_connected = kuka.connected();
    if(robot_connected)
    {
      response = kuka.read(String(kuka_var) + "[" + (i + 1) + "]");
      Serial.printf("%s[%d] = %s", kuka_var, (i + 1), response);

      if(response.indexOf("ERROR") != -1)
        continue;
      
      robot[i] = response.toInt();

      if(robot[i] > 0)
      {
        if(robot[i] == 1)
        {
          if(pRobot[i] == 0)
            motor[i] = 1;
          else if(motor[i] == 2)
          {
            if(!kuka.write(String(kuka_var) + "[" + (i + 1) + "]", String(motor[i])))
              continue;
          }
        }
        else if(robot[i] == 2)
        {
          if(motor[i] == 3)
          {
            if(!kuka.write(String(kuka_var) + "[" + (i + 1) + "]", String(motor[i])))
              continue;
          }
        }
        else if(robot[i] == 3)
        {
          if(motor[i] == 0)
          {
            if(!kuka.write(String(kuka_var) + "[" + (i + 1) + "]", String(motor[i])))
              continue;
          }
          else if(motor[i] == 2)
            motor[i] = 3;
        }
      }
      else
        i = (i + 1 >= 4) ? 0 : i + 1;

      pRobot[i] = robot[i];
    }
    else
    {
      Serial.println("[KUKA]: Connecting...");
      if(kuka.connect())
        Serial.println("[KUKA]: Connecting...OK");
      else
        Serial.println("[KUKA]: Connecting...FAIL");
    }
  }
}
#endif //KUKA_ROBOT


#ifdef NEUROMEKA_ROBOT
void neuromeka_task(void *parameters)
{
  IPAddress IP;
  IP.fromString(neuromeka_IP);

  uint8_t i = 0;
  uint16_t robot[] = {0, 0, 0, 0};
  uint16_t pRobot[] = {0, 0, 0, 0};

  modbus.client();
  for(;;)
  {
    robot_connected = modbus.isConnected(IP);
    if(!robot_connected)
    {
      Serial.println("[Neuromeka]: Connecting...");
      if(!modbus.connect(IP, 502))
      {
        Serial.println("[Neuromeka]: Connecting...FAIL");
        vTaskDelay(pdMS_TO_TICKS(100));
        continue;
      }
      Serial.println("[Neuromeka]: Connecting...OK");
    }
    else
    {
      modbus.task();
      
      modbus.readHreg(IP, neuromeka_address, robot, 4);

      Serial.printf("i:%d | rbt:%d / p:%d | pcb:%d\n", i+1, robot[i], pRobot[i], motor[i]);//TEST ONLY!

      if(robot[i] > 0)
      {
        if(robot[i] == 1)
        {
          if(pRobot[i] == 0)
          {
            motor[i] = 1;
            modbus.writeHreg(IP, neuromeka_address + 4 + i, motor[i]);
          }
          else if(motor[i] == 2)
            modbus.writeHreg(IP, neuromeka_address + 4 + i, motor[i]);
        }
        else if(robot[i] == 2)
        {
          if(motor[i] == 3)
            modbus.writeHreg(IP, neuromeka_address + 4 + i, motor[i]);
        }
        else if(robot[i] == 3)
        {
          if(motor[i] == 0)
            modbus.writeHreg(IP, neuromeka_address + 4 + i, motor[i]);
          else if(motor[i] == 2)
            motor[i] = 3;
        }
        pRobot[i] = robot[i];
      }
      else
      {
        pRobot[i] = robot[i];
        i = (i + 1 >= 4) ? 0 : i + 1;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(1000/neuromeka_refresh));
  }
}
#endif //NEUROMEKA_ROBOT


void callback(char* topic, byte* payload, unsigned int length) 
{
  Serial.printf("[MQTT]: Message arrived @ %s --> ", topic);

  String message = String((char*)payload).substring(0, length);

  Serial.println(message);

  if(message.indexOf("ping") != -1)
    mqtt.publish("dev/monitor",("[" + String(name) + "]: Pong!").c_str());

  for(int i = 1; i <= 4; i++)
  {
    if(String(topic).indexOf((String(name) + "/" + i)) != -1)
    {
      if(message.indexOf("TRUE") != -1)
        motor[i-1] = true;
      return;
    }
  }
}

void mqtt_task(void *parameters)
{
  mqtt.setServer(MQTT_IP, 1883);
  mqtt.setCallback(callback);

  for(;;)
  {
    mqtt_connected = mqtt.connected();
    if(!mqtt_connected)
    {
      Serial.println("[MQTT]: Connecting...");
      if(mqtt.connect(name))
      {
        Serial.println("[MQTT]: Connecting...OK");

        for(int i = 1; i <= 4; i++)
          mqtt.subscribe((String(name) + "/" + String(i)).c_str(), 0);
        
        mqtt.subscribe("dev/data", 0);

        if(ethernet_connected)
          mqtt.publish("dev/monitor", ("[" + String(name) + "]: Ethernet").c_str());
        else
          mqtt.publish("dev/monitor", ("[" + String(name) + "]: WiFi").c_str());
      }
      else
      {
        Serial.printf("[MQTT]: Connecting...FAIL (%d)\n", mqtt.state());
        delay(1000);
      }
    }
    else
    {
      mqtt.loop();

      for(int i = 0; i < 4; i++)
        mqtt.publish((String(name) + "/" + String(i+1) + "/state").c_str(), STEP_STR[motor[i]]);
      
      mqtt.publish((String(name) + "/current").c_str(), String(current, 0).c_str());
      mqtt.publish((String(name) + "/voltage").c_str(), String(voltage, 2).c_str());

      vTaskDelay(pdMS_TO_TICKS(1000/mqtt_rate));
    }
  }
}


void motor_task(void *parameters)
{
  TickType_t now;
  TickType_t then;

  int count = 0;

  for(;;)
  {
    for(int i = 0; i < 4; i++)
    {
      if(motor[i] == 1)
      {
        digitalWrite(stepPin, LOW);
        vTaskDelay(pdMS_TO_TICKS(1));
        digitalWrite(enablePin[i], LOW);
        vTaskDelay(pdMS_TO_TICKS(1));
        digitalWrite(dirPin, COUNTERCLOCKWISE);
        vTaskDelay(pdMS_TO_TICKS(1));

        steps = 0;
        position = 0;
        timerAlarm(timer, 1E+6 / (speed[i] * step_mode * STEPS_REV * 2), true, 0);
        timerStart(timer);

        then = xTaskGetTickCount();
        for(;;)
        {
          now = xTaskGetTickCount();
          if(now - then >= pdMS_TO_TICKS(timeout[i]))
            break;

          if((double(position) / double(STEPS_REV * step_mode * 2)) >= target[i])
            break;

          if(!current_sensor || threshold == 0)
            continue;

          // if(current >= (1 + threshold/100.0) * i_hold[i])
          // {
          //   timerAlarmDisable(timer);
          //   vTaskDelay(pdMS_TO_TICKS(100));
          //   digitalWrite(dirPin, CLOCKWISE);
          //   vTaskDelay(pdMS_TO_TICKS(1));

          //   uint16_t pSteps = steps;
          //   timerAlarmEnable(timer);
          //   for(;;)
          //   {
          //     if((steps - pSteps) >= (REVERSE_REVS * STEPS_REV * step_mode * 2)) 
          //       break;
          //     if(position <= 0)
          //       break;
          //   }
            
          //   timerAlarmDisable(timer);
          //   digitalWrite(dirPin, COUNTERCLOCKWISE);
          //   vTaskDelay(pdMS_TO_TICKS(1));
          //   timerAlarmEnable(timer);
          // }
        }
        timerStop(timer);
        motor[i] = 2;
        
        then = xTaskGetTickCount();
        for(;;)
        {
          now = xTaskGetTickCount();
          if(now - then >= pdMS_TO_TICKS(wait_time))
            motor[i] = 3;
          if(motor[i] == 3)
            break;
          vTaskDelay(pdMS_TO_TICKS(10));
        }

        digitalWrite(dirPin, CLOCKWISE);
        vTaskDelay(pdMS_TO_TICKS(1));

        timerAlarm(timer, 1E+6 / (speed[i] * step_mode * STEPS_REV * 2), true, 0);
        timerStart(timer);

        then = xTaskGetTickCount();
        for(;;)
        {
          // now = xTaskGetTickCount();
          // if(now - then >= pdMS_TO_TICKS(timeout[i]))
          //   break;

#ifdef NO_LIM_SW
          if(position <= 0)
            break;
#endif //NO_LIM_SW

          if(digitalRead(sensorPin[i]) == sensor_type)
            break;

          if(!current_sensor || threshold == 0)
            continue;
        }

        timerStop(timer);
        digitalWrite(enablePin[i], HIGH);
        digitalWrite(stepPin, LOW);
        motor[i] = 0;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}


void boot_screen()
{
  OLED.setFont(ArialMT_Plain_16);
  OLED.setTextAlignment(TEXT_ALIGN_CENTER);
  for(int i = 32; i > -64; i--)
  {
    OLED.clear();
    OLED.drawFastImage(0, i, 128, 32, robotine_logo_128_32);
    OLED.display();
    if(i == 0) delay(1000);
    else delay(5);
  }
  OLED.clear();
  do
  {
    OLED.clear();
    OLED.drawString(64, 1, "INA219 Setup");
    for (int i = 0; i <= 100; i++)
    {
      OLED.drawProgressBar(14, 24, 100, 6, i);
      OLED.display();
      delay(5);
    }
  }
  while(!current_sensor);

  // OLED.clear();  
  // OLED.drawString(64, 1, "Motor Calibration");
  // for (int i = 0; i <= 100; i++)
  // {
  //   OLED.drawProgressBar(4, 24, 120, 6, i);
  //   OLED.display();
  //   delay(40);
  // }
}

void dashboard_screen()
{
  OLED.clear();

  for(int i = 0; i < 4; i++)
  {
    if(motor[i])
    {
      OLED.setFont(ArialMT_Plain_10);
      OLED.setTextAlignment(TEXT_ALIGN_CENTER);
      OLED.drawString(32, 0, ("MOTOR " + String(i+1)).c_str());
      OLED.setFont(ArialMT_Plain_16);
      static int points = 0;
      String temp;
      for(int i = 0; i < points; i++) temp += "°";
      OLED.drawString(32, 14, temp);
      points++;
      if(points > 3)
        points = 0;

      OLED.setFont(ArialMT_Plain_10);
      OLED.setTextAlignment(TEXT_ALIGN_RIGHT);
      OLED.drawVerticalLine(64, 2, 28);
      OLED.drawString(100,  0, "i  =");
      OLED.drawString(100, 10, "pos =");
      OLED.drawString(128,  0, String(current, 0));
      OLED.drawString(128, 10, String((float(position)/(2*step_mode))/STEPS_REV));
      OLED.setTextAlignment(TEXT_ALIGN_CENTER);
      switch(motor[i])
      {
        case 1:
          OLED.drawString(100, 22, "---->");
          break;
        case 2:
          OLED.drawString(100, 22, "-----");
          break;
        case 3:
          OLED.drawString(100, 22, "<----");
          break;
      }
      OLED.display();
      return;
    }
  }

  OLED.setFont(ArialMT_Plain_10);
  OLED.drawVerticalLine(63, 2, 30);
  OLED.drawHorizontalLine(65, 22, 64);
  OLED.setTextAlignment(TEXT_ALIGN_CENTER);
#ifdef KUKA_ROBOT
  OLED.drawString(97, 22, "KUKA");
#endif //KUKA_ROBOT
#ifdef NEUROMEKA_ROBOT
  OLED.drawString(97, 22, "Neuromeka");
#endif //NEUROMEKA_ROBOT
  OLED.setTextAlignment(TEXT_ALIGN_RIGHT);
  OLED.drawString(83, 0,  "V =");
  OLED.drawString(83, 10, "i  =");
  OLED.drawString(128, 0, String(voltage) + "V");
  OLED.drawString(128, 10, String(current, 0) + "mA");

  static bool dot = false;
  static uint32_t then = millis();
  if(millis() - then >= 500)
  {
    dot = !dot;
    then = millis();
  }
  OLED.setTextAlignment(TEXT_ALIGN_LEFT);
  if(ethernet_connected)
  {
    OLED.drawString(0,  2, dot ? "°" : " ");
    OLED.drawString(5,  0, "Ethernet");
  }
  else
  {
    OLED.drawString(0,  2, WiFi.isConnected() && dot ? "°" : " ");
    OLED.drawString(5,  0, String(WiFi.SSID().c_str(), 11));
  }
  OLED.drawString(0, 12, mqtt_connected && dot ? "°" : " ");
  OLED.drawString(5, 10, "MQTT");
  OLED.drawString(0, 22, robot_connected && dot ? "°" : " ");
  OLED.drawString(5, 20, "Robot");
  
  OLED.display();
}


void setMicroStep(uint8_t mode)
{
  step_mode = mode;
  settings.putUChar("step_mode", step_mode);

  for(int i = 0; i < 3; i++)
    digitalWrite(modePin[i], LOW);
  if(mode / 16 > 0)
  {
    mode = mode / 16;
    digitalWrite(modePin[2], HIGH);
  }
  if(mode / 4 > 0)
  {
    mode = mode / 4;
    digitalWrite(modePin[1], HIGH);
  }
  if(mode / 2 > 0)
  {
    mode = mode / 2;
    digitalWrite(modePin[0], HIGH);
  }
}


void statusLED_task(void *parameters)
{
  byte blynks;

  for(;;)
  {
    ethernet_connected = Ethernet.connected();

    blynks = 1 
           + 1 * mqtt_connected 
           + 2 * robot_connected;
    for(int i = 0; i < blynks; i++)
    {
      digitalWrite(LED_BUILTIN, HIGH);
      vTaskDelay(pdMS_TO_TICKS(100));
      digitalWrite(LED_BUILTIN, LOW);
      vTaskDelay(pdMS_TO_TICKS(100));
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}