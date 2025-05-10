#include <Arduino.h>
#include <ArduinoModbus.h>
#include <ArduinoRS485.h>
#include <Controllino.h>
#include <Ethernet.h>
#include <SPI.h>

// Controllino Pin Definitions
int start_switch = CONTROLLINO_R0;
int stop_switch = CONTROLLINO_R9;
int led_red = CONTROLLINO_R2;
int led_green = CONTROLLINO_R1;
int led_blue = CONTROLLINO_R3;
int emer_switch = CONTROLLINO_DI0;
int limit_b_l = CONTROLLINO_AI5; //limit_1 //Analog input from old code
int limit_b_r = CONTROLLINO_AI4; //limit_2 //Analog input from old code
int limit_f_l = CONTROLLINO_AI2; //limit_3 //Analog input from old code
int limit_f_r = CONTROLLINO_AI3; //limit_4 //Analog input from old code
//int battery_read = CONTROLLINO_A0;
//int touchless_switch = CONTROLLINO_AI0;

// Variable Definitions
long preMillis = 0;
long debugMillis = 0;

// Ethernet Definitions
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
IPAddress ip(192, 168, 1, 190); // Controllino IP address
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);

// Modbus Definitions
EthernetServer ethernetServer(502);
ModbusTCPServer modbusTCPServer;

// Function Definetions
// void led_set(bool r, bool g, bool b);
void ReadAndPackSensor();
void ReadCommand();

void setup() {
  Serial.begin(115200);
  Serial.println(F("Start Controllinno Modbus Server"));

  // IO Setup
  pinMode(start_switch,OUTPUT);
  pinMode(stop_switch,OUTPUT);
  pinMode(led_red,OUTPUT);
  pinMode(led_green,OUTPUT);
  pinMode(led_blue,OUTPUT);
  pinMode(emer_switch,INPUT);
  Serial.println(F("IO Setup Complete"));

  // Set LED For Testing
  // led_set(true,true,true);

  // Ethernet Setup
  Serial.print(F("Starting Ethernet with IP: "));
  Serial.println(ip);
  Ethernet.begin(mac, ip, gateway, subnet);
  delay(1000); // Wait for Ethernet to initialize

  // —— Start Modbus TCP server ——  
  if (!modbusTCPServer.begin()) {
    Serial.println(F("ERROR: Failed to start Modbus TCP Server"));
    while (1);
  }
  Serial.println(F("Modbus TCP Server started successfully"));
  
  //Start server
  ethernetServer.begin();

  // Reserve coils 0–21
  modbusTCPServer.configureCoils(0, 22);
  Serial.println(F("Modbus coils configured (0-21)"));
  Serial.println(F("Setup complete, entering main loop"));
}

void loop() {
  // Check if a new TCP client has connected
  EthernetClient client = ethernetServer.available();
  if (client) {
    Serial.println(F(">> New Modbus client connected"));
    // Hand over to the Modbus server
    modbusTCPServer.accept(client);

    // While the client remains connected, poll for Modbus requests
    while (client.connected()) {
      // Update sensor coils & read commands each time poll() returns true
      if (modbusTCPServer.poll()) {
        ReadAndPackSensor();
        ReadCommand();
        // Print debug information every 2 seconds
        if (millis() - debugMillis > 2000) {
          debugMillis = millis();
          Serial.println(F("--- Status Update ---"));
          Serial.print(F("Emergency switch: "));
          Serial.println(!digitalRead(emer_switch) ? "ON" : "OFF");
          
          Serial.print(F("Limit sensors (BL,BR,FL,FR): "));
          Serial.print(analogRead(limit_b_l) == 0 ? "1" : "0");
          Serial.print(F(","));
          Serial.print(analogRead(limit_b_r) == 0 ? "1" : "0");
          Serial.print(F(","));
          Serial.print(analogRead(limit_f_l) == 0 ? "1" : "0");
          Serial.print(F(","));
          Serial.println(analogRead(limit_f_r) == 0 ? "1" : "0");
          
          Serial.print(F("Ethernet link: "));
          Serial.println(Ethernet.linkStatus() == LinkON ? "ON" : "OFF");
          
          Serial.println(F("-------------------"));
        }
      }
    }

    Serial.println(F(">> Modbus client disconnected"));
  }

  // Small delay to avoid hammering the server
  delay(10);
}

// void led_set(bool r, bool g, bool b) {
//   digitalWrite(led_red, r);
//   digitalWrite(led_blue, g);
//   digitalWrite(led_green, b);
// }

void ReadAndPackSensor(){
  // Read emergency switch and update Modbus coils
  modbusTCPServer.coilWrite(0, !digitalRead(emer_switch));

  // Read bumper sensors and update Modbus coils
  bool t_limit_b_l = (analogRead(limit_b_l) == 0);
  modbusTCPServer.coilWrite(1, t_limit_b_l);
  bool t_limit_b_r = (analogRead(limit_b_r) == 0);
  modbusTCPServer.coilWrite(2, t_limit_b_r);
  bool t_limit_f_l = (analogRead(limit_f_l) == 0);
  modbusTCPServer.coilWrite(3, t_limit_f_l);
  bool t_limit_f_r = (analogRead(limit_f_r) == 0);
  modbusTCPServer.coilWrite(4, t_limit_f_r);
}

void ReadCommand(){
  // LED Control Commands
  // if (modbusTCPServer.coilRead(10)){
  //   // coil10 → led_red
  //   led_set(true, false, false);
  //   modbusTCPServer.coilWrite(10, 0); 
  // }
  // if (modbusTCPServer.coilRead(11)){
  //   // coil11 → led_green
  //   led_set(false, true, false);
  //   modbusTCPServer.coilWrite(11, 0); 
  // }
  // if (modbusTCPServer.coilRead(12)){
  //   // coil12 → led_blue
  //   led_set(false, false, true);
  //   modbusTCPServer.coilWrite(12, 0); 
  // }
  // if (modbusTCPServer.coilRead(13)){
  //   // coil13 → led_off
  //   led_set(false, false, false);
  //   modbusTCPServer.coilWrite(13, 0); 
  // }
  // if (modbusTCPServer.coilRead(14)){
  //   // coil14 → led_white
  //   led_set(true, true, true);
  //   modbusTCPServer.coilWrite(14, 0);
  // }
  // if (modbusTCPServer.coilRead(15)){
  //   // coil15 → led_purple
  //   led_set(true, true, false);
  //   modbusTCPServer.coilWrite(15, 0);
  // }
  // if (modbusTCPServer.coilRead(16)){
  //   // coil16 → led_yellow
  //   led_set(true, false, true);
  //   modbusTCPServer.coilWrite(16, 0);
  // }
  // if (modbusTCPServer.coilRead(17)){
  //   // coil17 → led_indigo
  //   led_set(false, true, true);
  //   modbusTCPServer.coilWrite(17, 0);
  // }

  // Motor Control Commands
  if (modbusTCPServer.coilRead(20)){
    // coil20 → start_motor
    digitalWrite(start_switch, HIGH);
    digitalWrite(stop_switch, LOW);
    delay(1000); // Wait for 1 second
    digitalWrite(start_switch, LOW);
    modbusTCPServer.coilWrite(18, 0);
  }
  if (modbusTCPServer.coilRead(21)){
    // coil21 → stop_motor
    digitalWrite(start_switch, LOW);
    digitalWrite(stop_switch, HIGH);
    delay(1000); // Wait for 1 second
    digitalWrite(stop_switch, LOW);
    modbusTCPServer.coilWrite(19, 0);
  }
}