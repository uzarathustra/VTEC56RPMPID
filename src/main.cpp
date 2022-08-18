#include <Arduino.h>
#include <PID_v1.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <SPIFFS.h>
#include <Sauron.h>

// Define Webserver
char buffer[17];
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
AsyncEventSource events("/events");
const char html[] PROGMEM = R"rawliteral(
<p>Encodervalue:<div id="enc"> %PLACEHOLDER_ENC% </div> </p>
)rawliteral";

AsyncWebSocketClient* wsClient;

// Define PWM pins
//uint8_t  MotFwd = 22;                    // Motor Forward pin
//uint8_t  MotRev = 23;                    // Motor Reverse pin
const uint8_t  MotFwd1 = 16;               // Motor1 Forward pin
const uint8_t  MotRev1 = 17;               // Motor1 Reverse pin
const uint8_t  MotFwd2 = 19;               // Motor2 Forward pin
const uint8_t  MotRev2 = 18;               // Motor2 Reverse pin
const uint8_t  MotFwd3 = 25;               // Motor3 Forward pin
const uint8_t  MotRev3 = 26;               // Motor3 Reverse pin

// User input variables
String readString;                         //This while store the user input data
long User_Input = 0;                       // This while convert input string into integer

// Encoder-Init
int encoderPin1 = 33;                      // Encoder Output 'A'
int encoderPin2 = 32;                      // Encoder Output 'B'
const int encoder1Pin1 = 34;               // Encoder Output 'A' - Motor 1
const int encoder1Pin2 = 35;               // Encoder Output 'B' - Motor 1
const int encoder2Pin1 = 32;               // Encoder Output 'A' - Motor 2
const int encoder2Pin2 = 33;               // Encoder Output 'B' - Motor 2
const int encoder3Pin1 = 36;               // Encoder Output 'A' - Motor 3
const int encoder3Pin2 = 39;               // Encoder Output 'B' - Motor 3
volatile long encoderValue = 0;            // Raw encoder value
volatile long encoder1Value = 0;           // Raw encoder value - Motor 1
volatile long encoder2Value = 0;           // Raw encoder value - Motor 2
volatile long encoder3Value = 0;           // Raw encoder value - Motor 3
long REV = 0;                              // Set point REQUIRED ENCODER VALUE

// PID-Init
double kp_factor = 0.5 , ki_factor = 0.0001 , kd_factor = 0.00001;             // modify for optimal performance
double input = 0, output = 0, setpoint = 0;
PID myPID(&input, &output, &setpoint, kp_factor, ki_factor, kd_factor, DIRECT);

// WiFi-Parameter
String wifi_ssid = "PRAX";      			    // WiFi SSID
String wifi_pw = "4160233293075903";      // WiFi password

// Control-Parameter
unsigned int motor = 1;
long position = 0;
long pos_m1 = 0;                          // End-position motor 1
long pos_m2 = 0;                          // End-position motor 2
long pos_m3 = 0;                          // End-position motor 3
unsigned int reset = 0;                   // JSON-Reset -> 1 - Clear ConfigFile in SPIFF (JSON)
unsigned int debug = 1;                   // Debug Output Mode -> 1 - Debug ouput

// Time
unsigned long myTime;

// Sauron-Parameter
unsigned int sauron = 1;                  // sauron = 0 (deactive); sauron = 1 (active)
unsigned int channel = 0;
unsigned int iref= 5120;
unsigned int tint = 64;
String wsClient_returnValue = "";
volatile long sauronValue = 0;            // Raw sauron value
uint8_t i = 0;                            // Zähler
uint8_t num_ch_to_read = 8;               // Anzahl der Kanäle
int8_t isMicroWatt = -1;                  // Einheitenumbau
uint16_t ready_pin_timeouts = 0;
uint16_t faktor_einheit = 0;
uint16_t counts_current = 0;
uint16_t divider = 1;
uint16_t data_buffer[] = { 0, 0, 0, 0 };  // Kanal-Werte-Puffer
double power = 0;
double counts_1mW_normiert = 1;


// *******************************************************************
//
//                              SPIFFS
//                      (List all Files in FS)   
//
// ******************************************************************
//
void listFilesInDir(File dir, int numTabs) {
  while (true) {
    File entry =  dir.openNextFile();
    if (! entry) {
      // no more files in the folder
      break;
    }
    for (uint8_t i = 0; i < numTabs; i++) {
      Serial.print('\t');
    }
    Serial.print(entry.name());
    if (entry.isDirectory()) {
      Serial.println("/");
      listFilesInDir(entry, numTabs + 1);
    } else {
      // display zise for file, nothing for directory
      Serial.print("\t\t");
      Serial.println(entry.size(), DEC);
    }
    entry.close();
  }
}

// *******************************************************************
//
//                              SPIFFS
//                         (Read Config-File)                      
//
// *******************************************************************
//
// Webserver event variable
void read_config(unsigned int deb){

  /**
   * File derivate from Stream so you can use all Stream method
   * readBytes, findUntil, parseInt, println etc
   */

  // Open SPIFF-File (Read)
  File confFile = SPIFFS.open(F("/config.txt"), "r");
  
  if (confFile)
  {
    if (deb == 1) {Serial.println("Read file content!");}

    String dataString;

    // Extract each characters by one by one from File
    while (confFile.available()){
      dataString += char(confFile.read());
    }

    // Read JSON to work with
    StaticJsonDocument<500> doc;
    auto error = deserializeJson (doc, dataString);

    //Check for errors in parsing
    if (error) {   
      if (deb == 1) {
        // Print error message
        Serial.print(F("deserializeJson() failed with code "));
        Serial.println(error.c_str());
      }
      return;
    }

    // Print dataString
    if (deb == 1) {Serial.println(dataString);Serial.println("");}

    //Close File
    confFile.close();
  }

  else
  {
    if (deb == 1) {Serial.println("Problem on read file!");}
  }

}

// *******************************************************************
//
//                              SPIFFS
//                          Save Config-File                      
//
// *******************************************************************
//
// Webserver event variable
void save_config(uint8_t * data, unsigned int &mot, long &pos, long &pos_m1, \
long &pos_m2, long &pos_m3, unsigned int &deb, unsigned int &res, double &kp, \
double &ki, double &kd, String wifi_ssid, String wifi_pw ){

  
  // *** Read JSON-Data (websocket --> ws) to work with ***

  // Read JSON to work with - from webSocket-Data
  StaticJsonDocument<500> doc_ws;
  auto error_ws = deserializeJson (doc_ws, data);

  //Check for errors in parsing
  if (error_ws) {
    if (deb == 1) {
      // Print error message
      Serial.print(F("deserializeJson() failed with code "));
      Serial.println(error_ws.c_str());
    }
    return;
  }

  // JSON-Object to iterate over its key
  JsonObject docRoot_ws = doc_ws.as<JsonObject>();


  // *** Read JSON-Data (SPIFF --> fs) to work with ***

  // Open SPIFF-File (Read)
  File confFile_r = SPIFFS.open("/config.txt", "r");

  // Extract each characters by one by one from File
  String dataString;
  while (confFile_r.available())
  {
    dataString += char(confFile_r.read());
  }
  //Serial.println("DataString(Read):");
  Serial.println(dataString);

  // Read JSON to work with - from ConfigFile (SPIFF)
  StaticJsonDocument<500> doc_fs;
  auto error_fs = deserializeJson (doc_fs, dataString);

  //Check for errors in parsing
  if (error_fs) {
    if (deb == 1) {
      // Print error message
      Serial.print(F("deserializeJson() failed with code "));
      Serial.println(error_fs.c_str());
    }
    return;
  }

  // *** Update always Motor-Position-Data (SETUP-Issue)
    pos_m1 = doc_fs["pos_m1"];
    pos_m2 = doc_fs["pos_m2"];
    pos_m3 = doc_fs["pos_m3"];

  // *** Update all Key values sent from webSocket into ConfigFile (SPIFF) ***
    if (deb == 1) {    
      Serial.println("Update Data:");
    }

  if (doc_ws.size() == 0) { Serial.println("Nichts enthalten!");}

  // Iterate JSON Key Values - jp.key() and jp.value();
  for (JsonPair jp : docRoot_ws) {
    String tmp_value = doc_ws[jp.key().c_str()];
    if (deb == 1) {    
      Serial.print(jp.key().c_str());
      Serial.print(" : ");
      Serial.println(tmp_value);
    }
    // Update JSON Key Values for ConfigFile
    doc_fs[jp.key().c_str()] = tmp_value;

    // Update Parameter
    if (strcmp(jp.key().c_str(), "mot") == 0) { mot = tmp_value.toInt(); };
    if (strcmp(jp.key().c_str(), "pos") == 0) { pos = atol(tmp_value.c_str()); };
    if (strcmp(jp.key().c_str(), "pos_m1") == 0) { pos_m1 = atol(tmp_value.c_str()); };
    if (strcmp(jp.key().c_str(), "pos_m2") == 0) { pos_m2 = atol(tmp_value.c_str()); };
    if (strcmp(jp.key().c_str(), "pos_m3") == 0) { pos_m3 = atol(tmp_value.c_str()); };
    if (strcmp(jp.key().c_str(), "deb") == 0) { deb = tmp_value.toInt(); };
    if (strcmp(jp.key().c_str(), "res") == 0) { res = tmp_value.toInt(); };   
    if (strcmp(jp.key().c_str(), "kp") == 0) { kp = tmp_value.toDouble(); };
    if (strcmp(jp.key().c_str(), "ki") == 0) { ki = tmp_value.toDouble(); };
    if (strcmp(jp.key().c_str(), "kd") == 0) { kd = tmp_value.toDouble(); };
    if (strcmp(jp.key().c_str(), "wifi_ssid") == 0) { wifi_ssid = tmp_value; };
    if (strcmp(jp.key().c_str(), "wifi_pw") == 0) { wifi_pw = tmp_value; };
  }

  // Update WiFi always if it exist (from ConfigFile)
  if (doc_fs.containsKey("wifi_ssid")) { wifi_ssid = (const char*)doc_fs["wifi_ssid"];}
  if (doc_fs.containsKey("wifi_pw")) { wifi_pw = (const char*)doc_fs["wifi_pw"];}

  // Remove all JSON-pair-elements
  if (doc_fs["res"] == "1") {
    doc_fs.clear();
    Serial.println("JSON-File gelöscht");
  }

  // Close SPIFF-File (Read)
  confFile_r.close();


  // Open SPIFF-File (Write)
  File confFile_w = SPIFFS.open("/config.txt", "w");

  // Save config file direct from json-file into ConfigFile
  if (confFile_w) {
      if (deb == 1) {Serial.println("Write file content!");Serial.println("");}
      serializeJson(doc_fs, confFile_w);
      confFile_w.close();
  } else {
      if (deb == 1) {Serial.println("Problem on create file!");}
  }

  //Close SPIFF-File (Write)
  confFile_w.close();

}

void save_config_empty(){
  String dataString = "{}";
  uint8_t * data = (uint8_t*)dataString.c_str();
  save_config(data, motor, position, pos_m1, pos_m2, pos_m3, debug, reset, kp_factor, ki_factor, kd_factor, wifi_ssid, wifi_pw);
}


// *******************************************************************
//
//                       Initialize WiFi
//    (Wait for WiFi connection, and, if not connected, reboot)
//
// *******************************************************************
//
void waitForWiFiConnectOrReboot(unsigned int deb) {

  // Update WiFi-parameter (JSON-String "{}") - if not available in ConfigFile use default
  //save_config_empty()
  Serial.println(wifi_ssid);
  Serial.println(wifi_pw);

  // WiFi activate
  uint32_t notConnectedCounter = 0;
  if (deb == 1) {Serial.print("Wifi connecting ...");}
  while (WiFi.status() != WL_CONNECTED) {
    delay(100);
    if (deb == 1) {Serial.print(".");}
    notConnectedCounter++;
    if(notConnectedCounter > 50) { // Reset board if not connected after 5s
      if (deb == 1) {Serial.println("Resetting due to Wifi not connecting...");}
       ESP.restart();
    }
  }
  // Print wifi IP addess
  if (deb == 1) {
    Serial.print(" IP address: ");
    Serial.println(WiFi.localIP());
  }
}

void initWifi(){
  WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE);
 	WiFi.mode(WIFI_STA);
  WiFi.setHostname("ESP-Motor");
  WiFi.begin(wifi_ssid.c_str(), wifi_pw.c_str());
  waitForWiFiConnectOrReboot(debug);
}

// *******************************************************************
//
//                       Define PWM-Signals 
//                      
// *******************************************************************
//
void forward (int out, unsigned int mot) {
  if (mot == 1) {
    ledcWrite(1, out);               // Enabling motor enable pin to reach the desire angle
    ledcWrite(2, 0);
  }
  else if (mot == 2) {
    ledcWrite(3, out);               // Enabling motor enable pin to reach the desire angle
    ledcWrite(4, 0);
  }
  else {
    ledcWrite(5, out);               // Enabling motor enable pin to reach the desire angle
    ledcWrite(6, 0);
  } 
}
void reverse (int out, unsigned int mot) {
  if (mot == 1) {
    ledcWrite(1, 0);
    ledcWrite(2, abs(out));          // if REV < encoderValue motor move in forward direction.
  }
  else if (mot == 2) {
    ledcWrite(3, 0);
    ledcWrite(4, abs(out));          // if REV < encoderValue motor move in forward direction.
  }
  else {
    ledcWrite(5, 0);
    ledcWrite(6, abs(out));          // if REV < encoderValue motor move in forward direction.
  }
}
void finish (int out,  unsigned int mot) {
  if (mot == 1) {
    ledcWrite(1, 0); 
    ledcWrite(2, 0);
  }
  else if (mot == 2) {
    ledcWrite(3, 0); 
    ledcWrite(4, 0);
  }
  else {
    ledcWrite(5, 0); 
    ledcWrite(6, 0);
  } 
}

// *******************************************************************
//
//                   Consider direction of movement (PWM)
//                      
// *******************************************************************
//
void pwmOut(int out, unsigned int mot) {                               
  if (out > 0) {                           // if REV > encoderValue motor move in forward direction.    
    forward(out, mot);                     // calling motor to move forward
  }
  else {
    reverse(out, mot);                     // calling motor to move reverse
  }
 readString="";                            // Cleaning User input, ready for new Input
}

// *******************************************************************
//
//                         Encoder update 
//                      
// *******************************************************************
//
void updateEncoder1(){
  bool Pin2 = digitalRead(encoderPin2);
  bool Pin1 = digitalRead(encoderPin1);

  if (Pin2 == LOW) {
    if (Pin1 == HIGH) {
      encoderValue++;
    }
    else {
      encoderValue--;
    }
  }

  else {
    if (Pin1 == HIGH) {
      encoderValue--;
    }
    else {
      encoderValue++;
    }
  }
}

void updateEncoder2(){
  bool Pin2 = digitalRead(encoderPin1);
  bool Pin1 = digitalRead(encoderPin2);

  if (Pin1 == LOW) {
    if (Pin2 == HIGH) {
      encoderValue--;
    }
    else {
      encoderValue++;
    }
  }

  else {
    if (Pin2 == HIGH) {
      encoderValue++;
    }
    else {
      encoderValue--;
    }
  }
}

// *******************************************************************
//
//                         Init Encoder values 
//                      
// *******************************************************************
//
void initEncoderValues(unsigned int mot){
  // assign encoder variables (Pins)
  if (mot == 1) { encoderPin1 = encoder1Pin1; encoderPin2 = encoder1Pin2;}
  else if (mot == 2) { encoderPin1 = encoder2Pin1; encoderPin2 = encoder2Pin2;}
  else { encoderPin1 = encoder3Pin1; encoderPin2 = encoder3Pin2;}
  pinMode(encoderPin1, INPUT_PULLUP); 
  pinMode(encoderPin2, INPUT_PULLUP);
  digitalWrite(encoderPin1, HIGH); //turn pullup resistor on
  digitalWrite(encoderPin2, HIGH); //turn pullup resistor on
  // call updateEncoder() when any high/low changed seen
  // on interrupt 0 (pin 2), or interrupt 1 (pin 3) 
  attachInterrupt(digitalPinToInterrupt(encoderPin1), updateEncoder1, CHANGE); 
  attachInterrupt(digitalPinToInterrupt(encoderPin2), updateEncoder2, CHANGE);
  // Assign encoder start value  
  if (mot == 1) { encoderValue = pos_m1;}
  else if (mot == 2) { encoderValue = pos_m2;}
  else { encoderValue = pos_m3;}
  Serial.println("Enc-Value:");
  Serial.println(encoderValue);
}

// *******************************************************************
//                        Backup Encoder values
//                          (end position)
//                      
// *******************************************************************
//
void backupEncoderValue(unsigned int mot){
  // Assign encoder end value - write only changed motor encoder
  //encoderValue = 11;

  // Define String to save in configFile
  String dataString;
  if (mot == 1) { dataString = "{pos_m1:" + String(encoderValue) + "}";}
  else if (mot == 2) { dataString = "{pos_m2:" + String(encoderValue) + "}";}
  else { dataString = "{pos_m3:" + String(encoderValue) + "}";}
  Serial.println("Backup Data:");
  //Serial.print(dataString);

  // Convert String into Char-Array (Pointer)
  uint8_t * data = (uint8_t*)dataString.c_str();
  save_config(data, motor, position, pos_m1, pos_m2, pos_m3, debug, reset, kp_factor, ki_factor, kd_factor, wifi_ssid, wifi_pw);
}

// *******************************************************************
//
//                    Define Webevent und Websocket
//                      
// *******************************************************************
//
// Webserver event variable
String processor(const String& var){
  if (var == "PLACEHOLDER_ENC"){
    return String(encoderValue);
  }
  return String();
}

// Handling events (multiple)
// Define web socket andpoint handling function
// * server - pointer to server object which receive data
// * client - pointer to cliente object which the data will send back
// type - contain all websocket events that triggers the execution of handling function - check what eent has occured an handle accordingly
// *data - sent data in single frame
// len - length of data (text or binary)
void onWsEvent(AsyncWebSocket * server, AsyncWebSocketClient * client, AwsEventType type, void * arg, uint8_t *data, size_t len){
  if(type == WS_EVT_CONNECT){
    if (debug == 1) {Serial.println("Websocket client connection received ...");}
    wsClient = client;
    //initEncoderValues();
  } else if(type == WS_EVT_DISCONNECT){
    if (debug == 1) {Serial.println("Client disconnected.");}
    wsClient = nullptr;
    backupEncoderValue(motor);
    read_config(debug);
  } else if(type == WS_EVT_DATA){
    if (debug == 1) {Serial.println("Data received ...");}
    save_config(data, motor, position, pos_m1, pos_m2, pos_m3, debug, reset, kp_factor, ki_factor, kd_factor, wifi_ssid, wifi_pw);
    //read_config(debug);
    initEncoderValues(motor);

  }
}

// *******************************************************************
// 
//                           Init SAURON
//                    
// *******************************************************************
//
// void read_config WEGGELASSEN
// void save_config WEGGELASSEN
// void onWsEvent WEGGELASSEN
//
void initSauron(unsigned int deb){
  // volatile long sauronValue = 0;
  init_i2c();   // Initiate the Wire library and join the I2C bus as master or slave. Should be called only once!
  delay(500);

  // set Sauron in config mode
  set_mode(false);  // startet bzw. beendet Messung - true=Starten, false=Beenden

  // set Sauron parameter
  // parameters from text-file to variables
  if (deb == 1) {
    Serial.println("Set Sauron-Parameter:");
    Serial.println(iref);
    Serial.println(tint);
  }
    set_config(tint, iref); // Integrationszeit und Messbereich wird an den Sauron gesendet und Verarbeitet
}

double updateSauron(uint current_data_output){

  // set Sauron in measurement mode
  set_mode(true);   // startet bzw. beendet Messung - true=Starten, false=Beenden
  delay(100); // Verzögerung

  if ((get_data(data_buffer, num_ch_to_read) == 0)) {             // es werden alle 4 Kanäle auslesen - gleichzeitig 8 Byte

    //Serial.println(data_buffer[0]);
    //Serial.println(data_buffer[1]);
    //Serial.println(data_buffer[2]);
    //Serial.println(data_buffer[3]);

    counts_current = data_buffer[current_data_output];            // Counts aktuell
    power = (double)((double)counts_current / counts_1mW_normiert );
    power /= (double)divider;
  }
  //stop measurement mode
  //set_mode(false);
  return power;
}

// *******************************************************************
// *******************************************************************
// 
//                             S E T U P 
//                      
// *******************************************************************
// *******************************************************************
//
void setup() {

  // Serial comunication init (+ Event-Socket)
  Serial.begin(9600); 

  // WiFi connection init
  initWifi();

  delay(500);

  // Launch SPIFFS file system and get all information of SPIFFS
  Serial.print("Inizializing FS ...");
  if (SPIFFS.begin()){
    Serial.println(" SPIFFS mounted correctly.");
  }else{
    Serial.println(" An error occurred during SPIFFS mounting!");
  }
  // SPIFFS.format() // To format all space in SPIFFS
  unsigned int totalBytes = SPIFFS.totalBytes();
  unsigned int usedBytes = SPIFFS.usedBytes();
  Serial.println("===== File system info =====");
  Serial.print("Total space:      ");
  Serial.print(totalBytes);
  Serial.println("byte");
  Serial.print("Total space used: ");
  Serial.print(usedBytes);
  Serial.println("byte");
  Serial.println("Filelist:");
  File dir = SPIFFS.open("/"); // Open dir folder
  listFilesInDir(dir,1);       // List file at root
  Serial.println("===== File system info =====");

  // Read config file and set working variables
  read_config(debug);

  // Event Handling Webserver
  // Route for "/"
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
      //request->send(200, "text/html", "Hello World");
      request->send_P(200, "text/html", html, processor);
  });
  server.addHandler(&events);

  // Webserver init
  ws.onEvent(onWsEvent);
  server.addHandler(&ws);
  server.begin();

  // assign PWM pins to channels
  ledcAttachPin(MotFwd1, 1); // MotFwd -> Channel 1
  ledcAttachPin(MotRev1, 2); // MotRev -> Channel 2
  ledcAttachPin(MotFwd2, 3); // MotFwd -> Channel 3
  ledcAttachPin(MotRev2, 4); // MotRev -> Channel 4
  ledcAttachPin(MotFwd3, 5); // MotFwd -> Channel 5
  ledcAttachPin(MotRev3, 6); // MotRev -> Channel 6

  // Initialize channels 
  // channels 0-15, resolution 1-16 bits, freq limits depend on resolution
  // ledcSetup(uint8_t channel, uint32_t freq, uint8_t resolution_bits);
  ledcSetup(1, 31000, 8); // 12 kHz PWM, 8-bit resolution
  ledcSetup(2, 31000, 8);
  ledcSetup(3, 31000, 8);
  ledcSetup(4, 31000, 8);
  ledcSetup(5, 31000, 8);
  ledcSetup(6, 31000, 8);

  // Init Encoder values
  initEncoderValues(motor);

  // PID variables
  myPID.SetMode(AUTOMATIC);                // set PID in Auto mode
  myPID.SetSampleTime(1);                  // refresh rate of PID controller
  myPID.SetOutputLimits(-255, 255);        // this is the MAX PWM value to move motor, here change in value reflect change in speed of motor.

  // init SAURON
  if (sauron == 1) {   //Check SAURON activation
    initSauron(debug);
  }
}

// *******************************************************************
// *******************************************************************
//
//                         M A I N - L O O P 
//                      
// *******************************************************************
// *******************************************************************
//
void loop() {
  myTime = millis();

  if (sauron == 1) {   //Check SAURON activation
    sauronValue = updateSauron(2);
    // Serial.println(sauronValue);
  }

  // *****************************************************************
  //
  //                    Webclient communication
  //                         (websocket)
  //
  // *****************************************************************
  //
  // If client is connected ...
  if(wsClient != nullptr && wsClient->canSend()) {

    // ... encoder values - create string
    wsClient_returnValue.concat(encoderValue);

    if (sauron == 0) {                     // Check SAURON activation
      sauronValue = 0;
      //delay(100);                          // Delay for interrupt issue
    }
    wsClient_returnValue.concat(" ");
    wsClient_returnValue.concat(sauronValue);

    // output string - websocket (python-client)
    wsClient->text(String(wsClient_returnValue));
    
    // Output string - serial console
    Serial.println(wsClient_returnValue);
    
    // Clear string
    wsClient_returnValue = "";
  }
  

  // *****************************************************************
  //
  //                  Define pulses from encoder 
  //                       (PID-Controller)
  //
  // *****************************************************************
  //                  
  REV = map (position, 0, 360, 0, 8344);   // mapping degree into pulse
  setpoint = REV;                          // PID while work to achive this value consider as SET value
  if((encoderValue == input + 1)||(encoderValue == input -1)){
    Serial.println("Tolerance error!!!");
    position += 5;
  }

  input = encoderValue;                    // data from encoder consider as a Process value
  myPID.Compute();                         // calculate new output (PID-Algotithm)
  pwmOut(output, motor);                   // define PWM-Signal
}