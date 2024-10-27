#include <Wire.h>
#include <VL53L1X.h>
#include <Arduino_FreeRTOS.h>
#include <Servo.h> 

char data_in; // variabel data dari bluetooth

TaskHandle_t autonomousHandle = NULL; 

const int potX = A0 ; //potensiometer X
const int potY = A1; //potensiometer Y
int valueX, valueY ;
float angleX, angleY;

const int G_ENA = 8;
const int G_IN1 = 22;
const int G_IN2 = 23;
const int G_IN3 = 24;
const int G_IN4 = 25;
const int G_ENB = 9;

const int T_ENA = 7;
const int T_IN1 = 26;
const int T_IN2 = 27;
const int T_IN3 = 28;
const int T_IN4 = 29;
const int T_ENB = 6;

const int H_IN3 = 30;
const int H_IN4 = 34;


const int SRV = 5;

const int encA_pins[] = {19, 18, 2, 3};
const int encB_pins[] = {38, 40, 42, 44};

volatile int encoderPositions[4] = {0, 0, 0, 0}; // TrolleyKanan, TrolleyKiri, GantryKiri, GantryKanan
//volatile int RPM[4] = {0, 0, 0, 0};
//volatile unsigned long lastTimeRPM = 0 ;
//volatile unsigned long currentTimeRPM = 0;
//volatile int lastEncoderPositions[4];
//const int PPR = 1550;


// Jumlah ToF
const uint8_t sensorCount = 2;

// koneksi pin XSHUT ke arduino
const uint8_t xshutPins[sensorCount] = {31, 32};

float m[] = {1.05,1.02}; // konstanta regresi linear
float c[] = {-54,-26.1};

int sensorValue[] = {0, 0}; // nilai sensor ToF

bool motorState[] = {0, 0, 0, 0, 0, 0, 0, 0}; // dari kiri = G_IN1, G_IN2, G_IN3, G_IN4, T_IN1, T_IN2, T_IN3, T_IN4
int Gspeed = 0; // kecepatan gantry
int Tspeed = 0; // kecepatan trolley
Servo spreader;
VL53L1X sensors[sensorCount];

int pos = 0; // variabel posisi servo

int numData = 100; // jumlah log data
int jarakAktual;

float Kp = 0.1, Ki = 0.006, Kd = 0; //konstanta
int Setpoint = 300;
int Error = 0;
int Input = 0;
float Output = 0;
int prevError = 0.0;
float Integral = 0.0;
float Derivative = 0.0 ;
float rawOutput = 0;
double integralMax = 2000;
double integralMin = -2000;

bool speedSet = false;

unsigned long lastTime;
double sampleTime = 50; //waktu sampling dalam milisecond

void handleSerialInput(void *pvParameters);
void displayToF(void *pvParameters);
void displayEncoders(void *pvParameters);
void readToF(void *pvParameters);
void DisplayPM(void *pvParameters);
void autonomous(void *pvParameters);
void PID(void *pvParameters);
void logDataCont(void *pvParameters);
void DebugPID(void *pvParameters);
//void debugRPM(void *pvParameters);
//void debugPulse(void *pvParameters);


void setup() {
  spreader.attach(5);
  spreader.write(115); 
  Serial3.begin(9600);
  Serial.begin(115200);
  
  pinMode(potX,INPUT);
  pinMode(potY,INPUT);
  
  pinMode(G_ENA, OUTPUT);
  pinMode(G_IN1, OUTPUT);
  pinMode(G_IN2, OUTPUT);
  pinMode(G_IN3, OUTPUT);
  pinMode(G_IN4, OUTPUT);
  pinMode(G_ENB, OUTPUT);

  pinMode(T_ENA, OUTPUT);
  pinMode(T_IN1, OUTPUT);
  pinMode(T_IN2, OUTPUT);
  pinMode(T_IN3, OUTPUT);
  pinMode(T_IN4, OUTPUT);
  pinMode(T_ENB, OUTPUT);

  for (int i = 0; i < 4; i++) {
    pinMode(encA_pins[i], INPUT);
    pinMode(encB_pins[i], INPUT);
  }

  attachInterrupt(digitalPinToInterrupt(19), updateEncoder0, CHANGE);
  attachInterrupt(digitalPinToInterrupt(18), updateEncoder1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(2), updateEncoder2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(3), updateEncoder3, CHANGE);

  Wire.begin();
  Wire.setClock(400000); 

  // mematikan sensor
  for (uint8_t i = 0; i < sensorCount; i++) {
    pinMode(xshutPins[i], OUTPUT);
    digitalWrite(xshutPins[i], LOW);
  }

  // inisialisasi sensor satu persatu
  for (uint8_t i = 0; i < sensorCount; i++) {
    pinMode(xshutPins[i], INPUT);
    delay(10);

    sensors[i].setTimeout(500);
    if (!sensors[i].init()) {
      Serial.print("Failed to detect and initialize sensor ");
      Serial.println(i);
      while (1);
    }

    // setting address sensor
    sensors[i].setAddress(0x2A + i);
    sensors[i].startContinuous(50);
  }

  // Create tasks
  xTaskCreate(handleSerialInput, "SerialInput", 128, NULL, 1, NULL);
//  xTaskCreate(displayToF, "DisplayToF", 128, NULL, 1, NULL);
//  xTaskCreate(displayEncoders, "DisplayEncoders", 128, NULL, 1, NULL);
  xTaskCreate(readToF, "Read ToF", 128, NULL, 1, NULL);
  xTaskCreate(DisplayPM, "Display PM", 128, NULL, 1, NULL);
//  xTaskCreate(PID, "PID", 1024, NULL, 1, NULL);
  xTaskCreate(logDataCont, "Log Data Cont", 128, NULL, 1, NULL);
//  xTaskCreate(DebugPID, "Debug PID", 128, NULL, 1 , NULL);


  // Start the scheduler
  vTaskStartScheduler();
}

void loop() {
}

void handleSerialInput(void *pvParameters) {
  for (;;) {
    if (Serial3.available()) {
      data_in = Serial3.read(); // Get next character

      // 4 way pad
      if (data_in == '1') { // Up Direction Pressed
//        computePID(100);
        G_Drive(110); // min 110
//        T_Drive(100); 
      }
      if (data_in == '2') { // Right Direction Pressed
        T_Drive(150);
      }
      if (data_in == '3') { // Down Direction Pressed
        G_Drive(-150);
      }
      if (data_in == '4') { // Left Direction Pressed
        T_Drive(-150);
      }
      if (data_in == '0') { // 4-way Pad Released
        G_Drive(0);           
//        computePID(0);
        T_Drive(0);
      }
      if (data_in == 'U') { // Button Pressed
        HoistUp();
      }
      if (data_in == 'u') { // Button Released
        HoistStop();
      }
      if (data_in == 'D') { // Button Pressed
        HoistDown(); 
      }
      if (data_in == 'd') { // Button Released
        HoistStop(); 
      }
      if (data_in == 'X') { // Button Pressed
        SpreaderClose();
      }
      if (data_in == 'x') { // Button Released
        //<--- Insert button released code here 
      }
      if (data_in == 'O') { // Button Pressed
        SpreaderOpen();
      }
      if (data_in == 'o') { // Button Released
        //<--- Insert button released code here 
      }
      if (data_in == 'B') { // Button Pressed
      xTaskCreate(autonomous, "autonomous", 1024, NULL, 1, &autonomousHandle);
      }
      if (data_in == 'R') { // Button Pressed
        if(autonomousHandle != NULL){
          vTaskDelete(autonomousHandle);
          autonomousHandle = NULL;
          speedSet=false;
          }
      }        
    }
   if(Serial.available()){
    if (data_in == 'S'){
      }
    }
    vTaskDelay(10 / portTICK_PERIOD_MS); 
  }
}

void displayToF(void *pvParameters) {
  for (;;) {
    for (uint8_t i = 0; i < sensorCount; i++) {
      Serial.print(sensorValue[i]);
      if (i < sensorCount - 1) {
        Serial.print(',');  
      }
    }
    Serial.println();  
    vTaskDelay(50 / portTICK_PERIOD_MS);  
  }
}

void readToF(void *pvParameters) {
  for (;;) {
    float temp;
    for (uint8_t i = 0; i < sensorCount; i++) {
      sensorValue[i] = sensors[i].read();
      temp = (sensorValue[i] * m[i]) + c[i];
      sensorValue[i] = round(temp);
    }
    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}

void displayEncoders(void *pvParameters) {
  for (;;) {
    for (uint8_t i = 0; i < 4; i++) {
      Serial.print("Encoder ");
      Serial.print(i);
      Serial.print(": ");
      Serial.println(encoderPositions[i]);
    }
    vTaskDelay(500 / portTICK_PERIOD_MS); 
  }
}

void G_Drive(int speed) { // min 110 max 220
  if (speed > 0) {
    digitalWrite(G_IN1, LOW);
    digitalWrite(G_IN2, HIGH);
    digitalWrite(G_IN3, LOW);
    digitalWrite(G_IN4, HIGH);

    analogWrite(G_ENA, speed);
    analogWrite(G_ENB, speed);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    Gspeed = speed;
  } else if (speed < 0) {
    digitalWrite(G_IN1, HIGH);
    digitalWrite(G_IN2, LOW);
    digitalWrite(G_IN3, HIGH);
    digitalWrite(G_IN4, LOW);

    for (int i = 0; i < 4; i++) {
      motorState[i] = (i % 2 == 0) ? 1 : 0;
    }
    for (int i = 10; i <= abs(speed); i += 20) {
      analogWrite(G_ENA, i);
      analogWrite(G_ENB, i);
    }
    analogWrite(G_ENA, abs(speed));
    analogWrite(G_ENB, abs(speed));
    vTaskDelay(10 / portTICK_PERIOD_MS);
    Gspeed = speed;
  } else if (speed == 0) {
    digitalWrite(G_IN1, motorState[0]);
    digitalWrite(G_IN2, motorState[1]);
    digitalWrite(G_IN3, motorState[2]);
    digitalWrite(G_IN4, motorState[3]);

    for (int i = abs(Gspeed); i >= 0; i -= 20) {
      analogWrite(G_ENA, i);
      analogWrite(G_ENB, i);
      vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    digitalWrite(G_IN1, LOW);
    digitalWrite(G_IN2, LOW);
    digitalWrite(G_IN3, LOW);
    digitalWrite(G_IN4, LOW);
    for (int i = 0; i < 4; i++) {
      motorState[i] = 0;
    }
    Gspeed = 0;
  }                 
}

void PID_G_Drive(int speed) { // min 110 max 220
//  if (speed > 255) speed = 255;
//  else if (speed < 110 && speed > 0) speed = 120;
//  else if (speed < 0 && speed > -110) speed = -120; // Batas kecepatan negatif
//  else if (speed <= 0) speed = 0;
  if (speed > 0) { // Motor maju
        digitalWrite(G_IN1, LOW);
        digitalWrite(G_IN2, HIGH);
        digitalWrite(G_IN3, LOW);
        digitalWrite(G_IN4, HIGH);
//        for (float i = 1; i <= abs(speed); i += 0.5) {
//          analogWrite(G_ENA, int(i));
//          analogWrite(G_ENB, int(i));
//        Gspeed = i;
////      vTaskDelay(10 / portTICK_PERIOD_MS);
//    }
    analogWrite(G_ENA, abs(speed));
    analogWrite(G_ENB, abs(speed));
    Gspeed = speed;

    motorState[0] = LOW;
    motorState[1] = HIGH;
    motorState[2] = LOW;
    motorState[3] = HIGH;
      speedSet=true;
    

  } else if (speed < 0) { // Motor mundur
    digitalWrite(G_IN1, HIGH);
    digitalWrite(G_IN2, LOW);
    digitalWrite(G_IN3, HIGH);
    digitalWrite(G_IN4, LOW);

    analogWrite(G_ENA, abs(speed));
    analogWrite(G_ENB, abs(speed));
    Gspeed = speed;

    motorState[0] = HIGH;
    motorState[1] = LOW;
    motorState[2] = HIGH;
    motorState[3] = LOW;

  } else if (speed == 0) { // Motor berhenti
    digitalWrite(G_IN1, LOW);
    digitalWrite(G_IN2, LOW);
    digitalWrite(G_IN3, LOW);
    digitalWrite(G_IN4, LOW);
    Gspeed = 0;

    for (int i = 0; i < 4; i++) {
      motorState[i] = LOW; // Simpan status motor sebagai berhenti
    }
  }
  
}


void T_Drive(int speed) { // min 60 max 220
  if (speed > 0) {
    digitalWrite(T_IN1, HIGH);
    digitalWrite(T_IN2, LOW);
    digitalWrite(T_IN3, LOW);
    digitalWrite(T_IN4, HIGH);

    motorState[4] = 1;
    motorState[5] = 0;
    motorState[6] = 0;
    motorState[7] = 1;

    analogWrite(T_ENA, speed);
    analogWrite(T_ENB, speed);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    Tspeed = speed;
  } else if (speed < 0) {
    digitalWrite(T_IN1, LOW);
    digitalWrite(T_IN2, HIGH);
    digitalWrite(T_IN3, HIGH);
    digitalWrite(T_IN4, LOW);

    motorState[4] = 0;
    motorState[5] = 1;
    motorState[6] = 1;
    motorState[7] = 0;

    for (int i = 50; i <= abs(speed); i += 1) {
      analogWrite(T_ENA, i);
      analogWrite(T_ENB, i);
      vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    analogWrite(T_ENA, abs(speed));
    analogWrite(T_ENB, abs(speed));
    Tspeed = speed;
  } else if (speed == 0) {
    digitalWrite(T_IN1, motorState[0]);
    digitalWrite(T_IN2, motorState[1]);
    digitalWrite(T_IN3, motorState[2]);
    digitalWrite(T_IN4, motorState[3]);

    for (int i = abs(Tspeed); i >= 0; i -= 1) {
      analogWrite(T_ENA, i);
      analogWrite(T_ENB, i);
      vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    digitalWrite(T_IN1, LOW);
    digitalWrite(T_IN2, LOW);
    digitalWrite(T_IN3, LOW);
    digitalWrite(T_IN4, LOW);
    for (int i = 4; i < 8; i++) {
      motorState[i] = 0;
    }
    Tspeed = 0;
  }                 
}

void PID_T_Drive(int speed) { // min 60 max 220
  if (speed > 0) {
    digitalWrite(T_IN1, HIGH);
    digitalWrite(T_IN2, LOW);
    digitalWrite(T_IN3, LOW);
    digitalWrite(T_IN4, HIGH);
    
    analogWrite(T_ENA, speed);
    analogWrite(T_ENB, speed);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    Tspeed = speed;
  } else if (speed < 0) {
    digitalWrite(T_IN1, LOW);
    digitalWrite(T_IN2, HIGH);
    digitalWrite(T_IN3, HIGH);
    digitalWrite(T_IN4, LOW);

    analogWrite(T_ENA, abs(speed));
    analogWrite(T_ENB, abs(speed));
    Tspeed = speed;
  } else if (speed == 0) {
    digitalWrite(T_IN1, LOW);
    digitalWrite(T_IN2, LOW);
    digitalWrite(T_IN3, LOW);
    digitalWrite(T_IN4, LOW);
    for (int i = 4; i < 8; i++) {
      motorState[i] = 0;
    }
    Tspeed = 0;
  }                 
}

// ISR untuk encoder 0
void updateEncoder0() {
  if (digitalRead(encA_pins[0]) == digitalRead(encB_pins[0])) {
    encoderPositions[0]++;
  } else {
    encoderPositions[0]--;
  }
}

// ISR untuk encoder 1
void updateEncoder1() {
  if (digitalRead(encA_pins[1]) == digitalRead(encB_pins[1])) {
    encoderPositions[1]--;
  } else {
    encoderPositions[1]++;
  }
}

// ISR untuk encoder 2
void updateEncoder2() {
  if (digitalRead(encA_pins[2]) == digitalRead(encB_pins[2])) {
    encoderPositions[2]--;
  } else {
    encoderPositions[2]++;
  }
}

// ISR untuk encoder 3
void updateEncoder3() {
  if (digitalRead(encA_pins[3]) == digitalRead(encB_pins[3])) {
    encoderPositions[3]++;
  } else {
    encoderPositions[3]--;
  }
}
void HoistDown() {
  digitalWrite(H_IN3, HIGH);
  digitalWrite(H_IN4, LOW);
}

void HoistUp() {
  digitalWrite(H_IN3, LOW);
  digitalWrite(H_IN4, HIGH);
}

void HoistStop() {
  digitalWrite(H_IN3, LOW);
  digitalWrite(H_IN4, LOW);
}

void SpreaderClose() {
  spreader.write(115);
}

void SpreaderOpen() {
  spreader.write(50);
}
void logDataCont(void *pvParameters){
  for(;;){
    unsigned long uptime = millis();
    Serial.print(uptime);
    Serial.print(',');
    for(uint8_t i=0 ; i < sensorCount ; i++){
      Serial.print(sensorValue[i]);
      Serial.print(',');
    }
    for(uint8_t i = 0; i < 4; i++){
      Serial.print(encoderPositions[i]);
      Serial.print(',');
    }
    Serial.print(valueX);
    Serial.print(',');
    Serial.print(valueY);
    Serial.print(',');
    Serial.println(angleX);
    Serial.print(',');
    Serial.print(angleY);
    Serial.print(',');
    Serial.print(Kp);
    Serial.print(',');
    Serial.print(Ki);
    Serial.print(',');
    Serial.print(Kd);
    Serial.print(',');
    Serial.print(Error);
    Serial.print(',');
    Serial.print(rawOutput);
    Serial.print(',');    
    Serial.print(Output);
    Serial.print(',');        
    Serial.print(Gspeed);
    Serial.print(',');
    Serial.println(Tspeed);
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

void DebugPID(void *pvParameters){
  for(;;){
  Serial.print("Kp=");
  Serial.print(Kp);
  Serial.print('\t');
  Serial.print("Ki=");
  Serial.print(Ki);
  Serial.print('\t');
  Serial.print("Kd=");
  Serial.print(Kd);
  Serial.print('\t');
  Serial.print("Raw Output=");
  Serial.print(rawOutput);
  Serial.print('\t');
  Serial.print("Output=");
  Serial.print(Output);
  Serial.print('\t');
  Serial.print("PWM=");
  Serial.print(Gspeed);
  Serial.print('\t');
  Serial.print("Input=");
  Serial.print(Input);
  Serial.print('\t');
  Serial.print("Error=");
  Serial.print(Error);
  Serial.print('\t');
  Serial.print("Setpoint=");
  Serial.print(Setpoint);
  Serial.println('\t');
  vTaskDelay(100 / portTICK_PERIOD_MS);    
  }
}

void logData(){
  for (uint8_t i=0 ; i < numData ; i++){
    for (uint8_t i=0 ; i < sensorCount ; i++){
      Serial.print(sensorValue[i]);
      Serial.print(',');
      if (sensors[i].timeoutOccurred()) {
        Serial.print(" TIMEOUT");
        break;
      }
    }
  }
}

void DisplayPM(void *pvParameters){
  for(;;){
  valueX = analogRead(potX);
  valueY = analogRead(potY);
  angleX = map(valueX, 170, 874, -90, 90);
  angleX = angleX - 3;
  angleY = map(valueY, 170, 874, -90, 90);
  Serial.print("X= ") ;
  Serial.print(valueX) ;
  Serial.print("\t"); 
  Serial.print("X DEGREE= ");
  Serial.print(angleX);  
  Serial.print(" Y= ") ;jjjj
  Serial.print(valueY) ;
  Serial.print("\t"); 
  Serial.print("X DEGREE= ");
  Serial.print(angleX);
  Serial.print("\t");
  Serial.print("Y DEGREE=");
  Serial.println(angleY);
  vTaskDelay(100 / portTICK_PERIOD_MS);
    }
  }

void autonomous(void *pvParameters) {
  const int targetEncoderPosition = 5000;  
  const int targetToFDistance = 300;  // Contoh jarak tujuan untuk ToF
  // Loop terus-menerus sampai kedua kondisi terpenuhi
  for (;;) {
    // Dapatkan jarak yang tersisa untuk ToF dan encoder
    int remainingToFDistance = sensorValue[1] - targetToFDistance;
    int remainingEncoderDistance = targetEncoderPosition - encoderPositions[0];

    updatePIDGains(sensorValue[0], angleX) ;
    computePID();
    
    // Gerakkan motor sesuai kecepatan yang ditentukan
    PID_G_Drive(Output);
//    G_Drive(90);
    T_Drive(120);

    // Cetak status untuk debugging
//    Serial.print("ToF Distance: ");
//    Serial.println(sensorValue[1]);
//    Serial.print("Encoder Position: ");
//    Serial.println(encoderPositions[1]);
    
    // Cek kondisi untuk menghentikan motor Gantry
    if (remainingToFDistance <= 20) {
      PID_G_Drive(0);
//      Setpoint = 0;
    }
    
    // Cek kondisi untuk menghentikan motor Trolley
    if (remainingEncoderDistance <= 0) {
      T_Drive(0);
    }
    
    // Jika kedua kondisi terpenuhi, hentikan task
    if (remainingToFDistance <= 0 && remainingEncoderDistance <= 0) {
//      PID_G_Drive(0);
      T_Drive(0);
//      break;
    }
    
    vTaskDelay(10 / portTICK_PERIOD_MS); // Delay kecil untuk memberikan waktu eksekusi untuk task lain
  }
  
  vTaskDelete(NULL); // Menghapus task setelah selesai
}

void computePID() {
  unsigned long now = millis();
  unsigned long timeChange = now - lastTime;
  Input = sensorValue[1];
  

  if (timeChange >= sampleTime) {
    Error = -1*(Setpoint - Input);
    Integral += Error * timeChange;

    if (Integral > integralMax) { //anti wind-up
        Integral = integralMax;
    } else if (Integral < integralMin) {
        Integral = integralMin;
    }

    Derivative = (Error - prevError) / timeChange;

    Output = Kp * Error + Ki * Integral + Kd * Derivative;
    prevError = Error;
    lastTime = now;
    rawOutput = Output;

    // Batasi Output
    if (Output > 230) Output = 230;
    else if (Output <= 90 && Output >= 11) Output = 100;
    else if (Output <= 10 && Output >= -10) Output = 0 ;
    else if (Output <  -10 && Output >= -90) Output =  - 100;    
    else if (Output < -230) Output = -230;

  }
}

double defuzzifyKp(double ruleOutput[], double ruleStrength[], int ruleCount) {
  double numerator = 0.0;
  double denominator = 0.0;

  for (int i = 0; i < ruleCount; i++) {
    numerator += ruleOutput[i] * ruleStrength[i];
    denominator += ruleStrength[i];
  }

  return (denominator == 0) ? 0 : numerator / denominator;
}

// Fungsi keanggotaan untuk jarak
double dekat(double x) {
  if (x <= 300) return 1.0;
  else if (x <= 500) return (500 - x) / 200.0;
  else return 0.0;
}

double sedang(double x) {
  if (x <= 300) return 0.0;
  else if (x <= 500) return (x - 300) / 200.0;
  else if (x <= 1000) return 1.0;
  else if (x <= 1200) return (1200 - x) / 200.0;
  else return 0.0;
}

double jauh(double x) {
  if (x <= 1000) return 0.0;
  else if (x <= 1200) return (x - 1000) / 200.0;
  else return 1.0;
}

// Fungsi keanggotaan untuk sudut deviasi
double kecil(double y) {
  if (y <= 5) return 1.0;
  else if (y <= 7) return (7 - y) / 2.0;
  else return 0.0;
}

double sedangSudut(double y) {
  if (y <= 5) return 0.0;
  else if (y <= 10) return (y - 5) / 5.0;
  else if (y <= 15) return (15 - y) / 5.0;
  else return 0.0;
}

double besar(double y) {
  if (y <= 13) return 0.0;
  else if (y <= 15) return (y - 13) / 2.0;
  else return 1.0;
}

double fuzzifyDistance(double distance) {
  if (distance < 300) return 1.0;  // Near
  else if (distance < 1000) return (1000 - distance) / 700.0;  // Medium
  else return 0.0;  // Far
}

double fuzzifyAngle(double angle) {
  if (angle < 5) return 1.0;  // Small
  else if (angle < 10) return (10 - angle) / 5.0;  // Medium
  else return 0.0;  // Large
}

void updatePIDGains(double distance, double angle) {
  double distanceFuzzyDekat = dekat(distance);
  double distanceFuzzySedang = sedang(distance);
  double distanceFuzzyJauh = jauh(distance);

  double angleFuzzyKecil = kecil(angle);
  double angleFuzzySedang = sedangSudut(angle);
  double angleFuzzyBesar = besar(angle);

  double ruleStrengthsKp[9];
  double ruleOutputsKp[9];

  // IF "JAUH" AND "BESAR" THEN Kp = 0.3
  ruleStrengthsKp[0] = min(distanceFuzzyJauh, angleFuzzyBesar);
  ruleOutputsKp[0] = 0.1;

  // IF "JAUH" AND "SEDANG" THEN Kp = 0.5
  ruleStrengthsKp[1] = min(distanceFuzzyJauh, angleFuzzySedang);
  ruleOutputsKp[1] = 0.5;

  // IF "JAUH" AND "KECIL" THEN Kp = 0.8
  ruleStrengthsKp[2] = min(distanceFuzzyJauh, angleFuzzyKecil);
  ruleOutputsKp[2] = 0.8;

  // IF "SEDANG" AND "BESAR" THEN Kp = 0.2
  ruleStrengthsKp[3] = min(distanceFuzzySedang, angleFuzzyBesar);
  ruleOutputsKp[3] = 0.1;

  // IF "SEDANG" AND "SEDANG" THEN Kp = 0.5
  ruleStrengthsKp[4] = min(distanceFuzzySedang, angleFuzzySedang);
  ruleOutputsKp[4] = 0.3;

  // IF "SEDANG" AND "KECIL" THEN Kp = 0.8
  ruleStrengthsKp[5] = min(distanceFuzzySedang, angleFuzzyKecil);
  ruleOutputsKp[5] = 0.5;

  // IF "DEKAT" AND "BESAR" THEN Kp = 0.1
  ruleStrengthsKp[6] = min(distanceFuzzyDekat, angleFuzzyBesar);
  ruleOutputsKp[6] = 0.1;

  // IF "DEKAT" AND "SEDANG" THEN Kp = 0.3
  ruleStrengthsKp[7] = min(distanceFuzzyDekat, angleFuzzySedang);
  ruleOutputsKp[7] = 0.1;

  // IF "DEKAT" AND "KECIL" THEN Kp = 0.4
  ruleStrengthsKp[8] = min(distanceFuzzyDekat, angleFuzzyKecil);
  ruleOutputsKp[8] = 0.3;

  // Defuzzifikasi untuk mendapatkan Kp
  Kp = defuzzifyKp(ruleOutputsKp, ruleStrengthsKp, 9);
  
}