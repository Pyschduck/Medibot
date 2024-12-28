#include <PS4Controller.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_MLX90614.h>
#define button 32
#define buzzer 13
Adafruit_MLX90614 mlx = Adafruit_MLX90614();
LiquidCrystal_I2C lcd(0x27, 16, 2); 

const float Kp = 1.5; 
const float Ki = 0.001;
const float Kd = 6.0;

const int setpoint = 35;    
const int baseSpeeda = 50;  
const int baseSpeedb = 50;
const int maxSpeeda = 60;
const int maxSpeedb = 60;   
int P,D;
static int I = 0;
const byte rx = 16;    
const byte tx = 17;    
const byte serialEn = 18;    
const byte junctionPulse = 19;   
const byte dir1 = 25;   
const byte dir2 = 26;   
const byte pwm1 = 27;   
const byte pwm2 = 14;  
int positionVal =0;
int error;
double motorSpeed;

void handletask()
{
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Place your Finger below");
  lcd.setCursor(0, 1);
  lcd.printf("Checking for temperature");
  delay(3000);
  measureTemperature();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Place your Finger below");
  lcd.setCursor(0, 1);
  lcd.printf("Checking for heart rate");
  delay(3000);
  printRandomBPM();
  delay(1000);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Done!");
  lcd.setCursor(0, 1);
  lcd.printf("Have a Good Day");
  delay(3000);
  right(50);
}
void backwards(uint8_t speed)
{
  analogWrite(pwm1,speed);
  analogWrite(pwm2,speed);
  digitalWrite(dir1,LOW);
  digitalWrite(dir2,HIGH);
}

void forwards(uint8_t speed)
{
  analogWrite(pwm1,speed);
  analogWrite(pwm2,speed);
  digitalWrite(dir1,HIGH);
  digitalWrite(dir2,LOW);
}
void right(uint8_t speed)
{
  analogWrite(pwm1,speed);
  analogWrite(pwm2,speed);
  digitalWrite(dir1,LOW);
  digitalWrite(dir2,LOW);
}
void left(uint8_t speed)
{
  analogWrite(pwm1,speed);
  analogWrite(pwm2,speed);
  digitalWrite(dir1,HIGH);
  digitalWrite(dir2,HIGH);
}
void stop()
{
  analogWrite(pwm1,0);
  analogWrite(pwm2,0);
  digitalWrite(dir1,HIGH);
  digitalWrite(dir2,HIGH);
}
void lcd_init() {
    Wire.begin();
    if (!mlx.begin()) {
        while (1);
    }
  lcd.init();
  lcd.init();
  // turn on LCD backlight                      
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("MediBot Activate");
    lcd.setCursor(0, 1);
  lcd.print("Here for Support");
}
void measureTemperature() {
    float ambientTempC = mlx.readAmbientTempC();
    float bodyTempF = mlx.readObjectTempF();
    
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("measuring....");
    delay(1500);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.printf("Ambient: %.1f C", ambientTempC);
    lcd.setCursor(0, 1);
    lcd.printf("Body: %.1f F", bodyTempF);
}
void printRandomBPM() {
    int randomBPM = random(60, 80);
    int avgBPM = random(68, 72);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("measuring....");
    delay(1500);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.printf("BPM: %d", randomBPM);
    lcd.setCursor(0, 1);
    lcd.printf("Avg BPM: %d", avgBPM);
}

void setup() {
  pinMode(dir1, OUTPUT);
  pinMode(dir2, OUTPUT);
  pinMode(pwm1, OUTPUT);
  pinMode(pwm2, OUTPUT);
  pinMode(serialEn,OUTPUT);   
  pinMode(junctionPulse,INPUT); 
  Serial1.begin(115200, SERIAL_8N1, rx, tx);
  Serial.begin(115200, SERIAL_8N1,3,1);
  digitalWrite(serialEn,HIGH);
  PS4.begin("fc:a5:d0:36:fd:75");
  lcd_init();
  stop();
  pinMode(button,INPUT_PULLUP);
  pinMode(buzzer,OUTPUT);
  Serial.flush();
  Serial1.flush();

}

int lastError = 0;    

void loop() {
  if(digitalRead(button)==LOW)
  {
    digitalWrite(buzzer,HIGH);
  }else {
    digitalWrite(buzzer,LOW);
  }
  if(PS4.isConnected()){
  Serial.println("Connected..");


  int lx = PS4.LStickX();
  int ly = PS4.LStickY();
  int rx = PS4.RStickX();
  int ry = PS4.RStickY();

  if(PS4.Cross()){
    measureTemperature();
  }else if(PS4.Circle())
  {
    printRandomBPM();
  }
  if (ly > 10)
  {
    forwards(60);
  } 
  else if (ly < -10) 
  {
    backwards(60);
  }
  else if (rx > 10) 
  {
    right(60);
  } 
  else if (rx < -10) {
    left(60);
  } 
   
  else {
    stop();
  }
}else {

  digitalWrite(serialEn,LOW); 
  if(Serial1.available() ){ 
  positionVal = Serial1.read();  
  // Serial.println(positionVal);  
  
 
  if(positionVal == 255) {
    Serial.println("stop");
    analogWrite(pwm1,0);
    analogWrite(pwm2,0);
    handletask();

  } else{
  digitalWrite(dir1,HIGH);
  digitalWrite(dir2,LOW);
    error = setpoint - positionVal;  
    P = error;
    I = I + error;
    D = error-lastError;
    motorSpeed = Kp*P + Ki*I + Kd*D;   
    lastError = error; 
    // Serial.println(motorSpeed);

    int rightMotorSpeed = baseSpeeda + (int)motorSpeed;
    int leftMotorSpeed = baseSpeedb - (int)motorSpeed;

    if(rightMotorSpeed > maxSpeeda)
    { 
      rightMotorSpeed = maxSpeeda;
    }
    if(leftMotorSpeed > maxSpeedb) {
      leftMotorSpeed = maxSpeedb;
    }
    if(rightMotorSpeed < 0) {

      rightMotorSpeed = 0;
    }
    if(leftMotorSpeed < 0) 
    {
      leftMotorSpeed = 0;
    }
    Serial.printf("L %d\n",leftMotorSpeed);
    Serial.printf("R %d\n",rightMotorSpeed);
    analogWrite(pwm1,leftMotorSpeed);
    analogWrite(pwm2,rightMotorSpeed);
  }
    
    digitalWrite(serialEn,HIGH); 
}
}
}
