
#include <Arduino.h>
#include <stdlib.h>
#include "hardware/pwm.h"
#include "hardware/irq.h"
#include "hardware/gpio.h"
#include <U8g2lib.h>
#include <PID_v1.h>
#include <Temperature_LM75_Derived.h>
#include <Wire.h>
#include "RP2040_PWM.h"
RP2040_PWM* PWM_Instance;
RP2040_PWM* PWM_Instance2;

#define RPM_PIN 0   //tachmeter pin
#define PWM_PIN 1   //4th fan pin
#define HEATER_PIN 22
#define ENCODER_A_PIN 21
#define ENCODER_B_PIN 20
#define ENCODER_BTN_PIN 16
#define KP_PIN 13
#define KI_PIN 12
#define KD_PIN 11

int page0_edit = 0; //0 for monitor, 123 for PID parameter edit.
int page1_edit = 0; //0 for Target set, 1 for manual mode, 2 for Heater Power, 3 for save settings. 
int display_page = 0;
int manual_mode = 0;
int RPM_Counter = 0;
int FAN_RPM;
int Heater_Random = 0;

float temp = 0;

double kd = 20;
double kp = 8;
double ki = 12;
double set, in, out;
double Target_Temp = 50.0;
double heater_power = 0;
double heater_power_random = 0; //random heat power mode
double edit_idle = 0;           //edit nothing

volatile double *edit = &edit_idle; //the value edited via encoder

volatile unsigned long time1 = 0;
volatile unsigned long time2 = 0;
volatile unsigned long time_rpm = 0;

double manual_pwm = 0;
double Temp_Array[128] = {20};

Generic_LM75_12Bit temperature; 
PID myPID(&in, &out, &set, kp, ki, kd, DIRECT);
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

void drawMenu();
void drawSetting();
void drawGraph();
void RPM_Count();

void in_Array(double input);

void Encoder_Change();
void Encoder_Push();
void KP_Set();  //button push IRSs
void KI_Set();
void KD_Set();

void setup() {
  pinMode(ENCODER_A_PIN, INPUT);
  pinMode(ENCODER_B_PIN, INPUT);

  gpio_set_dir(ENCODER_BTN_PIN, GPIO_IN);
  gpio_set_dir(KP_PIN, GPIO_IN);
  gpio_set_dir(KI_PIN, GPIO_IN);
  gpio_set_dir(KD_PIN, GPIO_IN);
  gpio_set_dir(RPM_PIN, GPIO_IN);
  gpio_pull_up(RPM_PIN);

  gpio_pull_up(ENCODER_BTN_PIN);
  gpio_pull_up(KP_PIN);
  gpio_pull_up(KI_PIN);
  gpio_pull_up(KD_PIN);
  gpio_pull_up(PWM_PIN);

  attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN), Encoder_Change, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_BTN_PIN), Encoder_Push, FALLING);
  attachInterrupt(digitalPinToInterrupt(KP_PIN), KP_Set, FALLING);
  attachInterrupt(digitalPinToInterrupt(KI_PIN), KI_Set, FALLING);
  attachInterrupt(digitalPinToInterrupt(KD_PIN), KD_Set, FALLING);
  attachInterrupt(digitalPinToInterrupt(RPM_PIN), RPM_Count, FALLING);

  u8g2_SetI2CAddress(&u8g2, 0x78);
  u8g2.begin();
  u8g2.enableUTF8Print();	

  in = temp;
  set = Target_Temp;
  myPID.SetMode(AUTOMATIC);

  time2 = millis();

  double buf1 = temperature.readTemperatureC();
  for(int i = 0; i < 128; i++){
      Temp_Array[i] = buf1;
  }

  PWM_Instance = new RP2040_PWM(PWM_PIN, 25000, 0);
  PWM_Instance2 = new RP2040_PWM(HEATER_PIN, 25000, 0);
}

void loop() {

  temp = temperature.readTemperatureC();
  if(millis() - time2 > 750){       //once per 0.75s for drawing
    in_Array(temp);
    time2 = millis();
        if(Heater_Random){
      srand((unsigned int) millis());
      heater_power_random = rand() % 255 + 0; //generate random heat power
    }
  }
  if(millis() - time_rpm > 1000){   //once per second calculate rpm
    FAN_RPM = 30 * RPM_Counter;
    RPM_Counter = 0;
    time_rpm = millis();
  }
  // smoothed temperature reading
  temp = (1.5 * temp + Temp_Array[1] + Temp_Array[2] + 0.5 * Temp_Array[3]) / 4;

  if (manual_pwm > 255){
    manual_pwm = 255;
  } else if(manual_pwm <= 0){
    manual_pwm = 0;
  }
  u8g2.clearBuffer();
  if(display_page == 0) drawMenu();
  if(display_page == 1) drawSetting();
  if(display_page == 2) drawGraph();
  u8g2.sendBuffer();
  if(!manual_mode){ //PID Auto Mode
    set = Target_Temp * 10;
    in = temp * 10;
    myPID.Compute();
    if(out > 255) out = 255;
    out = 255 - out; 
    if(out > 0) PWM_Instance->setPWM(PWM_PIN, 25000, out/2.55);
    else        PWM_Instance->setPWM(PWM_PIN, 25000, 0);
  } else {          //Manual Mode
    PWM_Instance->setPWM(PWM_PIN, 25000, manual_pwm/2.55);
  }
  if(Heater_Random) PWM_Instance2->setPWM(HEATER_PIN, 25000, heater_power_random/2.55);
  else PWM_Instance2->setPWM(HEATER_PIN, 25000, heater_power/2.55);
}

void drawMenu(){
    u8g2.setDrawColor(1);
    u8g2.drawBox(0, 0, 128, 11);
    u8g2.setCursor(2,10);
    u8g2.setFont(u8g2_font_6x13_me);
    u8g2.setDrawColor(0);
    u8g2.setFontMode(1);
    if(page0_edit == 0){
      u8g2.print("Status    ");
      if(manual_mode) {
        u8g2.setCursor(86,10);
        u8g2.print("*Manual");
      } else {
        u8g2.setCursor(96,10);
        u8g2.print("*Auto");
      }
    } else if(page0_edit == 1){
      u8g2.print("Edit Proportional");
    } else if(page0_edit == 2){
      u8g2.print("Edit Integral");
    } else if(page0_edit == 3){
      u8g2.print("Edit Differential");
    }
    char buff[10];
    
    u8g2.setDrawColor(1);
    u8g2.setCursor(0,23);
    u8g2.setFont(u8g2_font_9x18B_tr);
    u8g2.print("Temp:");
    u8g2.print("   ");
    u8g2.print("RPM:");

    u8g2.setCursor(0,44);
    u8g2.setFont(u8g2_font_crox4h_tf);
    sprintf(buff, "%.1f", temp);
    u8g2.print(buff);
    u8g2.print("°C");

    u8g2.setCursor(72,42);
    u8g2.print(FAN_RPM);
    u8g2.drawHLine(0,46,128);
    u8g2.setCursor(0,58);
    u8g2.setFont(u8g2_font_crox1hb_tr);

    if(page0_edit == 0){
      u8g2.print("Kp:");
      sprintf(buff, "%.1f", kp);
      u8g2.print(buff);
      u8g2.print("|");

      u8g2.print("Ki:");
      sprintf(buff, "%.1f", ki);
      u8g2.print(buff);
      u8g2.print("|");

      u8g2.print("Kd:");
      sprintf(buff, "%.1f", kd);
      u8g2.print(buff);

    } else if(page0_edit == 1){
      u8g2.print("Kp:");
      sprintf(buff, "%.10f", kp);
      u8g2.print(buff);

    } else if(page0_edit == 2){
      u8g2.print("Ki:");
      sprintf(buff, "%.10f", ki);
      u8g2.print(buff);

    } else if(page0_edit == 3){
      u8g2.print("Kd:");
      sprintf(buff, "%.10f", kd);
      u8g2.print(buff);
    }
}

void drawSetting(){
    u8g2.setDrawColor(1);
    u8g2.drawBox(0, 0, 128, 11);
    u8g2.setCursor(2,10);
    u8g2.setFont(u8g2_font_6x13_me);
    u8g2.setDrawColor(0);
    u8g2.setFontMode(1);
    u8g2.print("Settings");

    char buff[10];
    
    u8g2.setDrawColor(1);
    u8g2.setCursor(0,23);
    u8g2.setFont(u8g2_font_9x18B_tr);
    if(manual_mode == 0){
      u8g2.print("Target");
      u8g2.setCursor(0,44);
      u8g2.setFont(u8g2_font_crox4h_tf);
      sprintf(buff, "%.1f", Target_Temp);
      u8g2.print(buff);
      u8g2.print("°C");
    } else if(manual_mode == 1){
      u8g2.print("Fan:");
      u8g2.setCursor(0,44);
      u8g2.setFont(u8g2_font_crox4h_tf);
      sprintf(buff, "%.1f", manual_pwm/2.55);
      u8g2.print(buff);
      u8g2.print("%");
    }

    u8g2.setCursor(66,23);
    u8g2.setFont(u8g2_font_9x18B_tr);
    u8g2.print("Heater");
    u8g2.setCursor(66,44);
    u8g2.setFont(u8g2_font_crox4h_tf);
    if(heater_power >= 0) {
      Heater_Random = 0;
      sprintf(buff, "%.1f", heater_power/2.55);
      u8g2.print(buff);
      u8g2.print("%");
    }
    else {
      Heater_Random = 1;
      u8g2.setFont(u8g2_font_9x18B_tr);
      u8g2.print("Random");
    }
    if(page1_edit == 2){
      u8g2.setFont(u8g2_font_crox4h_tf);
      u8g2.setCursor(66,45);
      u8g2.print("______");
    }

    u8g2.setCursor(2,62);
    u8g2.setFont(u8g2_font_crox1hb_tr);
    if(manual_mode == 1){
      u8g2.drawBox(1, 52, 44, 12);
      u8g2.setDrawColor(0);
    }
    u8g2.print("Manual");
    u8g2.setDrawColor(1);
    u8g2.print("| ");
    u8g2.print("Heater");
    u8g2.print("| ");
    u8g2.print("Save");

}

void drawGraph(){
    u8g2.setDrawColor(1);
    u8g2.drawBox(0, 0, 128, 11);
    u8g2.setCursor(2,10);
    u8g2.setFont(u8g2_font_6x13_me);
    u8g2.setDrawColor(0);
    u8g2.setFontMode(1);
    u8g2.print("Temp Graph");
    u8g2.setDrawColor(1);

    float Temp_Max = Temp_Array[0];
    float Temp_Min = Temp_Array[0];

    for(int i = 1; i < 128; i++){
      if(Temp_Array[i] > Temp_Max) Temp_Max = Temp_Array[i];
    }

    for(int i = 1; i < 128; i++){
      if(Temp_Array[i] < Temp_Min) Temp_Min = Temp_Array[i];
    }
    float Temp_Range = Temp_Max - Temp_Min;
    if(Temp_Range < 0.2) Temp_Range = 0.2;
    float Graph_Ratio = 40/(Temp_Range);
    float Buffer_Array[128];
    for(int i = 0; i < 128; i++){
      Buffer_Array[i] = (Temp_Array[i] - Temp_Min + 0.5) * Graph_Ratio;
    }
    for(int i = 127; i >= 0; i--){
      if(Buffer_Array[i]>12) u8g2.drawPixel(127 - i, 62 - Buffer_Array[i]);
    }
}

void in_Array(double input){
  for(int i = 127; i > 0; i--){
    Temp_Array[i] = Temp_Array[i - 1];
  }
  Temp_Array[0] = input;
}

void Encoder_Change(){    //ratate faster means more increment
  double ratio = 1;
  if (time1 == 0){
    time1 = millis();
  } 
    ratio = millis() - time1;
    if (ratio < 10) ratio = 9;
    if (ratio > 100) ratio = 156;
    ratio = 160 - ratio;
    ratio = ratio/20;
    if(display_page == 0 && page0_edit != 0) ratio = 0.1 * ratio;

  if(digitalRead(ENCODER_A_PIN) == LOW){
    if(digitalRead(ENCODER_B_PIN) == HIGH){
      *edit = *edit + ratio;
    } else {
      *edit = *edit - ratio;
    }
  } else{
    if(digitalRead(ENCODER_B_PIN) == HIGH){
      *edit = *edit - ratio;
    } else {
      *edit = *edit + ratio;
    }
  }
  time1 = millis();
  if(manual_pwm < 0) manual_pwm = 0;
  if(manual_pwm > 255) manual_pwm = 255;
  if(heater_power < -2.55) heater_power = -2.55;
  if(heater_power> 255) heater_power = 255;
}

void Encoder_Push(){    //toggle between pages
    display_page++;
    if (display_page >2)
    {
      display_page = 0;
      page0_edit = 0;
      edit = &edit_idle;
    }
    if (display_page == 1){

      if(manual_mode) {
        edit = &manual_pwm;
      }else{
        edit = &Target_Temp;
      }
      page1_edit = 0;
    } 
}

void KP_Set(){
  if (display_page == 0){
      if (page0_edit == 1){
    page0_edit = 0;
  } else {
    page0_edit = 1;
    edit = &kp;
    }
  } else if(display_page == 1){
      if (page1_edit == 1){
    page1_edit = 0;
    manual_mode = 0;
    edit = &Target_Temp;
  } else {
    page1_edit = 1;
    manual_mode = 1;
    edit = &manual_pwm;
    }
  }
}

void KI_Set(){
  if (display_page == 0){
    if (page0_edit == 2){
      page0_edit = 0;
    } else {
      page0_edit = 2;
      edit = &ki;
    }
  } else if(display_page == 1){
      if (page1_edit == 2){
    page1_edit = 0;
    if(manual_mode) edit = &manual_pwm;
    if(!manual_mode) edit = &Target_Temp;
  } else {
    page1_edit = 2;
    edit = &heater_power;
    }
  }
}

void KD_Set(){
  if(display_page == 0){
    if (page0_edit == 3){
    page0_edit = 0;
  } else {
    page0_edit = 3;
    edit = &kd;
  }
  } else if(display_page == 1){
  }
}

void RPM_Count(){
  RPM_Counter++;
}
