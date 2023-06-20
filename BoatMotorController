#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "GyverButton.h"
#include "GyverTimer.h" 

#define MODE_BUTTON_PIN D6
#define POWER_BUTTON_PIN D8
#define RELAY_PIN D7
#define VOLTAGE_SENSOR_PIN A0 


#define MOTOR_IS_ON motor_started_time>0
#define OFF 0
#define ON 1

GTimer displayRefreshTimer(MS, 1000);  // refreshed every second
GButton modeBtn(MODE_BUTTON_PIN, HIGH_PULL, NORM_OPEN);
GButton powerBtn(POWER_BUTTON_PIN, HIGH_PULL, NORM_OPEN);


// On NODE MCU board  display is attached to: D2 - SDA, D1 - SCL
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels
#define OLED_RESET    -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

int off_timeout = 1 * 60 * 1000; // time after which motor is turned off, milliseconds
int on_timeout = 10 * 60 * 1000; // time after which motor can be turned on, milliseconds
int system_on = 1;
unsigned long motor_started_time = 0; //time when motor has started. milliseconds
unsigned long motor_off_time = 0; //time when WE(!) switched motor off. milliseconds


void setup() {
  Serial.begin(9600);
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
  pinMode(RELAY_PIN, OUTPUT); 
  switchMotor(OFF);
  redraw();
}

void loop() {
  modeBtn.tick();
  powerBtn.tick();

  if (system_on == 0) { // system OFF
    if (powerBtn.isClick()) {
      systemOn();
    }
  }
  
  else {                // system ON
    if (powerBtn.isClick()) {
      Serial.println(F("System switched off"));
      systemOff();
    } 
    else if(modeBtn.isClick()) {
      Serial.println(F("Timeout changed"));
      changeOffTimeout();
    } 
    else if (MOTOR_IS_ON) {
      if (!motorHasVoltage()) {
        motor_off_time = 0;
        switchMotor(OFF);
        Serial.println(F("Motor was on, but no voltage detected. Went off."));
      } 
      else if (motorIsHot()) {
        motor_off_time = millis();
        switchMotor(OFF);
        Serial.println(F("Motor was on, but went off by timeout"));
      }
    }
    else if (!MOTOR_IS_ON) {
      if (motorHasVoltage() && motorIsCool()) {
        switchMotor(ON);
        Serial.println(F("Motor was off, but went on because voltage detected and it's cool"));
      }
    }

    if (displayRefreshTimer.isReady()) {
      redraw();
    }
  }
}

void redraw() {
  display.clearDisplay();

  if (motor_off_time) {
    // stop sign and countdown
    display.drawCircle(display.width()/2 -28, display.height()/2, display.height()/2-1, SSD1306_WHITE);
    display.fillRect(display.width()/2 - 35, display.height()/2 - 2, 14, 4, SSD1306_WHITE);
    display.setTextSize(3);
    display.setCursor(display.width()/2, 8);
    display.print((on_timeout - millis() + motor_off_time)/1000);
  } else { // draw timeout in minutes

    display.setCursor(0,0);
    display.setTextSize(5);
    display.setTextColor(SSD1306_WHITE);
    display.print(off_timeout/60/1000);
  }

  if (MOTOR_IS_ON) {
    // lightening sign
    //display.drawLine(display.width()/2-5, 0, display.width()/2 - 9, display.height()/2, SSD1306_WHITE);
    //display.drawLine(display.width()/2-4, 0, display.width()/2 - 9, display.height()/2, SSD1306_WHITE);
    //display.drawLine(display.width()/2 - 9, display.height()/2, display.width()/2 -2 , display.height()/2, SSD1306_WHITE);
    //display.drawLine(display.width()/2 -2, display.height()/2, display.width()/2 + 3, display.height(), SSD1306_WHITE);
    //display.drawLine(display.width()/2 -1, display.height()/2, display.width()/2 + 4, display.height(), SSD1306_WHITE);

    display.setTextSize(2);
    display.setCursor(display.width()/2 - 32, 0);
    display.print("ON ");
    display.setTextSize(3);
    display.print((off_timeout - millis() + motor_started_time)/1000);
  }


  display.display();
  displayRefreshTimer.start();
}

void switchMotor(int isOn) {
  if (isOn == ON) {
    motor_off_time = 0;
    motor_started_time = millis();
  } else {
    motor_started_time = 0;
  }
  digitalWrite(RELAY_PIN, isOn); 
}

bool motorIsCool() {
  bool neverWasOn = motor_started_time == 0 && motor_off_time == 0;
  bool cooledDown = millis() - (motor_off_time) > on_timeout;
  return neverWasOn || cooledDown;
}

bool motorIsHot() {
  return millis() - (motor_started_time) > off_timeout;
}
bool motorHasVoltage() {
  return analogRead(VOLTAGE_SENSOR_PIN) > 50; // ADC has 1024 values. 
}

void systemOn() {
  system_on = 1;
  redraw();
}

void systemOff() {
  motor_started_time = 0;
  motor_off_time = 0;
  system_on = 0;
  display.clearDisplay();
  display.display();
}

void changeOffTimeout() {
  if (off_timeout == 5*60*1000) off_timeout = 60 *1000;
  else off_timeout += 2*60*1000;
  redraw();
}
