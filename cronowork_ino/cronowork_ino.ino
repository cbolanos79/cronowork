/*
   cronowork_ino.ino

   Copyright (c) 2012 Cristo Saulo Bolaños Trujillo cbolanos@gmail.com
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions
   are met:
   1. Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
   2. Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
   3. Neither the name of copyright holders nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
   ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
   TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
   PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL COPYRIGHT HOLDERS OR CONTRIBUTORS
   BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
   CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
   SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
   INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
   CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
   ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
   POSSIBILITY OF SUCH DAMAGE.   
*/ 

#include <LiquidCrystal.h>
#include <Wire.h>
#include <ds1307.h>
#include <Bounce.h>
#include <MsTimer2.h>
#include <tone.h>

// Pines asignados al LCD
#define LCD_RED_PIN 11
#define LCD_GREEN_PIN 10
#define LCD_BLUE_PIN 9

#define RED 0
#define YELLOW 1
#define GREEN 2

// Pines asignados a botones
#define SET_BUTTON_PIN 8
#define CANCEL_BUTTON_PIN 4

// Tiempos para debounce
#define BOUNCE_TIME 30
#define REBOUNCE_TIME 200

#define SET_BUTTON 0
#define CANCEL_BUTTON 1
#define BUZZER_PIN 3

#define DEBUG

Bounce set_button_bounce(SET_BUTTON_PIN, BOUNCE_TIME);
Bounce cancel_button_bounce(CANCEL_BUTTON_PIN, BOUNCE_TIME);

LiquidCrystal lcd(17, 16, 15, 14, 13, 12);

// Bandera para controlar si un botón ha sido pulsado
volatile int pressed_button;

// Color de retroiluminación del LCD
volatile int color;

// Duración del botón pulsado
volatile bool button_long_press;
volatile bool long_press_processed;

#define POWER_SOURCE_BATTERY 0
#define POWER_SOURCE_USB 1

#define POWER_SOURCE_PIN 2

int power_source;

volatile struct _chrono {
  int hour;
  int minu;
  int sec;
  bool enabled;
} chrono;

static byte block[8] = {
  B11111,
  B11111,
  B11111,
  B11111,
  B11111,
  B11111,
  B11111,
  B11111,
};

void reset_chrono() {
  chrono.sec = 0;
  chrono.minu = 0;
  chrono.hour = 0;
  print_chrono();
  chrono.enabled = false;  
}

void start_chrono() {
  chrono.enabled = true;  
  MsTimer2::start();
  #ifdef DEBUG
    Serial.println("Arranca chrono");
  #endif
}

void stop_chrono() {
  chrono.enabled = false;
  MsTimer2::stop();
  #ifdef DEBUG
    Serial.println("Para chrono");
  #endif
}

void update_chrono() {

  chrono.sec++;
  if (chrono.sec>59) {
    chrono.sec = 0;
    chrono.minu++;
    
    // Imprime el estado de la batería a intervalos de 1 minuto
    print_battery_level();
    
    if (chrono.minu>59) {
      chrono.minu = 0;
      chrono.hour++;
        if (chrono.hour>23) {
          reset_chrono();
          chrono.enabled = true;
        }
    }
  }
  print_chrono();
}

void print_chrono() {
  char s[9];
  sprintf(s, "%02d:%02d:%02d", chrono.hour, chrono.minu, chrono.sec);
  lcd.setCursor(0, 1);
  lcd.print(s);
}

void print_job() {
  // TODO: poder tener diferentes tipos de trabajos (versión 0.6)
  lcd.setCursor(0, 0);
  lcd.print(F("Trabajo"));
}

void print_battery_level() {
  if (power_source == POWER_SOURCE_USB) {
    lcd.setCursor(13, 1);
    lcd.print(F("USB"));
    return;
  }
  
  lcd.setCursor(10, 1);
  lcd.print(F("B"));
  float aread = analogRead(4);
  
  // 3V -> 100%
  int blevel = map(aread, 0, 634, 0, 5);
  for (int i=0; i<blevel; i++)
    lcd.write(uint8_t(0));
 
}

void setColorRed() {
  pinMode(LCD_RED_PIN, OUTPUT);
  pinMode(LCD_GREEN_PIN, OUTPUT);
  pinMode(LCD_BLUE_PIN, OUTPUT);
  analogWrite(LCD_RED_PIN, 0);
  analogWrite(LCD_GREEN_PIN, 255);
  analogWrite(LCD_BLUE_PIN, 255);
}

void setColorGreen() {
  pinMode(LCD_RED_PIN, OUTPUT);
  pinMode(LCD_GREEN_PIN, OUTPUT);
  pinMode(LCD_BLUE_PIN, OUTPUT);
  analogWrite(LCD_RED_PIN, 255);
  analogWrite(LCD_GREEN_PIN, 0);
  analogWrite(LCD_BLUE_PIN, 255);
  
}

void setColorYellow() {
  pinMode(LCD_RED_PIN, OUTPUT);
  pinMode(LCD_GREEN_PIN, OUTPUT);
  pinMode(LCD_BLUE_PIN, OUTPUT);
  analogWrite(LCD_RED_PIN, 0);
  analogWrite(LCD_GREEN_PIN, 50);
  analogWrite(LCD_BLUE_PIN, 255);
}

void buzzer_beep(int duration=100) {
  tone(BUZZER_PIN, 4000, duration);
}

void print_start_screen() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("Cronowork 0.3"));
  lcd.setCursor(0, 1);
  lcd.print(F("cbolanos@gmail.com"));
  delay(3000);
  lcd.clear();
}

void setup() {
  lcd.begin(16, 2);
  
  color = GREEN;
  
  setColorGreen();
  print_start_screen();

  print_chrono();

  pinMode(SET_BUTTON_PIN, INPUT);
  pinMode(CANCEL_BUTTON_PIN, INPUT);

  lcd.createChar(0, block);
  print_job();
  
  // Comprueba si hay alimentación por USB (si no, es que tira de batería)
  pinMode(POWER_SOURCE_PIN, INPUT);
  if (digitalRead(POWER_SOURCE_PIN) == HIGH) {
    power_source = POWER_SOURCE_USB; 
  } else {
    power_source = POWER_SOURCE_BATTERY;
  }
  
  print_battery_level();

  /*  
    Reciclado de otro proyecto para la versión 0.5
  if ((digitalRead(SET_BUTTON_PIN) == HIGH) && (digitalRead(CANCEL_BUTTON_PIN) == HIGH)) {
    RTC.set(DS1307_SEC, 0);
    RTC.set(DS1307_MIN, 0);
    RTC.set(DS1307_HR, 0);
    RTC.set(DS1307_DOW, 6);
    RTC.set(DS1307_DATE, 15);
    RTC.set(DS1307_MTH, 1);
    RTC.set(DS1307_YR, 12);
    
    // Asocia la señal del rejo con la interrupción 0
    Wire.beginTransmission(DS1307_CTRL_ID);
    Wire.write((byte)0x07);
    Wire.write((byte)B10010000);
    Wire.endTransmission();
    
    // Comprobar si no interfiere con la interrupción de la libería de teclado PS/2 (versión 0.6)
    attachInterrupt(1, update_clock, RISING);
    clock_pulse = false;
  }
  */
  
  button_long_press = false;
  
  chrono.sec = 0;
  chrono.minu = 0;
  chrono.hour = 0;
  chrono.enabled = false;

  MsTimer2::set(1000, update_chrono);
  
  #ifdef DEBUG
    Serial.begin(9600);
    Serial.println(F("Arranca"));
  #endif
}

void loop() {
  
  pressed_button = -1;
  
  // Actualiza el estado de los botones
  set_button_bounce.update();
  cancel_button_bounce.update();
    
  // Pulsado el botón set (asignado al pulsador grande)
  if (set_button_bounce.fallingEdge()) {
    if (!button_long_press) { //(set_button_bounce.fallingEdge() && !button_long_press) {
      pressed_button = SET_BUTTON;
      
      if (!chrono.enabled) {
        setColorRed();
        start_chrono();
      } else {
        setColorYellow();
        stop_chrono();
      }
    }
    button_long_press = false;
    long_press_processed = false;
    // Cuando se pulsa el botón set mucho tiempo, se cambia el estado del display y resetea el cronómetro
  } else if (set_button_bounce.read() == HIGH) {
    if ((set_button_bounce.duration() >= 1000) && (!button_long_press)) {
      setColorGreen();
      reset_chrono();
      button_long_press = true;
      buzzer_beep();
      delay(100);
      buzzer_beep();
      stop_chrono();
      reset_chrono();
    }
  }

  // Cuando se pulsa un botón, lanza un pitido por el buzzer
  if (pressed_button>-1) {
    #ifdef DEBUG
      Serial.println(F("Pulsa botón"));
    #endif
    buzzer_beep();
  }
}
