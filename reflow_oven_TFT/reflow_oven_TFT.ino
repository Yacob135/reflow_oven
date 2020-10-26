#include <PID_v1.h>
#include <SPI.h>
#include "Adafruit_MAX31855.h"
#include "Adafruit_GFX.h"
#include "Adafruit_ILI9341.h"



//PID variables
double tempSetpoint, tempInput, dutyOutput;

//Specify the links and initial tuning parameters
double Kp = 2, Ki = 5, Kd = 1;
//Kp = 500, Ki = 375, Kd = 25;
PID myPID(&tempInput, &dutyOutput, &tempSetpoint, Kp, Ki, Kd, DIRECT);



#define MAXCS   7
Adafruit_MAX31855 thermocouple(MAXCS);


#define BLACK 0x0000
#define BLUE 0x001F
#define RED 0xF800
#define GREEN 0x07E0
#define CYAN 0x07FF
#define MAGENTA 0xF81F
#define YELLOW 0xFFE0
#define WHITE 0xFFFF

#define TFT_CLK A1
#define TFT_MISO A2
#define TFT_MOSI A3
#define TFT_DC 8
#define TFT_CS 10
#define TFT_RST 9
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_MOSI, TFT_CLK, TFT_RST, TFT_MISO);



uint8_t freqTOP = 5; //frequency in Hz for digital pin 4 of arduino - top heater of oven
uint8_t freqBOTTOM = 5; //frequency in Hz for digital pin 5 of arduino - bottom heater of oven
uint8_t freqBOOST = 5; //frequency in Hz for digital pin 6 of arduino - both booster heaters

uint8_t dutyTOP = 0; //duty cycle for digital pin 5 of arduino - setting 0 to 100
uint8_t dutyBOTTOM = 0; //duty cycle for digital pin 5 of arduino - setting 0 to 100
uint8_t dutyBOOST = 0; //duty cycle for digital pin 4 of arduino - setting 0 to 100

volatile uint32_t timeCounter = 0;
volatile int32_t stepCounter = 0;

uint8_t ovenMode = 0;


void constantTemperature(void);
void Sn60Pb40reflow(void);


void setup() {
  //timer1 interrupt on 1ms
  TCCR1A = 0;
  TCCR1B = 0;
  OCR1A = 249;
  TCCR1B |= (1 << CS11) | (1 << CS10) | (1 << WGM12);
  TIMSK1 |= (1 << OCIE1A);

  //external interrupt on pins 2 and 3 of arduino for rotary encoder
  attachInterrupt(0, ai0, RISING);
  //EICRA |= (1 << ISC00) | (1 << ISC01) | (1 << ISC11) | (1 << ISC10);
  //EIMSK |= (1 << INT1) | (1 << INT0);
  SREG |= (1 << 7);
  

  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(A0, INPUT);

  digitalWrite(4, LOW); //BOOST
  digitalWrite(5, LOW); //BOTTOM
  digitalWrite(6, LOW); //TOP

  Serial.begin(9600);
  
  tft.begin();
  tft.setRotation(1);
  tft.fillScreen(0x0000);

  while (digitalRead(A0)) {
    tft.setTextColor(0x07E0);
    tft.setTextSize(3);
    tft.setCursor(30, 30);
    tft.println("Choose mode:");
    static uint8_t i = 0;
    uint8_t changeLcd = 0;
    if (i != stepCounter) {
      if (i < stepCounter && ovenMode != 1) {
        ovenMode++;
        changeLcd = 1;
      }
      if (i > stepCounter && ovenMode != 0) {
        ovenMode--;
        changeLcd = 1;
      }
      i = stepCounter;
    }

    if (changeLcd) {
      switch (ovenMode) {
        case 0:
          tft.fillRect(30, 80, 300, 30, 0x0000);
          tft.setCursor(30, 80);
          tft.println("Sn60Pb40 reflow");
          break;
        case 1:
          tft.fillRect(30, 80, 300, 30, 0x0000);
          tft.setCursor(30, 80);
          tft.println("Constant temp");
          break;
      }
    }
  }

  while (!digitalRead(A0));
  delay(50);

  tft.fillRect(30, 30, 300, 30, 0x0000);
  tft.fillRect(30, 80, 300, 30, 0x0000);

  if (ovenMode == 1) {
    tempSetpoint = 49;
    while (digitalRead(A0)) {
      tft.setCursor(30, 30);
      tft.println("Set temp:");

      static uint8_t i = 0;
      uint8_t changeLcd = 0;
      if (i != stepCounter) {
        if (i < stepCounter && tempSetpoint <= 300) {
          tempSetpoint++;
          changeLcd = 1;
        }
        if (i > stepCounter && tempSetpoint >= 0) {
          tempSetpoint--;
          changeLcd = 1;
        }
        i = stepCounter;
      }

      if (changeLcd) {
        tft.fillRect(30, 80, 300, 30, 0x0000);
        tft.setCursor(30, 80);
        tft.println(tempSetpoint);
      }
    }
    tft.setCursor(30, 130);
    tft.println("Current temp:");

    tempInput = thermocouple.readCelsius();

    //turn the PID on
    myPID.SetMode(AUTOMATIC);
  }


}

void ai0() {
  // ai0 is activated if DigitalPin nr 2 is going from LOW to HIGH
  // Check pin 3 to determine the direction
  if (digitalRead(3) == LOW) {
    stepCounter--;
  } else {
    stepCounter++;
  }
}

void ai1() {
  // ai0 is activated if DigitalPin nr 3 is going from LOW to HIGH
  // Check with pin 2 to determine the direction
  if (digitalRead(2) == LOW) {
    stepCounter--;
  } else {
    stepCounter++;
  }
}

/*
ISR(INT0_vect) {
  static uint16_t localCount = 0;
  localCount++;
  if (localCount >= 30) {
    localCount = 0;
    if ((PIND & (1 << PIND3)) == 0) {
      stepCounter++;
    } else {
      stepCounter--;
    }
  }
}

ISR(INT1_vect) {
  static uint16_t localCount = 0;
  localCount++;
  if (localCount >= 30) {
    localCount = 0;
    if ((PIND & (1 << PIND2)) == 0) {
      stepCounter--;
    } else {
      stepCounter++;
    }
  }
}
*/
ISR(TIMER1_COMPA_vect) {
  static uint16_t msCounter = 0;
  static uint8_t outputStatusTOP = 0; //status of digital pin 4 (0 == LOW, 1 == HIGH)
  static uint8_t outputStatusBOTTOM = 0; //status of digital pin 5 (0 == LOW, 1 == HIGH)
  static uint8_t outputStatusBOOST = 0; //status of digital pin 6 (0 == LOW, 1 == HIGH)

  msCounter++; //millisecond counter
  timeCounter++; //overall time counter in milliseconds

  if (msCounter % (1000 / freqTOP) == 0) {
    if (dutyTOP / freqTOP > 0) {
      PORTD |= 1 << PORTD6;
      outputStatusTOP = 1;
    }
  }
  if (outputStatusTOP && msCounter % (1000 / freqTOP) == (dutyTOP / freqTOP) * 10) {
    PORTD &= ~(1 << PORTD6);
    outputStatusTOP = 0;
  }

  if (msCounter % (1000 / freqBOTTOM) == 0) {
    if (dutyBOTTOM / freqBOTTOM > 0) {
      PORTD |= 1 << PORTD5;
      outputStatusBOTTOM = 1;
    }
  }
  if (outputStatusBOTTOM && msCounter % (1000 / freqBOTTOM) == (dutyBOTTOM / freqBOTTOM) * 10) {
    PORTD &= ~(1 << PORTD5);
    outputStatusBOTTOM = 0;
  }

  if (msCounter % (1000 / freqBOOST) == 0) {
    if (dutyBOOST / freqBOOST > 0) {
      PORTD |= 1 << PORTD4;
      outputStatusBOOST = 1;
    }
  }
  if (outputStatusBOOST && msCounter % (1000 / freqBOOST) == (dutyBOOST / freqBOOST) * 10) {
    PORTD &= ~(1 << PORTD4);
    outputStatusBOOST = 0;
  }


  if (msCounter == 1000) {
    msCounter = 0;
  }
}

void loop() {
  if (ovenMode == 0) {
    Sn60Pb40reflow();
  }

  if (ovenMode == 1) {
    constantTemperature();
  }

  delay(10);
}

void Sn60Pb40reflow(void) {
  tft.setTextColor(0x07E0);
  tft.setTextSize(0);
  tft.setCursor(6, 226);
  tft.println("20");
  tft.setCursor(6, 196);
  tft.println("50");
  tft.setCursor(0, 146);
  tft.println("100");
  tft.setCursor(0, 96);
  tft.println("150");
  tft.setCursor(0, 46);
  tft.println("200");
  tft.setCursor(0, 0);
  tft.println("250");
  tft.setCursor(268, 0);
  tft.println("TEMP: ");
  tft.setCursor(268, 10);
  tft.println("TIME: ");
  
  tft.fillRect(20, 0, 1, 230, 0x07E0);
  tft.fillRect(20, 230, 300, 1, 0x07E0);
  tft.fillRect(21, 49, 300, 1, BLUE);
  tft.fillRect(21, 99, 300, 1, BLUE);
  tft.fillRect(21, 149, 300, 1, BLUE);
  tft.fillRect(21, 199, 300, 1, BLUE);
  tft.fillRect(80, 0, 1, 230, BLUE);
  tft.fillRect(140, 0, 1, 230, BLUE);
  tft.fillRect(200, 0, 1, 230, BLUE);
  tft.fillRect(260, 0, 1, 230, BLUE);
  
  tft.fillRect(50, 230, 1, 4, 0x07E0);
  tft.fillRect(80, 230, 1, 6, 0x07E0);
  tft.fillRect(110, 230, 1, 4, 0x07E0);
  tft.fillRect(140, 230, 1, 6, 0x07E0);
  tft.fillRect(170, 230, 1, 4, 0x07E0);
  tft.fillRect(200, 230, 1, 6, 0x07E0);
  tft.fillRect(230, 230, 1, 4, 0x07E0);
  tft.fillRect(260, 230, 1, 6, 0x07E0);
  tft.fillRect(290, 230, 1, 4, 0x07E0);


  tft.setCursor(268, 20);
  tft.println("PRE-HEAT");
  dutyTOP = 100;
  dutyBOTTOM = 100;
  dutyBOOST = 50;
  tempInput = thermocouple.readCelsius();
  Serial.println(tempInput);
  
  timeCounter = 0;

  uint32_t timeRefresh = 0;

  while (tempInput < 116) {
    static uint16_t lcdCount = 0;
    tempInput = thermocouple.readCelsius() + 6.0; //temperature compensation because of heating elements (6 degrees)
    lcdCount++;
    if (lcdCount == 100) {
      tft.fillRect(300, 0, 20, 20, BLACK);
      tft.setCursor(300, 0);
      tft.println((int)tempInput);
      tft.setCursor(300, 10);
      tft.println(timeCounter / 1000);
      lcdCount = 0;
    }

    if((timeCounter - timeRefresh) > 250){
      if(tempInput > 20) {
        tft.fillRect(20 + (timeCounter / 1000), 250 - (int)tempInput, 1, 1, 0x07E0);
      }
      timeRefresh = timeCounter;
    }
    
    delay(10);
  }

  tft.fillRect(268, 20, 50, 10, BLACK);
  tft.setCursor(268, 20);
  tft.println("SOAKING");
  dutyTOP = 0;
  dutyBOTTOM = 0;
  dutyBOOST = 0;

  delay(200);

  uint32_t currentTime = timeCounter;

  while (tempInput < 170 && (timeCounter <= (currentTime + 90000))) {
    static uint16_t lcdCount = 0;
    tempInput = thermocouple.readCelsius();
    lcdCount++;
    if (lcdCount == 100) {
      tft.fillRect(300, 0, 20, 20, BLACK);
      tft.setCursor(300, 0);
      tft.println((int)tempInput);
      tft.setCursor(300, 10);
      tft.println(timeCounter / 1000);
      lcdCount = 0;
    }

    if((timeCounter - timeRefresh) > 250){
      if(tempInput > 20) {
        tft.fillRect(20 + (timeCounter / 1000), 250 - (int)tempInput, 1, 1, 0x07E0);
      }
      timeRefresh = timeCounter;
    }
    
    delay(10);
  }

  tft.fillRect(268, 20, 50, 10, BLACK);
  tft.setCursor(268, 20);
  tft.println("REFLOW");
  dutyTOP = 100;
  dutyBOTTOM = 100;
  dutyBOOST = 100;

  delay(200);

  currentTime = timeCounter;

  while (tempInput < 215) {
    static uint16_t lcdCount = 0;
    tempInput = thermocouple.readCelsius() + 7.0; //temperature compensation because of heating elements (7 degrees)
    lcdCount++;
    if (lcdCount == 100) {
      tft.fillRect(300, 0, 20, 20, BLACK);
      tft.setCursor(300, 0);
      tft.println((int)tempInput);
      tft.setCursor(300, 10);
      tft.println(timeCounter / 1000);
      lcdCount = 0;
    }

    if((timeCounter - timeRefresh) > 250){
      if(tempInput > 20) {
        tft.fillRect(20 + (timeCounter / 1000), 250 - (int)tempInput, 1, 1, 0x07E0);
      }
      timeRefresh = timeCounter;
    }
    
    delay(10);
  }

  tft.fillRect(268, 20, 50, 10, BLACK);
  tft.setCursor(268, 20);
  tft.println("COMPLETE");
  dutyTOP = 0;
  dutyBOTTOM = 0;
  dutyBOOST = 0;

  delay(200);

  currentTime = timeCounter;

  while (1) {
    static uint16_t lcdCount = 0;
    tempInput = thermocouple.readCelsius();
    lcdCount++;
    if (lcdCount == 100) {
      tft.fillRect(300, 0, 20, 20, BLACK);
      tft.setCursor(300, 0);
      tft.println((int)tempInput);
      tft.setCursor(300, 10);
      tft.println(timeCounter / 1000);
      lcdCount = 0;
    }

    if((timeCounter - timeRefresh) > 250){
      if(tempInput > 20) {
        tft.fillRect(20 + (timeCounter / 1000), 250 - (int)tempInput, 1, 1, 0x07E0);
      }
      timeRefresh = timeCounter;
    }
    
    delay(10);
  }
}

void constantTemperature(void) {
  static uint16_t lcdCount = 0;
  static uint16_t faultCounter = 0;
  lcdCount++;
  tempInput = thermocouple.readCelsius();
  Serial.println(tempInput);
  static uint8_t tempFlag = 0;

  if (tempInput >= tempSetpoint) {
    tempFlag = 1;
  }
  if (tempFlag && (tempInput <= 10.0 || (tempInput >= tempSetpoint + 20.0 && timeCounter >= 100000))) {
    dutyTOP = 0;
    dutyBOTTOM = 0;
    dutyBOOST = 0;
    tft.fillScreen(0x0000);
    tft.setCursor(30, 30);
    tft.println("ERROR");

    if (tempInput <= 10.0) {
      tft.setCursor(30, 80);
      tft.println("TEMP TOO LOW");
    }
    if (tempInput >= tempSetpoint + 20.0) {
      tft.setCursor(30, 80);
      tft.println("TEMP TOO HIGH");
    }
    tft.setCursor(30, 130);
    tft.println("HEATING STOPPED");
    while (1);
  }

  if (lcdCount == 50) {
    Serial.println(tempInput);
    tft.fillRect(30, 180, 150, 22, 0x0000);
    tft.setCursor(30, 180);
    tft.println(tempInput);

    lcdCount = 0;
  }

  myPID.Compute();
  dutyTOP = dutyOutput;
  dutyBOTTOM = dutyOutput;
}
