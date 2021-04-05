#include <Arduino.h>
#include <U8g2lib.h>
#include <PID_v1.h>

#define PWM_PIN    9
#define SENSE_PIN  3
#define SCREEN_SDA 4
#define SCREEN_SCL 5

// Screen variables
U8G2_SSD1306_128X32_UNIVISION_1_HW_I2C u8g2(U8G2_R0, SCREEN_SDA, SCREEN_SCL);

// RPM Sense variables
volatile uint16_t curVal  = 0;
volatile uint16_t prevVal = 0;
volatile long     rpm     = 0;
volatile long     rpmIIR  = 480;

// LED variables
bool ledOn = false;

// PID variables
double Setpoint, Input, Output;
double aggKp=4, aggKi=0.2, aggKd=1;
double consKp=1, consKi=0.05, consKd=0.25;
PID pwmPID(&Input, &Output, &Setpoint, consKp, consKi, consKd, DIRECT);


void setupPID() {
  Input = double(rpmIIR);
  Setpoint = 1000;
  pwmPID.SetMode(AUTOMATIC);
  pwmPID.SetOutputLimits(0, 255);
}

void updatePID() {
  Input = double(rpmIIR);
  double gap = abs(Setpoint-Input);
  if (gap < 20) {
    pwmPID.SetTunings(consKp, consKi, consKd);
  } else {
    pwmPID.SetTunings(aggKp, aggKi, aggKd);
  }
  pwmPID.Compute();
  Serial.println(Output);
  /* digitalWrite() */
}

void senseInterrupt() {
  curVal = TCNT1 - prevVal;
  prevVal = TCNT1;

  digitalWrite(LED_BUILTIN, ledOn);
  ledOn = !ledOn;

  long currentRPM = long(60.0f / ((128.0f * curVal) / 16000000.0f)) / 10 * 10;
  
  if (currentRPM > 3000) return;
  rpm = currentRPM;

  rpmIIR = ((rpm + rpmIIR) >> 1) / 10 * 10; 
}

void startTimer() {
  TCCR1A = 0;
  TCCR1B = _BV(CS11) | _BV(CS10);
  attachInterrupt(digitalPinToInterrupt(SENSE_PIN), senseInterrupt, RISING);
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(9200);
  u8g2.begin();
  startTimer();
  setupPID();
}

void printToScreen() {
  char printedText[10];
  char rpmText[10];

  sprintf(rpmText, "%4d RPM", (int) rpmIIR);

  u8g2.firstPage();
  do {
    /* u8g2.setFont(u8g2_font_crox4hb_tf); */
    /* u8g2.drawStr(0,20, printedText); */
    u8g2.setFont(u8g2_font_profont22_mf);
    u8g2.drawStr(34,14, rpmText);
  } while ( u8g2.nextPage() );
}

void loop() {
  updatePID();
  printToScreen();
}
