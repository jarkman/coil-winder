
#include <stdint.h>

//#include "robot.h"

// #define DEBUG

// Maximum step rate
//#define INTERRUPT_FREQUENCY 16000 //Anton had 8000
#define INTERRUPT_FREQUENCY 8000 


// For RAMPS 1.4, mapped to ATMeta2560
// See ./hardware/arduino/variants/mega/pins_arduino.h

/*****************************************************
#define X_STEP_PIN         6 // Port D, PD6 
#define X_DIR_PIN          8 // Port B, PB0 
#define X_ENABLE_PIN       7 // Port D, PD7 

*******************************************************/

#define SET_X_STEP() PORTD |= _BV(PD6)
#define CLR_X_STEP() PORTD &= ~_BV(PD6)


#define STEP_X() do { SET_X_STEP(); CLR_X_STEP(); } while (0)

#define SET_X_DIR() PORTB |= _BV(PB0)
#define CLR_X_DIR() PORTB &= ~_BV(PB0)

#define FORWARD_X() CLR_X_DIR()

#define BACKWARD_X() SET_X_DIR()

// Enables are active low on Pololu
#define ENABLE_X_MOTOR() PORTD &= ~_BV(PD7)
#define DISABLE_X_MOTOR() PORTD |= _BV(PD7)


// Position bit that is written to the STEP output
// Bit 0 steps at half the timer frequency, etc.
#define STEP_BIT (1 << 0)

// Call setup only once...
void setupTimerISR() {
  
  // Set step, direction and enable pins as output for 1 axis
  DDRD |= _BV(PD6) | _BV(PD7) ;
  DDRB |= _BV(PB0);

   
  cli(); // disable interrupts

  // Use 8-bit timer2 to drive steppers (timer 0 is used by millis())
  
  // Enable power to timer2
  //PRR0 &= ~(1 << PRTIM2); /// doesn't build for Nano
  
  // CTC mode: clear counter and interrupt when OCR0A is matched
  TCCR2A = (1 << WGM21);
  
  // prescaler divides clk by 64 (250 kHz tick) (timer0 differs here)
  TCCR2B = (1 << CS22);
  
  // set compare match register for frequency (value must be <256)
  OCR2A = (uint8_t)(16000000L / ((uint32_t) INTERRUPT_FREQUENCY * 64L) - 1L);
  
  TCNT2 = 0; //initialize counter value
  
  // enable timer compare interrupt
  TIMSK2 |= (1 << OCIE2A);

  sei(); // enable interrupts
}

#define MODE_POSITION 0
#define MODE_SPEED 1

volatile uint16_t stepperMode = MODE_SPEED;

volatile uint16_t xPosition = 16384;

volatile uint16_t xTarget = xPosition;
volatile int16_t xTicksSpeed;
volatile float xTicksSpeedFloat;
float xTicksRemainingFloat = 1; // number of ticks before next step is issued - float so we can do partial ticks
volatile int16_t xDirSpeed; // 1 or -1

// Table approximates a square root function (at constant acceleration, elapsed time goes as square root of distance).
// Acceleration increases towards the far end (non-linear), so that we reach max speed in a reasonable time.
#define ACCEL_STEPS 75
const uint8_t accelTab[] = {71,29,22,19,17,15,14,13,12,11,11,10,10,10,9,9,9,9,8,8,8,8,8,7,7,7,7,7,7,6,6,6,6,6,6,6,5,5,5,5,5,5,5,5,4,4,4,4,4,4,4,4,4,3,3,3,3,3,3,3,3,3,3,2,2,2,2,2,2,2,2,2,2,2,1};

// Accessed only within ISR, so no need for volatile
uint8_t xAccelStep = 0; // the acceleration step we are on, index into accelTab[]
uint8_t xTicksRemaining = 1; // number of ticks before next step is issued
uint8_t xTicksMin = 1; // minimum ticks before next step, use to set maximum speed


ISR(TIMER2_COMPA_vect) {

  if( stepperMode == MODE_SPEED )
  {
    if (--xTicksRemainingFloat <= 0.0) {
      xTicksRemainingFloat += xTicksSpeedFloat;
      if (xTicksRemainingFloat < xTicksMin) {
        xTicksRemainingFloat = xTicksMin; // respect the speed limit
      }
      
      if (xDirSpeed > 0) {
        FORWARD_X();
        
        ++xPosition;
        STEP_X();
  
      } else if (xDirSpeed < 0) {
        BACKWARD_X();
  
        --xPosition;
        STEP_X();
        
      }  
    }
  }
  
  if( stepperMode == MODE_POSITION)
  {
    if (--xTicksRemaining == 0) {
      uint16_t stepsToGo = xPosition > xTarget ? xPosition - xTarget : xTarget - xPosition;
      
      if (stepsToGo > xAccelStep) {
        // We still have enough room to decelerate, so continue to accelerate.
        if (xAccelStep < ACCEL_STEPS - 1) ++xAccelStep;
      } else {
        // Decelerate. Set the accelStep from stepsToGo (rather than decrementing), because
        // the target may have been changed.
        xAccelStep = stepsToGo > 0 ? stepsToGo - 1 : 0;
      }
      
      xTicksRemaining = accelTab[xAccelStep];
      if (xTicksRemaining < xTicksMin) {
        xTicksRemaining = xTicksMin; // respect the speed limit
      }
      
      if (xPosition < xTarget) {
        FORWARD_X();
        
        ++xPosition;
        STEP_X();
  
      } else if (xPosition > xTarget) {
        BACKWARD_X();
  
        --xPosition;
        STEP_X();
        
      }  
    }
  }
}

// Public API below here
// ---------------------

void enableX() {
  ENABLE_X_MOTOR();
}


void disableX() {
  DISABLE_X_MOTOR();
}


// The maximum speed available is INTERRUPT_FREQUENCY

void setMaxSpeedX(float stepsPerSec) {
  xTicksMin = ((float)INTERRUPT_FREQUENCY / stepsPerSec) + 0.5; // NB: truncation
}

void setSpeedX(float stepsPerSec) {
  if( fabs( stepsPerSec ) < 0.001 )
  {
    xTicksSpeedFloat = 0;
  
     xDirSpeed = 0;
  }
  else
  {
    xTicksSpeedFloat = ((float)INTERRUPT_FREQUENCY / fabs(stepsPerSec)) + 0.5; // float so no truncation
  
    if( stepsPerSec > 0 )
      xDirSpeed = 1;
    else
      xDirSpeed = -1;
  }
}


// Note: target updates that cause an instant change of direction
// will not accelerate or decelerate. Wait for motion to finish before doing this.

void moveToX(uint16_t target) {
  
  cli(); // ensure update is atomic
  xTarget = target;
  sei();
}


uint16_t getPositionX() {
  uint16_t result;
  cli();
  result = xPosition; // atomic access
  sei();
  return result;
}

void setPositionX(uint16_t x) {
  cli();
  xPosition = x; // atomic access
  sei();

}


bool isMotionDoneX() {
  boolean result;
  cli();
  result = (xPosition == xTarget);
  sei();
  return result;
}
