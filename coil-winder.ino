/**
 * Underwood / Jarkman / State51 Coil winder
 * 
 * Uses a SilentStepstick stepper driver
 * https://uk.farnell.com/trinamic/tmc-silentstepstick-spi/stepper-driver-board-2-phase-motor/dp/2822154
 * datasheet
 * http://www.farnell.com/datasheets/2551906.pdf?_ga=2.160021501.1379882106.1561653689-1449061382.1561653689
 * 
 * library:
 * https://github.com/teemuatlut/TMC2130Stepper
 * see also
 * https://hackaday.com/2016/09/30/3d-printering-trinamic-tmc2130-stepper-motor-drivers-shifting-the-gears/

  Stepper is currently a 2A one:
  https://www.omc-stepperonline.com/nema-17-bipolar-59ncm-84oz-in-2a-42x48mm-4-wires-w-1m-cable-and-connector.html
  
  The stepper driver has a stick-on heatsink and a 40mm 5v fan, without that it has overheated more than once.
*/

// Set driver current by measuring vRef voltage and turning pot
// vRef is the furthest from the edge of the thee diag pins near the pot
// Or, set it in software like we do here

// Instructions are  https://learn.watterott.com/silentstepstick/faq/
// 200mA is 0.28V
// 400mA is 0.56

// Note: You also have to connect GND, 5V/VIO and VM.
//       A connection diagram can be found in the schematics.
#define EN_PIN    7 //enable (CFG6)
#define DIR_PIN   8 //direction
#define STEP_PIN  6 //step

#define CS_PIN   10 //chip select
#define MOSI_PIN 11 //SDI/MOSI (ICSP: 4, Uno: 11, Mega: 51 Nano: D11)
#define MISO_PIN 12 //SDO/MISO (ICSP: 1, Uno: 12, Mega: 50 Nano: D12)
#define SCK_PIN  13 //CLK/SCK  (ICSP: 3, Uno: 13, Mega: 52 Nano: D13)



bool dir = true;

#include <TMC2130Stepper.h>
#include <TMC2130Stepper_REGDEFS.h>

TMC2130Stepper driver = TMC2130Stepper(EN_PIN, DIR_PIN, STEP_PIN, CS_PIN);

#define MICROSTEPS 4


float currentSpeed = 0.0;
float targetSpeed = 200.0 * MICROSTEPS;
unsigned long lastUpdateMicros = 0;


#define MAX_RPS 4



long count_start = 0;
long loops = 0;
long ranges = 0;

int32_t stepOrigin = 0; //1;
int32_t stepRange = 0; //100.0 * 200.0 * (float) MICROSTEPS / (float) PULLEY_CIRCUMFERENCE; // about 5000

int brakingSteps = 10 * 200 * MICROSTEPS; 


bool trace = false;

void setup() {
	Serial.begin(115200);
	while(!Serial);
  Serial.println("");
  Serial.println("CoilWinder");
  Serial.println("");

  Serial.println("https://bitbucket.org/jarkman/acoustic-modular/src/master/CoilWinder/");
  Serial.println("");

  Serial.println("Enter a number to start winding, or return to stop");
	SPI.begin();

   
  setupDriver();

  setupTimerISR();
  //setMaxSpeedX(20000);
  setMaxSpeedX(200.0 * MICROSTEPS * MAX_RPS);
  enableX();

  

  setImmediateSpeed( 0 );

}


void setupDriver()
{
	pinMode(MISO, INPUT_PULLUP);
	driver.begin(); 			// Initiate pins and registeries
  
	driver.rms_current(600); 	// Set stepper current in mA. The command is the same as command TMC2130.setCurrent(600, 0.11, 0.5);
                            // Values higher than some limit (1200-ish) are ignored
                            // our usual motors are 400mA: https://www.ebay.co.uk/itm/Nema-17-Stepper-Motor-26Ncm-36-8oz-in-0-4A-12V-1m-Cable-for-3D-Pinter-Extruder/392155413077?ssPageName=STRK%3AMEBIDX%3AIT&_trksid=p2057872.m2749.l2649
                            
                               
  
	driver.stealthChop(1); 	// Enable extremely quiet stepping
  // turning this off gets us a bit more torque but also gets us a lot of singing when stopped

  // turn off for a bit more speed
  driver.stealth_autoscale(1);

  driver.microsteps(MICROSTEPS); 
  driver.intpol(true); // turn on microPlyer interpolation

  enableDriverOutputs( true );


}

void enableDriverOutputs(boolean on)
{
  //TMC2130 outputs on (LOW active)
  digitalWrite(EN_PIN, ! on);
}


float stepsPerSec = 0;
 
void loop() {

  loopInput();
  loopRun();
}



int32_t lastTurn = 0;
int turns = 0;
int targetTurns = 0;

void loopRun()
{

  long now = millis();
  
    if( count_start == 0 )
      count_start = now;
      
    loops ++;

       
 
    int32_t position = getPositionX(); // sneaky coercion to signed

    int32_t turn = position / (200 * MICROSTEPS);

   
    if( turn != lastTurn )
    {
      // wrapped
      turns ++;

      if( turns % 100 == 0 )
      {
        Serial.println( turns );
      }

      if( turns > targetTurns && targetSpeed > 0)
      {
        Serial.print("Stopping at ");
        Serial.println(turns);

        setTargetSpeed( 0 );
      }
    }

    lastTurn = turn;
    
   
    updateSpeed();

 


}

char buffer[100];
int buffPos = 0;

void loopInput()
{

  int n = Serial.available();
  
  if( n == 0 )
    return;

  boolean gotReturn = false;

  while( n > 0 )
  {
    char c = Serial.read();
    buffer[buffPos++] = c;
    buffer[buffPos] = '\0';
    n--;

    if( c == '\n' )
     {
      gotReturn = true;
      break;
     }
  }
  
  if( gotReturn )
  {
    float revsPerSec = 0;
    
    if( buffPos == 1 ) // hit return to stop
    {
      revsPerSec = 0;
      Serial.print("Interrupted at at ");
      Serial.println(turns);
    }
    else
    {
      int newTurns = atoi( buffer );

      
      if( newTurns > turns)
      {
        targetTurns = newTurns;
        Serial.print("Running until ");
        Serial.println(targetTurns);
        revsPerSec = MAX_RPS;
      }
    }
    
    revsPerSec = fconstrain(revsPerSec, 0, MAX_RPS);

  
        
    float stepsPerSec = 200.0 * MICROSTEPS * revsPerSec;

    setTargetSpeed( stepsPerSec );

    buffPos = 0; // consume input
  }
  
}




void setImmediateSpeed(float steps_per_sec)
{
 
  setTargetSpeed( steps_per_sec );
  currentSpeed = targetSpeed;
  setSpeedX(currentSpeed);
  lastUpdateMicros = micros();
//   Serial.print("immediate speed: ");
//  Serial.println(current_speed);

  //enableDriverOutputs(steps_per_sec > 0.1 );

  
}

void setTargetSpeed(float steps_per_sec)
{
  targetSpeed = steps_per_sec;
}


void updateSpeed()
{
  unsigned long now = micros();
  
  if( lastUpdateMicros == 0 )
  {
    // first go
    setImmediateSpeed(targetSpeed);
    return;
  }

  
  // v^2 = u^2 + 2 a S
  // a = u^2 / 2S

  float maxSpeed = MAX_RPS * 200.0 * MICROSTEPS;

  // calcualate acceleration we need in principle
  // currently 0.32
  float acc =  maxSpeed * maxSpeed / (2.0 * (float) brakingSteps)   / 1000000.0; // in steps per second per microsec

  // turns out to not be quite enough, need a bit more
  acc = 0.2;  // 1.0 found by experiment for 8mm braking distance, 400mA motor
  
  
  float dt = now - lastUpdateMicros;

  float oldSpeed = currentSpeed;
  
  if( targetSpeed > currentSpeed )
  {
    currentSpeed += dt * acc;
    if( currentSpeed > targetSpeed )
      currentSpeed = targetSpeed;
  }
  else
  {
    currentSpeed -= dt * acc;
    if( currentSpeed < targetSpeed )
      currentSpeed = targetSpeed;
  }

/*
  if( currentSpeed > maxSpeed )
      currentSpeed = maxSpeed;
      
   if( currentSpeed < - maxSpeed )
      currentSpeed = - maxSpeed;
 */ 

  if( false )
  {
    
    Serial.print("updateSpeed from: ");
    Serial.print(oldSpeed);
     Serial.print(" to: ");
    Serial.print(currentSpeed);
     Serial.print(" target: ");
    Serial.print(targetSpeed);
    Serial.print(" in ");
     Serial.println(dt);
  }
  setSpeedX(currentSpeed);
  lastUpdateMicros = now;

}

float fmapConstrained(float x, float in_min, float in_max, float out_min, float out_max)
{
  float f = fmap( x,  in_min, in_max, out_min, out_max);

  if( f < out_min )
    f = out_min;

  if( f > out_max )
    f = out_max;

  return f;
}

float fmap(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


float fconstrain(float f, float out_min, float out_max)
{
  if( f < out_min )
    f = out_min;

  if( f > out_max )
    f = out_max;

  return f;
}
