#include <Arduino.h>
#include <AccelStepper.h>

// Flag for speed selection
// 0 - off
// 1 - low
// 2 - mid
// 3 - high
uint8_t speedFlag = 0;


// Stepper setup
const uint8_t dir = 11;
const uint8_t step = 12;
AccelStepper stepper(AccelStepper::FULL2WIRE, 11, 12);

// PushButton setup
const uint8_t increase_speed_pin = 2;
const uint8_t decrease_speed_pin = 3;

int buttonState_inc;             // the current reading from the input pin
int buttonState_dec;             // the current reading from the input pin
int lastButtonState_inc = LOW;   // the previous reading from the input pin
int lastButtonState_dec = LOW;   // the previous reading from the input pin

// the following variables are unsigned longs because the time, measured in
// milliseconds, will quickly become a bigger number than can be stored in an int.
unsigned long lastDebounceTime_inc = 0;  // the last time the output pin was toggled
unsigned long lastDebounceTime_dec = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 80;    // the debounce time; increase if the output flickers


// Function headers
void increase_speed_handler();
void decrease_speed_handler();
void print_flag_non_blocking(int ms);
void change_flag(void);

void setup() {

  // Setup buttons
  pinMode(increase_speed_pin, INPUT);
  pinMode(decrease_speed_pin, INPUT);
  
  // Setup stepper motor
  stepper.setMaxSpeed(1000);
  stepper.setSpeed(0);

  Serial.begin(115200);
  Serial.println("Serial communication started");
}

void loop() {

    // Read button states
    int reading_inc = digitalRead(increase_speed_pin);
    int reading_dec = digitalRead(decrease_speed_pin);

    // Reset debouncing timer
    if (reading_inc != lastButtonState_inc)
        lastDebounceTime_inc = millis();

    if (reading_dec != lastButtonState_dec)
        lastDebounceTime_dec = millis();


    // Check inc reading
    if ((millis() - lastButtonState_inc) > debounceDelay) {
        // whatever the reading is at, it's been there for longer than the debounce
        // delay, so take it as the actual current state:

        // if the button state has changed:
        if (reading_inc != buttonState_inc) {
            buttonState_inc = reading_inc;

        // call increase speed if the new button state is HIGH
        if (buttonState_inc == HIGH) {
            increase_speed_handler();
            }
        }
    }

    // Check dec reading
    if ((millis() - lastButtonState_dec) > debounceDelay) {
        // whatever the reading is at, it's been there for longer than the debounce
        // delay, so take it as the actual current state:

        // if the button state has changed:
        if (reading_dec != buttonState_dec) {
            buttonState_dec = reading_dec;

        // only toggle the LED if the new button state is HIGH
        if (buttonState_dec == HIGH) {
            decrease_speed_handler();
            }
        }
    }

    // Save the readings
    lastButtonState_inc = reading_inc;
    lastButtonState_dec = reading_dec;

    // Change stepper speed
    change_flag();   
    //print_flag_non_blocking(5000);

    // Save the readings
    lastButtonState_inc = reading_inc;
    lastButtonState_dec = reading_dec;
  
}

void print_flag_non_blocking(int ms)
{
  unsigned long time_now = 0;
  int period = 100;

  time_now = millis();

  while(millis() < time_now + period){}
  Serial.print("SpeedFlag : ");
  Serial.print(speedFlag);
  Serial.println();
}

void change_flag(void){
  // Change stepper speed due to the speed flag
  if (speedFlag == 0)
  {
    stepper.setSpeed(0);
  }
  else if(speedFlag == 1)
  {
    stepper.setSpeed(200);
  }
  else if(speedFlag == 2)
  {
    stepper.setSpeed(500);
  }
  else if(speedFlag == 3)
  {
    stepper.setSpeed(1000);
  }
  else
  {
    stepper.setSpeed(0);
  }

    stepper.runSpeed();
}


// Interrupt Handlers
void increase_speed_handler()
{
  if(speedFlag <= 2 )
    speedFlag++;
}

void decrease_speed_handler()
{
  if(speedFlag > 0)
    speedFlag--;
}