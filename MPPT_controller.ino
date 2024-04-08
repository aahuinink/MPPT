/*

Program Description:

This program is intended to be used as a Maximum Power Point Tracker (MPPT)which generates a 100KHz PWM signal on digital pin 3 and automatically varies the PWM duty cycle to achieve maximum power at the load output.

100KHz example code from Arduino forum page at
https://forum.arduino.cc/index.php?topic=310753.0#/

This sets up timer 2 to run 100kHz on pin 3. It uses the OCR2A register to set the MAX value, so pin 11 is unavailable for PWM.

A similar thing can be done with timer1 (pins 9 and 10), and this timer allows both pins to PWM since the period can be set with ICR1 
(using a mode timer2 doesn't have).Timer 0 (pins 5 and 6) is used by millis(), delay() and micros() for timekeeping. This is why using Timer 1 or Timer 2 is preferred as it does not interfere with other peripheral processes in the Arduino.



Arduino digital pins can each only provide a maximum of 40mA at 5V.

*/

#define VL_PIN A0
#define IL_PIN A1
#define R_SENSE 
#define DC_BUFFER OCR2B

void setup()
{
  
  // configure hardware timer2 to generate a fast PWM on OC2B (Arduino digital pin 3)
  // set pin high on overflow, clear on compare match with OCR2B.
  TCCR2A = 0x23;
  
  TCCR2B = 0x09;  // select timer2 clock as unscaled 16 MHz I/O clock.
  
  OCR2A = 159;  // top/overflow value is 159 => produces a 100 kHz PWM (10us period).
  
  pinMode(3, OUTPUT);  // enable the PWM output (you now have a PWM signal on digital pin 3).

  pinMode(14, INPUT);   // Configuring analog A0 pin 14 as input. (0 to 1023 result).

  pinMode(A1, INPUT);   // configure analog A1 as input

  Serial.begin(9600);   // Open the serial port at 9600 bps.




}



void loop()
{
  // Define initial setup
    // Define initial duty cycle as 0%
  unsigned char dutyCycle = 0; // Initial PWM at digital pin 3 is a 0% duty cycle(0/160).
    // Define maximum power as 0
  int max_power = 0;
    // Define optimal duty cycle for maximum power
  int dc_max = 0;
    // load voltage & current adc values
  int v_load = 0;
  int i_load = 0;

    // load power value
  int power = 0;
  while(1)
  {
    // init values
    power = 0;
    max_power = 0;
    // cycle through 10 duty cycles to find optimal power output:
    for (dutyCycle = 0; dutyCycle < 161; dutyCycle += 16)
    {
      // Step 1: Set PWM duty cycle
      DC_BUFFER = dutyCycle;

      // let system stabilize
      delay(100);

      // Step 2: Measure load voltage and current
      v_load = analogRead(VL_PIN);
      i_load = analogRead(IL_PIN);

      // Step 3: calculate power (not converting to watts yet because thats not required to do the comparison and that would waste processor power).
      power = v_load*(v_load - i_load);

      // check if power is max power
      if (power > max_power)
      {
        dc_max = dutyCycle;
        max_power = power;
      }  
    }

    // after finding max power point and duty cycle calculate load power
    DC_BUFFER = dc_max;
    dc_max = dc_max/16;
    delay(100);
    double volts = map(0,1023,0,5.0,analogRead(VL_PIN));
    double current = map(0,1023,0,5.0,analogRead(IL_PIN));
    volts -= current;
    current = current/(float)R_SENSE;
    double watts = volts*current;

    Serial.printf("The maximum load power is: %.2fW\n",watts);
    Serial.printf("The duty cycle at max power is: %d%\n",dc_max);
    delay(5000); // sleep for 5 seconds
  }//close while(1)
}//close Void loop
