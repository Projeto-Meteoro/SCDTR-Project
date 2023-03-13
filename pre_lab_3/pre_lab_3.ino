#include "pid.h"
// constants
const int LED_PIN = 15; const int buffer_limit = 10000;
const int DAC_RANGE = 4096; int counter = 0;
const float Vcc = 3.3; const float R = 10000; const float b = 2.15; float rate = 0; 
const int p_max = 108;
// b =6.15

// variables
float lux;
int pwm;
float reference {5};
double energy;
float visibility_error;

//Create a pid controller
pid my_pid {0.01, 25, 0.1, 0.1};

// Buffers
int timer[buffer_limit];
double duty_cycle[buffer_limit];
float reference_buffer[buffer_limit];
float measured_illuminance[buffer_limit];

void setup() {
  analogReadResolution(12);
  Serial.begin(9600);
}

float volt2lux(int read_adc){
  float v, R_ldr;
  v = Vcc-read_adc*Vcc/DAC_RANGE;
  R_ldr = R*(Vcc-v)/v;
  lux = pow(R_ldr/pow(10, b), 1.25);
  return lux;
}

void calculate_energy(){
  energy = 0;
  for(int i = 1; i < counter; i++){
    energy = energy + duty_cycle[i-1]*(timer[i] - timer[i-1]);
  }
  Serial.print("energy: ");
  Serial.println(energy*p_max);
}

void calc_visibility_error(){
  visibility_error = 0;
  for(int i = 1; i < counter; i++){
      visibility_error += max(0, reference_buffer[i] - measured_illuminance[i]);
  }
  Serial.print("visibility_error:");
  Serial.println(visibility_error/counter);
} 


void readSerial(){
  String serial = Serial.readStringUntil('\n');
  
  char command = serial.charAt(0);
  String argument = serial.substring(2);
  switch (command) {
    case 'r':
      reference = argument.toFloat();
      Serial.println(reference);
      break;
    case 'g':
      if(argument == "e")
        calculate_energy();
      break;
    default:
      Serial.println("Invalid command");
      break;
  }
}

void loop() {
  if ( Serial.available() )
    readSerial();
  float y = volt2lux(analogRead(A0));
  float u = my_pid.compute_control(reference, y);
  pwm = (int)u;

  // update buffer
  timer[counter] = millis();
  duty_cycle[counter] =(double) pwm/DAC_RANGE;
  reference_buffer[counter] = reference;
  measured_illuminance[counter] = y;

  // update counter
  counter=counter+1;

    
  if(counter >= buffer_limit) {
    counter = 0;
  }

  /// plot
  //Serial.print("y:");
  //Serial.print(y);
  //Serial.print(y);
  //Serial.print(",pwm:");
  //Serial.print(pwm);
  //Serial.print(",reference:");
  //Serial.println(reference);


  analogWrite(LED_PIN, pwm);
  delay(10);
}