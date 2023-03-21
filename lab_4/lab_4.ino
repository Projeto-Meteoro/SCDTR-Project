
// Libraries
#include "pid.h"
#include <hardware/flash.h>
#include "mcp2515.h"

// Common
MCP2515 can0 {spi0, 17, 19, 16, 18, 10000000};
void msg_to_bytes(uint32_t msg, uint8_t * bytes) {
  bytes[0] = msg; bytes[1] = (msg >> 8);
  bytes[2] = (msg >> 16); bytes[3] = (msg >> 24);
}

// constants
const int LED_PIN = 15; const int buffer_limit = 6000;
const int DAC_RANGE = 4096; int counter = 0;
const float Vcc = 3.3; const float R = 10000; const float b = 2.15; float rate = 0; 
const int p_max = 108;

// variables
bool isAuto = true;
bool displayMessage = true;
float lux;
int pwm;
float y;
float reference {0};
double energy;
float visibility;
float flicker;

// temporary variables
// long jitter = 0;
// long serial_reading = 0;
// long serial_writing = 0;
// long metrics_delay = 0;

enum inter_core_cmds {
  //From core1 to core0: contains data read (16 bit)
  ICC_READ_DATA = 1,
  // From core0 to core1: contains data to write (16 bit)
  ICC_WRITE_DATA = 2,
  // From core1 to core0: contains regs CANINTF, EFLG
  ICC_ERROR_DATA = 3
};

char canintf_str[] {"MERRF | WAKIF | ERRIF | TX2IF | TX0IF | TX1IF | RX1IF | RX0IF | "};
char eflg_str [] {"RX1OV | RX0OV | TXBO | TXEP | RXEP | TXWAR | RXWAR | EWARN | "};

//Create a pid controller
pid my_pid {0.01, 25, 0.1, 0.1};

// Buffers and metrics
int timer[buffer_limit];
double duty_cycle[buffer_limit];
float reference_buffer[buffer_limit];
float measured_illuminance[buffer_limit];

// Arduino ID
uint8_t pico_flash_id[8];
uint8_t node_address;

/// run state machines () 
enum States { CONTROL, MESSAGE, WAIT_10MS };

States current_state = CONTROL;
bool waiting = false;
int current_time; 


void setup() {
  rp2040.idleOtherCore();
  //flash calls are unsafe if two cores are operating
  flash_get_unique_id(pico_flash_id);
  rp2040.resumeOtherCore();
  
  node_address = pico_flash_id[7];
  current_time = millis();
  analogReadResolution(12);
  Serial.begin(9600);
}


void loop(){
  read_events();
  run_state_machine();
}

void read_events(){
  if ( Serial.available() )
    readSerial();
  if( millis() - current_time > 10 ) waiting = true;
  if(current_state == CONTROL) control_system();
  else if(current_state == MESSAGE) control_message();
}

void read_message(){
  uint32_t msg;
  uint8_t b[4];
   //check incoming data in fifo
  if( rp2040.fifo.pop_nb(& msg) ) {
    msg_to_bytes(msg, b);
    if(b[3] == ICC_READ_DATA) {
      uint16_t val = msg;
      if(displayMessage == true) Serial.print("Received ");
      print_message(b[1], node_address, b[2], b[0] );
    }
    else if(b[3] == ICC_ERROR_DATA) {
      print_can_errors(b[1],b[0]);
      noInterrupts();
      can0.clearRXnOVRFlags();
      can0.clearInterrupts();
      interrupts();
    }
  }
}
void write_message(){
  uint8_t b[4];
  b[3] = ICC_WRITE_DATA;
  b[2] = node_address;
  b[0] = y;
  b[1] = pwm;
  rp2040.fifo.push(bytes_to_msg(b));
  if(displayMessage == true) Serial.print("Sending ");
  print_message(b[1], b[2], b[2], b[0]);
}
void control_message(){
  write_message();
  read_message();
}

void control_system(){
  y = volt2lux(analogRead(A0));
  float u = my_pid.compute_control(reference, y);
  pwm = (int)u;

  // update buffer
  timer[counter] = millis();
  current_time = timer[counter];
  duty_cycle[counter] =(double) pwm/DAC_RANGE;
  reference_buffer[counter] = reference;
  measured_illuminance[counter] = y;

  // unsigned long startTime = millis();
  // unsigned long endTime = millis();
  // jitter += (endTime - startTime); 

  // update counter
  counter=counter+1;

    
  if(counter >= buffer_limit) {
    counter = 0;
  }

  /// plot
  if(isAuto == true){
    // startTime = millis();
    Serial.print("time:");
    Serial.print(millis());
    Serial.print(",y:");
    Serial.print(y);
    Serial.print(",pwm:");
    Serial.print(0.01*pwm);
    Serial.print(",reference:");
    Serial.println(reference);
    // endTime = millis();
    // serial_writing = endTime - startTime;
  }


  analogWrite(LED_PIN, pwm);
}

void run_state_machine() {
  switch ( current_state )
  {
    case CONTROL:
      waiting = false;
      current_state = MESSAGE;
      break;
    case MESSAGE:
      current_state = WAIT_10MS;
      break;
    case WAIT_10MS:
      if( waiting ) {
        waiting = false;
        current_state = CONTROL;
      }
      break;
  }
}

// Metrics functions
float volt2lux(int read_adc){
  float v, R_ldr;
  v = Vcc-read_adc*Vcc/DAC_RANGE;
  R_ldr = R*(Vcc-v)/v;
  lux = pow(R_ldr/pow(10, b), 1.25);
  return lux;
}

void calculate_energy(){
  // unsigned long startTime = millis();
  energy = 0;
  for(int i = 1; i < counter; i++){
    energy = energy + duty_cycle[i-1]*(timer[i] - timer[i-1]);
  }
  Serial.print("energy: ");
  Serial.println(energy*p_max);
  unsigned long endTime = millis();
  // metrics_delay = endTime - startTime;
  // if(metrics_delay > 8 ){
  //   Serial.print(" counter:");
  //   Serial.print(counter);
  //   Serial.print(" ");
  // }
}

void calc_visibility_error(){
  visibility = 0;
  for(int i = 1; i < counter; i++){
      visibility += max(0, reference_buffer[i] - measured_illuminance[i]);
  }
  Serial.print("visibility_error:");
  Serial.println(visibility/counter);
} 

void calc_flicker(){
  flicker = 0;
  for(int i = 2; i-2 < counter; i++){
    float flicking = (duty_cycle[i]-duty_cycle[i-1])*(duty_cycle[i-1]-duty_cycle[i-2]);
    if( flicking< 0 ){
      flicker+= abs(duty_cycle[i]-duty_cycle[i-1])  + abs(duty_cycle[i-1]-duty_cycle[i-2]);
    }
  }
  Serial.print("flicker:");
  Serial.println(flicker/counter);
}


void readSerial(){
  // unsigned long startTime = millis();

  String serial = Serial.readStringUntil('\n');
  
  char command = serial.charAt(0);
  String argument = serial.substring(2);
  // unsigned long endTime = millis();

  // serial_reading = endTime - startTime;

  switch (command) {
    case 'r':
      reference = argument.toFloat();
      Serial.println(reference);
      break;
    case 'g':
      if(argument == "e") calculate_energy();
      if(argument == "v") calc_visibility_error();
      if(argument == "f") calc_flicker();
      break;
    case 's':
      isAuto = !isAuto;
      break;
    case 'm':
      displayMessage = !displayMessage;
      break;
    default:
      Serial.println("Invalid command");
      break;
  }
  // Serial.print("Jitter: ");
  // Serial.print(jitter);
  // Serial.print(",serial_writing: ");
  // Serial.print(serial_writing);
  // Serial.print(",serial_reading: ");
    
  // Serial.print(serial_reading);
  // Serial.print(", metrics_delay: ");
  // Serial.println(metrics_delay);
}

// Auxiliar functions
void print_message(int number, int node, int id, int val)
{
  if(displayMessage == true){
    Serial.print("pwm ");
    Serial.print( number );
    Serial.print(" at node " );
    Serial.print( node, HEX );
    Serial.print(" with id ");
    Serial.print( id, HEX );
    Serial.print(", y: ");
    Serial.println( val );
  }
}

void print_can_errors(uint8_t canintf, uint8_t eflg) {
  Serial.println( canintf_str );
  for (int bit = 7; bit >= 0; bit--) {
    Serial.print(" ");
    Serial.write( bitRead( canintf, bit ) ? '1' : '0' );
    Serial.print(" | ");
  }
  Serial.println(".");
  Serial.println(eflg_str);
  for (int bit = 7; bit >= 0; bit--) {
    Serial.print(" ");
    Serial.write(bitRead(eflg, bit) ? '1' : '0');
    Serial.print(" | ");
  }
  Serial.println(".");
} 


uint32_t bytes_to_msg(uint8_t * b) {
  uint32_t b0 {b[0]}, b1 {b[1]},b2 {b[2]},b3 {b[3]};
  return b0 + (b1 << 8) + (b2 << 16) + (b3 << 24);
}


//////////////////////////////////////////////////////////////////////// CORE 1 ////////////////////////////////////////////////////////////////////////
// Variables 
const uint8_t interruptPin {20};
volatile bool got_irq {false};

//the interrupt service routine
void read_interrupt(uint gpio, uint32_t events) {
  got_irq = true;
}

uint32_t can_frame_to_msg(can_frame * frm) {
  uint8_t b[4];
  b[3] = ICC_READ_DATA; b[2] = frm->can_id;
  b[1] = frm->data[1]; b[0] = frm->data[0];
  return bytes_to_msg(b);
}

uint32_t error_flags_to_msg(uint8_t canintf, uint8_t eflg) {
  uint8_t b[4];
  b[3] = ICC_ERROR_DATA; b[2] = 0;
  b[1] = canintf; b[0] = eflg;
  return bytes_to_msg(b);
}

void setup1(){ 
  can0.reset();
  can0.setBitrate(CAN_1000KBPS);
  can0.setNormalMode();
  gpio_set_irq_enabled_with_callback( interruptPin,
    GPIO_IRQ_EDGE_FALL,
    true,
    &read_interrupt 
  );
}

void loop1() {
  can_frame frm;
  uint32_t msg;
  uint8_t b[4];
  //reading the can-bus and writing the fifo
  if(got_irq) {
    got_irq = false;
    uint8_t irq = can0.getInterrupts();
    if(irq & MCP2515::CANINTF_RX0IF) {
      can0.readMessage( MCP2515::RXB0, &frm );
      rp2040.fifo.push_nb(can_frame_to_msg( &frm ) );
    }
    if(irq & MCP2515::CANINTF_RX1IF) {
      can0.readMessage(MCP2515::RXB1, &frm);
      rp2040.fifo.push_nb(can_frame_to_msg( &frm ) );
    }
    if( can0.checkError()) {
      uint8_t err = can0.getErrorFlags();
      rp2040.fifo.push_nb(error_flags_to_msg(irq, err));
    }
  }

  if( rp2040.fifo.pop_nb( &msg ) ) {//read fifo write bus
    msg_to_bytes( msg , b );
    if( b[3] == ICC_WRITE_DATA ) {
      frm.can_id = b[2];
      frm.can_dlc = 2;
      frm.data[1] = b[1];
      frm.data[0] = b[0];
      can0.sendMessage(&frm);
    }
  }
}

