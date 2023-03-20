#include <hardware/flash.h>
#include "mcp2515.h"

// Common
void msg_to_bytes(uint32_t msg, uint8_t * bytes) {
  bytes[0] = msg; bytes[1] = (msg >> 8);
  bytes[2] = (msg >> 16); bytes[3] = (msg >> 24);
}



// Core 0
  // Variables
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


  // Functions
void print_message(int number, int node, int id, int val)
{
  Serial.print(" number ");
  Serial.print( number );
  Serial.print(" at node " );
  Serial.print( node, HEX );
  Serial.print(" with id ");
  Serial.print( id, HEX );
  Serial.print(" : ");
  Serial.println( val );
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

uint8_t pico_flash_id[8];
uint8_t node_address;
unsigned long counterTx{0}, counterRx{0};
unsigned long time_to_write;
unsigned long write_delay {1};

void setup() {
  rp2040.idleOtherCore();
    //flash calls are unsafe if two cores are operating
    flash_get_unique_id(pico_flash_id);
  rp2040.resumeOtherCore();
  node_address = pico_flash_id[7];
  Serial.begin();
  time_to_write = millis() + write_delay;
}

void loop() {
  can_frame frm;
  uint32_t msg;
  uint8_t b[4];
  if(millis() >= time_to_write ) {
    b[3] = ICC_WRITE_DATA;
    b[2] = node_address;
    b[0] = counterTx;
    b[1] = (counterTx >> 8);
    rp2040.fifo.push(bytes_to_msg(b));
    Serial.print("Sending ");
    print_message(counterTx, b[2], b[2], counterTx);
    counterTx++;
    time_to_write = millis() + write_delay;
  }
  //check incoming data in fifo
  if( rp2040.fifo.pop_nb(& msg) ) {
    msg_to_bytes(msg, b);
    if(b[3] == ICC_READ_DATA) {
      uint16_t val = msg;
      Serial.print("Received ");
      print_message(counterRx, node_address, b[2], val );
      counterRx++;
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

// Core 1
// Variables 
MCP2515 can0 {spi0, 17, 19, 16, 18, 10000000};
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

