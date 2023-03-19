#include <hardware/flash.h> //for flash_get_unique_id
#include "mcp2515.h"
uint8_t this_pico_flash_id[8], node_address;
struct can_frame canMsgTx, canMsgRx;
unsigned long counterTx {0}, counterRx {0};
MCP2515::ERROR err;
uint8_t unsigned long time_to_write, time_to_read;
unsigned long read_delay {10}, write_delay {1000};
MCP2515 can0 {spi0, 17, 19, 16, 18, 10000000};
void setup() {
flash_get_unique_id(this_pico_flash_id); //unique
node_address = this_pico_flash_id[7]; //maybe unique
Serial.begin();
can0.reset();
can0.setBitrate(CAN_1000KBPS);
can0.setNormalMode(); //setLoopbackMode() for debug
unsigned long current_time = millis();
time_to_write = current_time + write_delay;
time_to_read = current_time + read_delay;
}

void loop() {
unsigned long current_time = millis();
if( current_time >= time_to_write ) {
canMsgTx.can_id = node_address;
canMsgTx.can_dlc = 8;
//converts data from binary to text mode
unsigned long div = counterTx;
for( int i = 0; i < 8; i++ ) {
canMsgTx.data[7-i] = '0'+(int)(div%10);
div = div/10;
}
err = can0.sendMessage(&canMsgTx);
Serial.print("Sending message ");
Serial.print( counterTx );
Serial.print(" from node ");
Serial.println( node_address, HEX );
counterTx++;
time_to_write = current_time+write_delay;
}

if(current_time >= time_to_read )
{
err = can0.readMessage( &canMsgRx ) ;
if ( err == MCP2515::ERROR_OK)
{
Serial.print("Received message number ");
Serial.print( counterRx++ );
Serial.print(" from node ");
Serial.print( canMsgRx.can_id , HEX);
Serial.print(" : ");
//the message comes in text mode
for (int i=0 ; i < canMsgRx.can_dlc ; i++)
Serial.print((char) canMsgRx.data[ i ]);
Serial.println(" ");
}
time_to_read = current_time + read_delay;
}
}