#include "pid.h"
pid::pid( float _h, float _K, float b_,
float Ti_, float Td_, float N_)
// member variable initialization list
: h {_h}, K {_K}, b {b_}, Ti {Ti_}, Td {Td_},
N {N_}, I {0.0}, D {0.0}, y_old{0.0}
{ 
  init();
} // should check arguments validity

void pid::init(){
  float Tt = 1;
  bi = K*h/Ti; //integral gain
  ad = Td/(Td+N*h);
  bd = K*N*Td/(Td+N*h); //derivative gains
  ao = h/Tt; //back calculation gain
}

float sat(float v, float ulow, float uhigh){
  if(v < 0) return 0;
  if(v > 4095) return 4095;
  return v;
}
float pid::compute_control( float r, float y ) {
  float P = K*(b*r-y); //proportional part
  D = ad*D-bd*(y-y_old);  //update derivative
  float v = P+I+D;  //temporary output
  float u = sat(v, 0, 4095); //saturate
  I = I +bi*(r-y)+ao*(u-v); //update integral
  y_old = y;
  return u;
}

void pid::change_control(float r, float y, float k_new, float b_new){
  float b_old = b;
  float k_old = K;
  b = b_new;
  K = k_new;
  init();
  I = I + k_old*(b_old*r-y)-K*(b*r-y);
}