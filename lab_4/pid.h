#ifndef PID_H
#define PID_H
class pid {
public:

float I, D, K, Ti, Td, b, h, y_old, N, bi, ad, bd, ao;
explicit pid( float _h, float _K = 1, float b_ = 1,
float Ti_ = 1, float Td_ = 0, float N_ = 10);
~pid() {};
float compute_control( float r, float y);
void change_control( float r, float y, float k_new, float b_new);
void init();
};
#endif