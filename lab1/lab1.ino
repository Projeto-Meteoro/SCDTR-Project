//BASIC IO
const int LED_PIN = 15; const int DAC_RANGE = 4096; int counter = 0;
const float Vcc = 3.3; const float R = 10000; const float b = 6.15;float rate = 0;

void setup() {
  pinMode(LED_PIN, OUTPUT);
  Serial.begin(115200);
  analogReadResolution(12);
}

void loop() {
  int read_adc;
  float v, R_ldr, lux;
  read_adc = analogRead(A0); // read analog voltages
  v = Vcc-read_adc*Vcc/DAC_RANGE;
  R_ldr = R*(Vcc-v)/v;
  lux = pow(pow(10, b)/R_ldr, 1.25);
  Serial.print(0); Serial.print(" "); Serial.print(R_ldr);
  Serial.print(" ");  Serial.print(DAC_RANGE); Serial.print(" ");   Serial.print(v);
  Serial.print(" ");  Serial.print(read_adc);  Serial.print(" ");  Serial.print(lux);
  Serial.print(" ");  Serial.print(counter);
  Serial.println();
  delay(1000);
  if(Serial.available() != 0){
    rate = Serial.parseInt(SKIP_ALL, '\n');
  }
  analogWrite(LED_PIN, rate);
}