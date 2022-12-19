#define T1_IN1 12
#define T1_IN2 14
#define T1_EN 13

#define T2_IN1 25
#define T2_IN2 26
#define T2_EN 27

#define T1_ENCA 34
#define T1_ENCB 15

#define T2_ENCA 39
#define T2_ENCB 4

int T1_pulse = 0;
int T2_pulse = 0;

const int freq = 50;
const int led_channel_1 = 0;
const int led_channel_2 = 1;
const int resolution = 8;
const int PPR = 620;

// Constanta for sampling time
const double dt = 0.05;
const double f = 1/dt;
volatile double pulse_count = 0;
volatile double count = 0;

// Parameter for position control
volatile double q_init = 0;
volatile double q_final = 0;
volatile double q_res = 0;
volatile double q_res_old = 0;
volatile double q_cmd = 0;
volatile double err = 0;
volatile double err_old = 0;
volatile double integral_err = 0;
volatile double duty_cycle = 0;

// PID Parameter
float kp = 5;
float kd = 1;
float ki = 2;

void isr_T1();
void isr_T2();

void IRAM_ATTR onTimerMain();

// Make timer
hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  
  // Set pinmode for T1 rotary encoder
  pinMode(T1_ENCA, INPUT_PULLUP);
  pinMode(T1_ENCB, INPUT_PULLUP);

  // Set pinmode for T1 rotary encoder
  pinMode(T2_ENCA, INPUT_PULLUP);
  pinMode(T2_ENCB, INPUT_PULLUP);

  // Set the rotary encoder interrupt
  attachInterrupt(T1_ENCA, isr_T1, CHANGE);
  attachInterrupt(T2_ENCA, isr_T2, CHANGE);

  // Set pinmode fo T1 DC motor
  pinMode(T1_IN1, OUTPUT);
  pinMode(T1_IN2, OUTPUT);
  

  pinMode(T2_IN1, OUTPUT);
  pinMode(T2_IN2, OUTPUT);

  ledcSetup(led_channel_1, freq, resolution);
  ledcAttachPin(T1_EN, led_channel_1);


  ledcSetup(led_channel_2, freq, resolution);
  ledcAttachPin(T2_EN, led_channel_2);

  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimerMain, true);
  timerAlarmWrite(timer, dt * 1000000, true);
  timerAlarmEnable(timer);
}

void loop() {
  // put your main code here, to run repeatedly:

  double deg_t1, deg_t2;
  deg_t1 = T1_pulse/PPR*360;
  deg_t2 = T2_pulse/PPR*360;
  digitalWrite(T1_EN, HIGH);
  digitalWrite(T2_EN, HIGH);
  digitalWrite(T1_IN1, HIGH);
  digitalWrite(T1_IN2, LOW);
  digitalWrite(T2_IN1, HIGH);
  digitalWrite(T2_IN2, LOW);

  
  Serial.println("T1 Pulse: " + String(T1_pulse));
  Serial.println("T2 Pulse: " + String(T2_pulse));
  Serial.println("T1_deg: " + String(deg_t1));
  Serial.println("T2_deg: " + String(deg_t2));
  delay(50);
}


void isr_T1(){
  if(digitalRead(T1_ENCA) && digitalRead(T1_ENCB)) T1_pulse--;
  else if(!digitalRead(T1_ENCA) && !digitalRead(T1_ENCB)) T1_pulse--;
  else if(digitalRead(T1_ENCA) && !digitalRead(T1_ENCB)) T1_pulse++;
  else if(!digitalRead(T1_ENCA) && digitalRead(T1_ENCB)) T1_pulse++;
}

void isr_T2(){
  if(digitalRead(T2_ENCA) && digitalRead(T2_ENCB)) T2_pulse--;
  else if(!digitalRead(T2_ENCA) && !digitalRead(T2_ENCB)) T2_pulse--;
  else if(digitalRead(T2_ENCA) && !digitalRead(T2_ENCB)) T2_pulse++;
  else if(!digitalRead(T2_ENCA) && digitalRead(T2_ENCB)) T2_pulse++;
}

void IRAM_ATTR onTimerMain(){
  portENTER_CRITICAL_ISR(&timerMux);

  // hitung posisi
  q_res_old = q_res;
  q_res = ((double)pulse_count/PPR)*360.0;

  //path trajectory
  if(count < f*3){
    count++;
    q_cmd = (q_final-q_init)*(double)((double)count/(double)(f*3.0)) + q_init;
  }

  err_old = err;
  err = q_cmd - q_res;
  integral_err += err*dt;
  duty_cycle = kp*err + (kd/dt)*(err - err_old) + ki*integral_err;

  //Output
  if(abs(duty_cycle) > 255){
    duty_cycle = (duty_cycle/abs(duty_cycle))*255;
  }

  if(duty_cycle > 0){
    digitalWrite(T1_IN1, HIGH);
    digitalWrite(T1_IN2, LOW);
    ledcWrite(led_channel_1, abs(duty_cycle));
  }else if(duty_cycle < 0){
    digitalWrite(T1_IN1, LOW);
    digitalWrite(T1_IN2, HIGH);
    ledcWrite(led_channel_1, abs(duty_cycle));
  }else{
    digitalWrite(T1_IN1, HIGH);
    digitalWrite(T1_IN2, HIGH);
    ledcWrite(led_channel_1, 0);
  }
}
