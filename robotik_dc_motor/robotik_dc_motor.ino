// Muhammad Dzaki Mubarak
int last_digit_npm = 75;
float theta_final = 75*10 + 50;

#define INDC1 26
#define INDC2 27
#define ENDC 14
#define ENCA 33
#define ENCB 32

#define PWM_STEPPER 25
#define DIRECT_STEPPER 15

#define SERVO_PWM 2


const int freq = 500;
const int led_channel = 0;
const int resolution = 8;
const int PPR = 480;

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



void isr();
void IRAM_ATTR onTimerMain();

// Make timer
hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);\

  // Set Pin Mode for DC PWM
  ledcSetup(led_channel, freq, resolution);
  ledcAttachPin(ENDC, led_channel);


  // Set Pin mode for DC Motor
  pinMode(INDC1, OUTPUT);
  pinMode(INDC2, OUTPUT);

  // Set pinmode for rotary encoder
  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);

  // Set the interrupt
  attachInterrupt(ENCA, isr, CHANGE);

  // Make timer interrupt
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimerMain, true);
  timerAlarmWrite(timer, dt * 1000000, true);
  timerAlarmEnable(timer);

  q_final = theta_final;

  
  

}

void loop() {
  // put your main code here, to run repeatedly:

  Serial.println("time(s): " + String(millis()/1000));
  Serial.println("q_cmd: " + String(q_cmd));
  Serial.println("q_res: " + String(q_res));

  delay(200);

}

void isr(){
  if(digitalRead(ENCA) && digitalRead(ENCB)) pulse_count--;
  else if(!digitalRead(ENCA) && !digitalRead(ENCB)) pulse_count--;
  else if(digitalRead(ENCA) && !digitalRead(ENCB)) pulse_count++;
  else if(!digitalRead(ENCA) && digitalRead(ENCB)) pulse_count++;
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
    digitalWrite(INDC1, HIGH);
    digitalWrite(INDC2, LOW);
    ledcWrite(led_channel, abs(duty_cycle));
  }else if(duty_cycle < 0){
    digitalWrite(INDC1, LOW);
    digitalWrite(INDC2, HIGH);
    ledcWrite(led_channel, abs(duty_cycle));
  }else{
    digitalWrite(INDC1, HIGH);
    digitalWrite(INDC1, HIGH);
    ledcWrite(led_channel, 0);
  }
}
