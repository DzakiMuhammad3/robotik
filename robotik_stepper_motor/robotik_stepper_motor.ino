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
#define STEPS_REV 200

#define SERVO_PWM 2

int pulse_count = 0;
int steps_req;
int delay_req;
int start_millis;
float final_angle = theta_final;
float desired_time = 3 * 1000000;

void isr();

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(PWM_STEPPER, OUTPUT);
  pinMode(DIRECT_STEPPER, OUTPUT);

  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);

  attachInterrupt(ENCA, isr, CHANGE);
  digitalWrite(DIRECT_STEPPER, HIGH);

  steps_req = (final_angle/360)*STEPS_REV;
  delay_req = desired_time / steps_req;
  start_millis = millis();

  for(int i=0; i<steps_req; i++){
    digitalWrite(PWM_STEPPER, HIGH);
    delayMicroseconds(2000);
    digitalWrite(PWM_STEPPER, LOW);
    delayMicroseconds(delay_req - 2000);
  }

}

void loop() {
  // put your main code here, to run repeatedly:

  Serial.println("start time: " + String(start_millis));
  Serial.println("final time: " + String(millis()));
  Serial.println("delta time " + String(millis() - start_millis));

}

void isr(){
  if(digitalRead(ENCA) && digitalRead(ENCB)) pulse_count--;
  else if(!digitalRead(ENCA) && !digitalRead(ENCB)) pulse_count--;
  else if(digitalRead(ENCA) && !digitalRead(ENCB)) pulse_count++;
  else if(!digitalRead(ENCA) && digitalRead(ENCB)) pulse_count++;
}
