#define T1_IN1 12
#define T1_IN2 14
#define T1_EN 13

#define T2_IN1 26
#define T2_IN2 25
#define T2_EN 27

#define T1_ENCA 34
#define T1_ENCB 15

#define T2_ENCA 39
#define T2_ENCB 4

int T1_pulse = 0;
int T2_pulse = 0;

void isr_T1();
void isr_T2();

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
  pinMode(T1_EN, OUTPUT);

  pinMode(T2_IN1, OUTPUT);
  pinMode(T2_IN2, OUTPUT);
  pinMode(T2_EN, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(T1_EN, HIGH);
  digitalWrite(T2_EN, HIGH);
  digitalWrite(T1_IN1, HIGH);
  digitalWrite(T1_IN2, LOW);
  digitalWrite(T2_IN1, HIGH);
  digitalWrite(T2_IN2, LOW);
  Serial.println("T1 Pulse: " + String(T1_pulse));
  Serial.println("T2 Pulse: " + String(T2_pulse));
  Serial.println("T1_deg: " + String(T1_pulse/620*360));
  Serial.println("T2_deg: " + String(T2_pulse/620*360));
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
