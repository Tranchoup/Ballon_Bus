//Rad Timer
unsigned long duration;  //the variable to store the HIGH length of the pulse
unsigned long rad;
unsigned long Sv;
int pin = A0;
int i = 0;
float tension = 0.0;
int tout_ou_rien = 0;

void setup()
{
  pinMode(pin, INPUT); 
  Serial.begin(9600);  // start serial port at 9600 bps:
}

void loop()
{
  while(i <= 10000){
    tout_ou_rien = analogRead(pin);
    tension = tout_ou_rien * (5.0/1023.0);
    if (tension <= 3.3){
      rad += 1;
    }
    i +=1;
  }
  i=0;
  Serial.println(rad);
  rad = 0;
}