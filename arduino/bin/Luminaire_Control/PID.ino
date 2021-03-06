// Defining constants and pins
#define A -0.61092
#define B 4.69897
#define VDD 5
#define R1 10000
#define MAXLUX 75
#define HIGH 50
#define LOW 25
#define W 75
//PWM=M*LUX+OFFSET
#define M 2.582
#define OFFSET 28
#define ERROR_MAX 1
const int analogInPin = A0;
const int analogOutPin = 9;


// Convert voltage to lux
double lux_converter(double volt){
  double l=pow(((VDD/volt)-1)*R1/pow(10,B),1/A);
  return l;
}

//Convert lux to voltage
double voltage_converter(double lux){
  double v=(VDD)/(1+pow(10,B)*pow(lux,A)/R1);
  return v;
}

//Maps variables with double type
double map_double(double vi, int vi_min, int vi_max, int vo_min, int vo_max){
  return (double)(vi-vi_min)*(vo_max-vo_min)/(double)(vi_max-vi_min)+vo_min;
}



//PID class

class PID
{
  public:
      PID();  //constructor
      ~PID(); //destructor
      bool Control(double);  //control loop function
  
  
  
      void SetReference(double);                           //set reference of PID
      void SetParameters(double, double, double, double);  //set parameters kp,ki,kd,pole
      void SetSamplingTime(unsigned long);                 //set sampling time
      void Reset();                                        //resets all members to default (constructor) values
      bool ToggleFeedforward();
      double GetReference();
      double GetPWMPercent();
  
  private:
  
      double Kp;
      double Ki;
      double Kd;
      double Pole;

      double K2;
      double K3;
      double K4;
      
      double reference;
      int feedforward;
      int full_response;
      bool reference_change;   //whether the reference has been changed after the last computation

      double Previous_input;        //input at last computation
      double I;                     //integral term of PID
      double D;                     //derivative term of PID
      double Previous_error;        //error at last computation
      
      unsigned long Previous_Time;  //instant of last computation
      unsigned long Sampling_Time;  //sampling time
  
      bool feedforward_on;
  
};

//Constructor
PID::PID(){
  
  //set all members to default parameters
  Previous_input=0;
  I=0;
  D=0;
  Previous_error=0;
  Previous_Time=millis()-Sampling_Time;
  feedforward_on = 1;
  
  reference_change=false;

}

//Destructor
PID::~PID(){
  
}

//Control loop function
bool PID::Control(double input){ 
  
  //check if time interval between last computation
  //and present one is at least one sample interval
  unsigned long Present_Time = millis();
  unsigned long deltaT = (Present_Time - Previous_Time);
  
  //initialize aux variables as zero
  double error=0;
  double u=0;
  
  if(deltaT>=Sampling_Time)
   {
      //checks if has to include feedback term
      if(!reference_change || !feedforward_on){
        error=reference-input;

        /*(error >= ERROR_MAX) {
          error = error - ERROR_MAX;
        }
        else if(error <= -ERROR_MAX) {
          error = error - (-ERROR_MAX);
        }
        else
          error = 0;*/
        
        //Proportional
        double P=Kp*error;
        
        //Integral
        I+=K2*(error+Previous_error);
        
        //ANTI-WINDUP
        if(I>W) {
          I=W;
        }
        else if (I<-W){
          I=-W;
        }
        
        //Derivative
        D*=K3;
        D-=K4*(input-Previous_input);
        
        //Total feedback=PID (in lux)
        u=P+I+D;

      }
      
      //makes variable false so in next computation feedback is included
      else
        reference_change=false;

    int feedback=M*u;  //total feedback (in PWM)
    if(feedforward_on) {
      full_response=feedback+feedforward;
    }
    else { 
      full_response = feedback;
    }
      
    //Check maximum values of PWM
    if(full_response>255) full_response=255;
    else if(full_response<0) full_response=0;
    analogWrite(analogOutPin, full_response);

    //Records variables of present calculation
    Previous_error=error;
    Previous_input=input;
    Previous_Time=Present_Time;
    return true;      
   }
  else return false;  //interval was smaller than sampling time
}

void PID::SetReference(double reference_value){
  reference=reference_value;   //reference (in lux)
  feedforward=M*reference+OFFSET; //reference (in PWM)
  reference_change=true;       //in next cycle, feedback will not be included
}

void PID::SetParameters(double kp_value, double ki_value, double kd_value, double pole_value){
  Kp=kp_value;
  Ki=ki_value;
  Kd=kd_value;
  Pole=pole_value;
  
  K2=Kp*Ki*Sampling_Time/2;
  K3=Kd/(Kd+Pole*Sampling_Time);
  K4=Kp*Kd*Pole/(Kd+Pole*Sampling_Time);
}

void PID::SetSamplingTime(unsigned long Sampling_Time_value){
  Sampling_Time=Sampling_Time_value;
}

void PID::Reset(){
  Previous_input=0;
  I=0;
  D=0;
  Previous_error=0;
  Previous_Time=millis()-Sampling_Time;
  
  reference_change=false;
}


bool PID::ToggleFeedforward(){
  feedforward_on = !feedforward_on;
  return feedforward_on;
}

double PID::GetReference(){
  return reference;
}

double PID::GetPWMPercent(){
  return ((double)full_response/255)*100;
}


int sensorValue=0;
double sensorVoltage=0;
double sensorLux=0;

int setpoint_changes=0;

unsigned long sample=25;
double Kp=0.1, Ki=0.1, Kd=0, pole_value=10;

bool fftoggle = 1;

PID myPID;


void setup() {
  Serial.begin(115200);
  myPID.Reset();
  myPID.SetReference(HIGH);
  myPID.SetSamplingTime(sample);
  myPID.SetParameters(Kp,Ki,Kd,pole_value);
  pinMode(13, OUTPUT);
}


void loop() {
  unsigned long startTime=millis(); 
  //time at start of loop
  digitalWrite(13, HIGH); 
  sensorValue = analogRead(analogInPin);                    //input in 0-1023 range
  sensorVoltage=map_double(sensorValue,0,1023,0,5); //maps input to voltage interval 0-5
  sensorLux=lux_converter(sensorVoltage);             //converts to lux
  myPID.Control(sensorLux);                           //computes with input in lux
  String message = String(myPID.GetReference());
  message += (" "+String(sensorLux));
  message += (" "+String(myPID.GetPWMPercent()));
  message += (" "+String(fftoggle));
  message += (" "+String(millis()));
  Serial.println(message);

  unsigned long endTime=millis();                     //time at end of loop
  delay(sample-(endTime-startTime));                  //delay to avoid over computation of "control"
  
  while (Serial.available()) {
    int read=Serial.parseInt();
    if(read>0 && read<70) {
      Serial.print("Nova Referência: ");
      Serial.println(read);
      myPID.SetReference(read);
    }
    else if(read == -1) {
      fftoggle = myPID.ToggleFeedforward();
      Serial.print("Feedforward status: ");
      Serial.println(fftoggle);
    }
  }
  /*if(millis()>5000 && setpoint_changes==0){
    myPID.SetReference(HIGH);
    setpoint_changes=1;
  }
  if(millis()>10000 && setpoint_changes==1){
    myPID.SetReference(LOW);
    setpoint_changes=2;
  }
  if(millis()>15000 && setpoint_changes==2){
    myPID.SetReference(HIGH);
    setpoint_changes=3;
  }*/
  
}