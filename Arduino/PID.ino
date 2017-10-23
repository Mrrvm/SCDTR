#define A -0.61092
#define B 4.69897
#define VDD 5
#define R1 10000
#define MAXLUX 75
#define HIGH 50
#define LOW 25
#define GANHO 3.8
const int analogInPin = A0;
const int analogOutPin = 9;


double lux_converter(double volt){
  double l=pow(((VDD/volt)-1)*R1/pow(10,B),1/A);
  return l;
}

double voltage_converter(double lux){
  double v=(VDD)/(1+pow(10,B)*pow(lux,A)/R1);
  return v;
}

double map_double(double vi, int vi_min, int vi_max, int vo_min, int vo_max){
  return (double)(vi-vi_min)*(vo_max-vo_min)/(double)(vi_max-vi_min)+vo_min;
}

class PID
{
  public:
      PID();
      ~PID();
      bool Control(double);

      void SetReference(double);
      void SetParameters(double, double, double, double);
      void SetSamplingTime(unsigned long);
      void Reset();
      
  
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
      bool reference_change;
      
      double Previous_input;
      double I;
      double D;
      double Previous_error;
      double Antiwindup;
      
      unsigned long Previous_Time;
      unsigned long Sampling_Time;
  
  
};


PID::PID(){
  
  Previous_input=0;
  I=0;
  D=0;
  Previous_error=0;
  Antiwindup=0;
  Previous_Time=millis()-Sampling_Time;
  
  reference_change=false;

}

PID::~PID(){
  
}


bool PID::Control(double input){ 
  unsigned long Present_Time = millis();
  unsigned long deltaT = (Present_Time - Previous_Time);
  double error=0;
  double u=0;
  if(deltaT>=Sampling_Time)
   {
      if(!reference_change){
        error=reference-input;
        double P=Kp*error;

        I+=K2*(error+Previous_error);
        
        //ANTI-WINDUP
        I-=Antiwindup;
    
        if(I>100) {
          Antiwindup=I-100;
          I=100;
        }
        else if (I<-100){
          Antiwindup=I+100;
          I=-100;
        }
        else
          Antiwindup=0;
    
        D*=K3;
        D-=K4*(input-Previous_input);
        u=P+I+D;
      }
      
    else
      reference_change=false;

    int u_int=GANHO*u;
    int full_response=u_int+feedforward;
    if(full_response>255) full_response=255;
    else if(full_response<0) full_response=0;
    analogWrite(analogOutPin, full_response);
    //Serial.print("Controlo:   ");
    //Serial.println(u_int);
    Previous_error=error;
    Previous_input=input;
    Previous_Time=Present_Time;
    return true; 
   }
  else return false;
}

void PID::SetReference(double reference_value){
  reference=reference_value;
  feedforward=GANHO*reference;
  reference_change=true;
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
  Antiwindup=0;
  Previous_Time=millis()-Sampling_Time;
  
  reference_change=false;
}



//const int analogInPin = A0;
//const int analogOutPin = 9;
//const double a=-0.61092;
//const double b=4.69897;
int sensorValue=0;
double sensorVoltage=0;
double sensorLux=0;
int stop=0;


double Input=0;
double Setpoint=LOW;
unsigned long sample=25;
double Kp=0.05, Ki=0.1, Kd=0.1, pole_value=10;

PID myPID;


void setup() {
  Serial.begin(9600);
  myPID.Reset();
  myPID.SetReference(Setpoint);
  myPID.SetSamplingTime(sample);
  myPID.SetParameters(Kp,Ki,Kd,pole_value);
}


void loop() {
  unsigned long startTime=millis();
  Input = analogRead(analogInPin);
  sensorVoltage=map_double((double)Input,0,1023,0,5);
  sensorLux=lux_converter(sensorVoltage);
  myPID.Control(sensorLux);
  //Serial.println("Lux");
  Serial.print(sensorLux);
  Serial.print("  ");
  Serial.println(millis());
  unsigned long endTime=millis();
  delay(sample-(endTime-startTime));
  /*if(millis()>10000){
    Setpoint=40;
    myPID.SetReference(Setpoint);
  }
  if(millis()>20000){
    Setpoint=60;
    myPID.SetReference(Setpoint);
  }*/
}
