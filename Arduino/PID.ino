#define A -0.61092
#define B 4.69897
#define VDD 5
#define R1 10000
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
      PID(double, double, double, double, double, double, double, double, unsigned long);
      bool Control(double);
  
  
  
      void SetReference(double);
      void SetParameters(double, double, double, double, double);
      void SetSamplingTime(unsigned long);
      
      double GetK1();
  
  private:
  
      double Kp;
      double Ki;
      double Kd;
      double a;
      double b;
      double K1;
      double K2;
      double K3;
      double K4;
      
      double PID_input;
      double PID_output;
      double PID_reference;
      
      double Previous_input;
      double Previous_i;
      double Previous_d;
      double Previous_e;
      double antiwindup;
      
      unsigned long Previous_Time;
      unsigned long Sampling_Time;
  
  
};


PID::PID(double input, double output, double reference_value, double kp_value, 
          double ki_value, double kd_value, double a_value, double b_value, unsigned long Sampling_Time_value){
  
  PID_input=input;
  PID_output=output;
  
  PID::SetReference(reference_value);
  
  PID::SetParameters(kp_value, ki_value, kd_value, a_value, b_value);
  
  PID::SetSamplingTime(Sampling_Time_value);
  
  Previous_input=0;
  Previous_i=0;
  Previous_d=0;
  Previous_e=0;
  antiwindup=0;
  Previous_Time=millis()-Sampling_Time;

}


bool PID::Control(double input){ 
  unsigned long Present_Time = millis();
  unsigned long deltaT = (Present_Time - Previous_Time);
  if(deltaT>=Sampling_Time)
   {
    double error=PID_reference-input;
    double p=K1*PID_reference-Kp*input;
    //Serial.println(p);
    Previous_i+=K2*(error+Previous_e);
    Previous_i-=antiwindup;
    //ANTI-WINDUP
    
    if(Previous_i>100) {
      antiwindup=Previous_i-100;
      Previous_i=100;
    }
    else if (Previous_i<-100){
      antiwindup=Previous_i+100;
      Previous_i=-100;
    }
    else
      antiwindup=0;
    
    
    //Serial.println(Previous_i);
    Previous_d*=K3;
    Previous_d-=K4*(input-Previous_input);
    double u=p+Previous_i+Previous_d;
    // ADD LIMITS TO U
    u+=PID_reference*0.1;
    double ffd=voltage_converter(u);
    int ffd2=(int)map_double(ffd,0,5,0,255);
    analogWrite(analogOutPin, ffd2);
    Previous_e=error;
    Previous_input=input;
    Previous_Time=Present_Time;
    return true; 
   }
  else return false;
}

void PID::SetReference(double reference_value){
  PID_reference=reference_value;
}

void PID::SetParameters(double kp_value, double ki_value, double kd_value, double a_value, double b_value){
  Kp=kp_value;
  Ki=ki_value;
  Kd=kd_value;
  a=a_value;
  b=b_value;
  
  K1=Kp*b;
  K2=Kp*Ki*Sampling_Time/2;
  K3=Kd/(Kd+a*Sampling_Time);
  K4=Kp*Kd*a/(Kd+a*Sampling_Time);
}

void PID::SetSamplingTime(unsigned long Sampling_Time_value){
  Sampling_Time=Sampling_Time_value;
}

double PID::GetK1(){
  return K1;
}


//const int analogInPin = A0;
//const int analogOutPin = 9;
//const double a=-0.61092;
//const double b=4.69897;
int sensorValue=0;
double sensorVoltage=0;
double sensorLux=0;
int stop=0;


double Input;
double Output;
double Setpoint=60;
unsigned long sample=50;
double Kp=0.8, Ki=0.8, Kd=0.1, a_value=10, b_value=1;

PID myPID(Input, Output, Setpoint, Kp, Ki, Kd, a_value, b_value, sample);




void setup() {
  Serial.begin(9600);
  myPID.SetParameters(Kp,Ki,Kd,a_value,b_value);
}


void loop() {
  
  unsigned long startTime=millis();
  Input = analogRead(analogInPin);
  sensorVoltage=map_double((double)Input,0,1023,0,5);
  sensorLux=lux_converter(sensorVoltage);
  myPID.Control(sensorLux);
  //Serial.println("Lux");
  Serial.println(sensorLux);
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
