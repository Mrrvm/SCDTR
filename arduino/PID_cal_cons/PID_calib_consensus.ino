#include <Wire.h>
#include <EEPROM.h>
#include "MatrixMath.h"

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
#define MEASURES 1
#define OFFSET 28
#define ERROR_MAX 1
#define N 2
#define C 3
const int analogInPin = A0;
const int analogOutPin = 9;

int my_address=0;
bool calibrated=0;
int count_sum=0, count=0; 
unsigned int dcycle=0;
double other_sum=0;
int step=1;
float K[N]={0};

float L;
float o;
float d[N][N]={0};
float d_av[N][1]={0};
float c[N][1]={0};
float Q[N][N]={0};
float rho=0.01;
float y[N][1]={0};

int step2=0;


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
        
        //DEADZONE
        if(error >= ERROR_MAX) {
          error = error - ERROR_MAX;
        }
        else if(error <= -ERROR_MAX) {
          error = error - (-ERROR_MAX);
        }
        else
          error = 0;
          

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
        u=0;//P+I+D;

      }
      
      //makes variable false so in next computation feedback is included
      else
        reference_change=false;

    int feedback=u/K[my_address-1];  //total feedback (in PWM)
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
  L=reference;
  K[0]=K[0]*255/100;
  K[1]=K[1]*255/100;
  consensus();
  feedforward=d_av[my_address-1][0]*255/100;//reference/K[my_address-1]; //reference (in PWM)
  Matrix.Print((float*)d_av,N,1,"dav");
  Serial.println(feedforward);
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


void calib(){
  double self_sum=0;
  sensorValue=analogRead(analogInPin); //then reads in it's OWN LDR
	sensorVoltage=map_double((double)sensorValue,0,1023,0,5);
	o=lux_converter(sensorVoltage);
  for(int j=0;j<C;++j){
    for(int i=0;i<=255;++i){
      analogWrite(analogOutPin,i);
      delay(30);
      sensorValue=analogRead(analogInPin); //then reads in it's OWN LDR
	    sensorVoltage=map_double((double)sensorValue,0,1023,0,5);
	    sensorLux=lux_converter(sensorVoltage);
	    if(i>50) //ignores first values because of non linearity
	      self_sum+=sensorLux/i;
	    Wire.beginTransmission(0); //master arduino sends to slave the duty cycle he wants on OTHER'S LED
	    Wire.write("k");
      Wire.write(i);
      Wire.endTransmission();
      //Serial.println(i+256*j);
    }
  }
  analogWrite(analogOutPin,0);
  K[step-1]=self_sum/205/C;
  /*Serial.print(my_address);
  Serial.print(" ");
  Serial.print(step);
  Serial.print(" ");
  Serial.println(K[step-1][my_address-1]);*/
  step++;
}


void consensus(){
  unsigned long now=millis();
  for(int it=0;it<30;++it)
    {
        
      float min_best=10000;
      float z[N][1]={0};
      float P[N][N]={0};
      float R[N][N]={0};
      float d_aux[N][1]={0};
      float aux1[N][1]={0};
      float aux2[N][1]={0};
      float aux3[N][2]={0};
      float aux4[N][2]={0};
      float aux5[2][1]={0};
      float cost_function={0};
      bool sol[5];
      for(int l=0;l<5;++l) sol[l]=1;

      
      
      Matrix.Copy((float*)d_av,N,1,(float*)z);
      
      Matrix.Scale((float*)z,N,1,rho);
      
      Matrix.Subtract((float*)z, (float*)y, N, 1, (float*)z);
      
      Matrix.Subtract((float*)z, (float*)c, N, 1, (float*)z);
      
      for(int l=0;l<N;++l)
        R[l][l]=Q[l][l]+rho;
      Matrix.InvertDiagonal((float*)R, N, (float*)P);
      Matrix.Multiply((float*)P, (float*)z, N, N, 1, (float*)d_aux);
      if(d_aux[my_address-1][0]<0) 
        sol[0] =0;
      if(d_aux[my_address-1][0]>100) 
        sol[0] = 0;
      
      //Matrix.Multiply((float*)K,(float*)d_aux,N,N,1,(float*)aux);
      if(Matrix.Dot((float*)K,N,(float*)d_aux)<L-o) 
        sol[0]=0;

      if(sol[0]){
        cost_function=0;
        Matrix.Multiply((float*)Q, (float*)d_aux, N, N, 1, (float*)aux1);
        Matrix.Subtract((float*)d_aux,(float*)d_av,N,1,(float*)aux2);
        cost_function+=0.5*Matrix.Dot((float*)d_aux,N,(float*)aux1);
        cost_function+=Matrix.Dot((float*)c,N,(float*)d_aux);
        cost_function+=Matrix.Dot((float*)y,N,(float*)aux2);
        cost_function+=0.5*rho*Matrix.Dot((float*)aux2,N,(float*)aux2);
        if(cost_function<min_best){
          for(int l=0;l<N;++l)
            d[l][my_address-1]=d_aux[l][0];
          min_best=cost_function;
        }
      }
      
      
      float A_1[1][N]={0};
      float A_2_3[1][N]={0};
      float A_4_5[2][N]={0};
      float u_1_2_3={0};
      float u_4_5[2][1]={0};
      float w_1_2_3={0};
      float w_4_5[2][1]={0};
      float x_1_2_3={0};
      float x_4_5[2][2]={0};
      for(int l=0;l<N;++l){
        A_1[0][l]=-K[l];
        A_4_5[0][l]=-K[l];
        if(l==my_address-1){
          A_2_3[0][l]=-1;
          A_4_5[1][l]=-1;
        }
      }
      
      // A1
      u_1_2_3=(o-L);
      Matrix.Multiply((float*)P, (float*)z, N, N, 1, (float*)d_aux);
      Matrix.Transpose((float*)A_1,1,N,(float*)aux1);
      
      Matrix.Multiply((float*)P, (float*)aux1, N, N, 1, (float*)aux2);
      
      x_1_2_3=Matrix.Dot((float*)A_1,N,(float*)aux2);
      
      x_1_2_3=1/x_1_2_3;
      
      w_1_2_3=Matrix.Dot((float*)A_1,N,(float*)d_aux);   
      u_1_2_3=(u_1_2_3-w_1_2_3)*x_1_2_3;
      Matrix.Scale((float*)aux2,2,1,u_1_2_3);
      Matrix.Add((float*)d_aux,(float*)aux2,N,1,(float*)d_aux);
      
      if(d_aux[my_address-1][0]<0) 
        sol[1] =0;
      if(d_aux[my_address-1][0]>100) 
        sol[1] =0;
      
      
      if(sol[1]){
        cost_function=0;
        Matrix.Multiply((float*)Q, (float*)d_aux, N, N, 1, (float*)aux1);
        Matrix.Subtract((float*)d_aux,(float*)d_av,N,1,(float*)aux2);
        cost_function+=0.5*Matrix.Dot((float*)d_aux,N,(float*)aux1);
        cost_function+=Matrix.Dot((float*)c,N,(float*)d_aux);
        cost_function+=Matrix.Dot((float*)y,N,(float*)aux2);
        cost_function+=0.5*rho*Matrix.Dot((float*)aux2,N,(float*)aux2);
        if(cost_function<min_best){
          for(int l=0;l<N;++l)
            d[l][my_address-1]=d_aux[l][0];
          min_best=cost_function;
        }
      }
      
      // A2
      u_1_2_3=0;
      Matrix.Multiply((float*)P, (float*)z, N, N, 1, (float*)d_aux);
      Matrix.Transpose((float*)A_2_3,1,N,(float*)aux1);
      
      Matrix.Multiply((float*)P, (float*)aux1, N, N, 1, (float*)aux2);
      
      x_1_2_3=Matrix.Dot((float*)A_2_3,N,(float*)aux2);
      
      x_1_2_3=1/x_1_2_3;
      
      w_1_2_3=Matrix.Dot((float*)A_2_3,2,(float*)d_aux);   
      u_1_2_3=(u_1_2_3-w_1_2_3)*x_1_2_3;
      Matrix.Scale((float*)aux2,N,1,u_1_2_3);
      Matrix.Add((float*)d_aux,(float*)aux2,N,1,(float*)d_aux);
      //Matrix.Multiply((float*)K,(float*)d_aux,N,N,1,(float*)aux);
      if(Matrix.Dot((float*)K,N,(float*)d_aux)<L-o) 
        sol[2] =0;
      if(d_aux[my_address-1][0]>100) 
        sol[2] = 0;
      if(sol[2]){
        cost_function=0;
        Matrix.Multiply((float*)Q, (float*)d_aux, N, N, 1, (float*)aux1);
        Matrix.Subtract((float*)d_aux,(float*)d_av,N,1,(float*)aux2);
        cost_function+=0.5*Matrix.Dot((float*)d_aux,N,(float*)aux1);
        cost_function+=Matrix.Dot((float*)c,N,(float*)d_aux);
        cost_function+=Matrix.Dot((float*)y,N,(float*)aux2);
        cost_function+=0.5*rho*Matrix.Dot((float*)aux2,N,(float*)aux2);
        if(cost_function<min_best){
          for(int l=0;l<N;++l)
            d[l][my_address-1]=d_aux[l][0];
          min_best=cost_function;
        }
      }
      
      // A3
      u_1_2_3=100;
      A_2_3[0][my_address-1]=1;
      Matrix.Multiply((float*)P, (float*)z, N, N, 1, (float*)d_aux);
      Matrix.Transpose((float*)A_2_3,1,N,(float*)aux1);
      
      Matrix.Multiply((float*)P, (float*)aux1, N, N, 1, (float*)aux2);
      
      x_1_2_3=Matrix.Dot((float*)A_2_3,N,(float*)aux2);
      
      x_1_2_3=1/x_1_2_3;
      
      w_1_2_3=Matrix.Dot((float*)A_2_3,N,(float*)d_aux);   
      u_1_2_3=(u_1_2_3-w_1_2_3)*x_1_2_3;
      Matrix.Scale((float*)aux2,N,1,u_1_2_3);
      Matrix.Add((float*)d_aux,(float*)aux2,N,1,(float*)d_aux);
      //Matrix.Multiply((float*)K,(float*)d_aux,N,N,1,(float*)aux);
      if(Matrix.Dot((float*)K,N,(float*)d_aux)<L-o) 
        sol[3] =0;
      if(d_aux[my_address-1][0]<0) 
        sol[3] = 0;
     
      if(sol[3]){
        cost_function=0;
        Matrix.Multiply((float*)Q, (float*)d_aux, N, N, 1, (float*)aux1);
        Matrix.Subtract((float*)d_aux,(float*)d_av,N,1,(float*)aux2);
        cost_function+=0.5*Matrix.Dot((float*)d_aux,N,(float*)aux1);
        cost_function+=Matrix.Dot((float*)c,N,(float*)d_aux);
        cost_function+=Matrix.Dot((float*)y,N,(float*)aux2);
        cost_function+=0.5*rho*Matrix.Dot((float*)aux2,N,(float*)aux2);
        if(cost_function<min_best){
          for(int l=0;l<N;++l)
            d[l][my_address-1]=d_aux[l][0];
          min_best=cost_function;
        }
      }
      
      
      // A4
      u_4_5[0][0]=(o-L);
      u_4_5[1][0]=0;
      Matrix.Multiply((float*)P, (float*)z, N, N, 1, (float*)d_aux);
      Matrix.Transpose((float*)A_4_5,2,N,(float*)aux3);
      
      Matrix.Multiply((float*)P, (float*)aux3, N, N, 2, (float*)aux4);
      Matrix.Multiply((float*)A_4_5, (float*)aux4, 2, N, 2, (float*)x_4_5);
      Matrix.Invert((float*)x_4_5,2);
      
      Matrix.Multiply((float*)A_4_5,(float*)d_aux, 2, N, 1, (float*)w_4_5);
      Matrix.Subtract((float*)u_4_5,(float*)w_4_5,2,1,(float*)u_4_5);
      Matrix.Multiply((float*)x_4_5,(float*)u_4_5,2,2,1,(float*)aux5);
      Matrix.Multiply((float*)aux4,(float*)aux5,N,2,1,(float*)aux2);
      Matrix.Add((float*)d_aux,(float*)aux2,N,1,(float*)d_aux);
      
      if(d_aux[my_address-1][0]>100) 
        sol[4] =0;
      
      if(sol[4]){
        cost_function=0;
        Matrix.Multiply((float*)Q, (float*)d_aux, N, N, 1, (float*)aux1);
        Matrix.Subtract((float*)d_aux,(float*)d_av,N,1,(float*)aux2);
        cost_function+=0.5*Matrix.Dot((float*)d_aux,N,(float*)aux1);
        cost_function+=Matrix.Dot((float*)c,N,(float*)d_aux);
        cost_function+=Matrix.Dot((float*)y,N,(float*)aux2);
        cost_function+=0.5*rho*Matrix.Dot((float*)aux2,N,(float*)aux2);
        
        if(cost_function<min_best){
          for(int l=0;l<N;++l)
            d[l][my_address-1]=d_aux[l][0];
          min_best=cost_function;
        }
      }
      
      
      // A5
      u_4_5[0][0]=(o-L);
      u_4_5[1][0]=100;
      A_4_5[1][my_address-1]=1;
      Matrix.Multiply((float*)P, (float*)z, N, N, 1, (float*)d_aux);
      Matrix.Transpose((float*)A_4_5,2,N,(float*)aux3);
      
      Matrix.Multiply((float*)P, (float*)aux3, N, N, 2, (float*)aux4);
      Matrix.Multiply((float*)A_4_5, (float*)aux4, 2, N, 2, (float*)x_4_5);
      Matrix.Invert((float*)x_4_5,2);
      
      Matrix.Multiply((float*)A_4_5,(float*)d_aux, 2, N, 1, (float*)w_4_5);
      Matrix.Subtract((float*)u_4_5,(float*)w_4_5,2,1,(float*)u_4_5);
      Matrix.Multiply((float*)x_4_5,(float*)u_4_5,2,2,1,(float*)aux5);
      Matrix.Multiply((float*)aux4,(float*)aux5,N,2,1,(float*)aux2);
      Matrix.Add((float*)d_aux,(float*)aux2,N,1,(float*)d_aux);
      
      if(d_aux[my_address-1][0]<0) 
        sol[5] =0;
      
      if(sol[5]){
        cost_function=0;
        Matrix.Multiply((float*)Q, (float*)d_aux, N, N, 1, (float*)aux1);
        Matrix.Subtract((float*)d_aux,(float*)d_av,N,1,(float*)aux2);
        cost_function+=0.5*Matrix.Dot((float*)d_aux,N,(float*)aux1);
        cost_function+=Matrix.Dot((float*)c,N,(float*)d_aux);
        cost_function+=Matrix.Dot((float*)y,N,(float*)aux2);
        cost_function+=0.5*rho*Matrix.Dot((float*)aux2,N,(float*)aux2);
        
        if(cost_function<min_best){
          for(int l=0;l<N;++l)
            d[l][my_address-1]=d_aux[l][0];
          min_best=cost_function;
        }
      }

      
      
      byte a[N];
      for(int l=0;l<N;++l)
        a[l]=(byte)d[l][my_address-1];
      step2=1;
      
      //if(my_address==1){
      //  Wire.beginTransmission(0);
      //  Wire.write(1);
      //  Wire.endTransmission();

      while(step2<=N){
        delay(1);
        if(step2==my_address){
          for(int l=0;l<N;++l){
            Wire.beginTransmission(0);
            Wire.write("c");
            Wire.write(l);
            Wire.write(a[l]);
            Wire.endTransmission();
          }
          step2++;
        }
        
      }

      for(int l=0;l<N;++l){
        d_av[l][0]=0;
        d_aux[l][0]=d[l][my_address-1];
        for(int l2=0;l2<N;++l2)
          d_av[l][0]+=d[l][l2];
        d_av[l][0]=d_av[l][0]/N;
      }
      
      
      //Matrix.Print((float*)d_av,N,1,"dav");
      //Matrix.Print((float*)d,N,N,"dbest");
      //Serial.println(min_best);
      
      Matrix.Subtract((float*)d_aux,(float*)d_av,N,1,(float*)d_aux);
      Matrix.Scale((float*)d_aux,N,1,rho);
      Matrix.Add((float*)y,(float*)d_aux,N,1,(float*)y);

    }
    Serial.println(millis()-now);
}

void receiveEvent(int howMany)	{
  while(Wire.available() >	0)	{
    char phase=Wire.read();
    if(phase="k"){
      if(step!=my_address && step<=N){
        dcycle=Wire.read();
        sensorValue=analogRead(analogInPin); //then reads in it's OWN LDR
	      sensorVoltage=map_double((double)sensorValue,0,1023,0,5);
	      sensorLux=lux_converter(sensorVoltage);
	      if(dcycle>50){ 
	        other_sum+=sensorLux/dcycle;
	        count_sum++;
	      }
	      count++;
	      if(dcycle==256-1 && count>(C-1)*256+100){
	        K[step-1]=other_sum/count_sum;
	        count=0;
	        count_sum=0;
	        other_sum=0;
	        step++;
	      }
      }
    }
    if(phase="c"){
      int h=Wire.read();
      int b=Wire.read();
      d[h][step2-1]=b;
      if(h==N-1) step2++;
    }
  }
}



void setup() {
  Serial.begin(115200);
  myPID.Reset();
  myPID.SetSamplingTime(sample);
  myPID.SetParameters(Kp,Ki,Kd,pole_value);
  pinMode(13, OUTPUT);
  my_address=EEPROM.read(0);
  Wire.begin(my_address);
  TWAR = (my_address << 1) | 1;
  Wire.onReceive(receiveEvent);
  c[my_address-1][0]=1;
  Q[my_address-1][my_address-1]=0;
  for(int l=0;l<N;++l)
    d_av[l][0]=20;
}


void loop() {
  
  if(!calibrated){
    while(step<=N){
      delay(5);
      if(step==my_address){
        calib();
      }
    }
    Serial.println(K[0]);
    Serial.println(K[1]);
    calibrated=1;
    myPID.SetReference(HIGH);
  }
  
  
  unsigned long startTime=millis(); 
  //time at start of loop
  digitalWrite(13, HIGH);
  sensorValue=0;
  for(int j=0;j<MEASURES;++j)
    sensorValue += analogRead(analogInPin);                    //input in 0-1023 range
  sensorValue=sensorValue/MEASURES;  
  sensorVoltage=map_double(sensorValue,0,1023,0,5); //maps input to voltage interval 0-5
  sensorLux=lux_converter(sensorVoltage);             //converts to lux
  myPID.Control(sensorLux);                           //computes with input in lux
  String message = String(myPID.GetReference());
  message += (" "+String(sensorLux));
  message += (" "+String(myPID.GetPWMPercent()));
  message += (" "+String(fftoggle));
  message += (" "+String(millis()));
  Serial.println(message);
  
  //Serial.print(sensorValue);
  //Serial.print(" ");
  //Serial.println(sensorLux);

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
