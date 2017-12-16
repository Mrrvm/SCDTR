#include <Wire.h>
#include <EEPROM.h>

#include "MatrixMath.h"
#include "PID.h"
#include "Defs.h"


int my_address=0;
bool calibrated=0;
int count_sum=0, count=0; 
double other_sum=0;
int step=1;
float K[N]={0};
float o=0;

float d[N][N]={0};
float d_av[N][1]={0};
float c[N][1]={0};
float Q[N][N]={0};
float rho=0.01;
float y[N][1]={0};

int step2=1;


int sensorValue=0;
double sensorVoltage=0;
double sensorLux=0;

int setpoint_changes=0;

unsigned long sample=25;
double Kp=0.1, Ki=0.1, Kd=0, pole_value=10;

bool fftoggle = 1;

PID myPID;

bool call_consensus=0;
bool change_occupancy[2]={0};

bool loop_calib(){
  analogWrite(analogOutPin,0);
  delay(1000);
  o=0;
  for(int j=0;j<C;++j){
    sensorValue=analogRead(analogInPin); //then reads in it's OWN LDR
	  sensorVoltage=map_double((double)sensorValue,0,1023,0,5);
	  o+=lux_converter(sensorVoltage);
  }
  o=o/C;
  while(step<=N){
      delay(5);
      if(step==my_address){
        calib();
      }
    }
  step=1;
  count_sum=0;
  count=0;
  other_sum=0;
  return true;
}


void calib(){
  double self_sum=0;
  for(int j=0;j<C;++j){
    for(int i=0;i<=255;++i){
      analogWrite(analogOutPin,i);
      delay(30);
      sensorValue=analogRead(analogInPin); //then reads in it's OWN LDR
	    sensorVoltage=map_double((double)sensorValue,0,1023,0,5);
	    sensorLux=lux_converter(sensorVoltage);
	    if(i>50) //ignores first values because of non linearity
	      self_sum+=(sensorLux-o)/i;
	   //Serial.println(sensorLux);
	    Wire.beginTransmission(0); //master arduino sends to slave the duty cycle he wants on OTHER'S LED
	    Wire.write("k");
      Wire.write(i);
      Wire.endTransmission();
    }
  }
  analogWrite(analogOutPin,0);
  K[step-1]=self_sum/205/C;
  step++;
}


float consensus(float L){
  //Serial.println("entering consensus...");
  //unsigned long now=millis();
  y[0][0]=0;
  y[1][0]=0;
  for(int l=0;l<N;++l)
    d_av[l][0]=0;
  //if(my_address==1) delay(1000);
  for(int it=0;it<10;++it)
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
      //Matrix.Print((float*)P,N,N,"P");
      Matrix.Multiply((float*)P, (float*)z, N, N, 1, (float*)d_aux);
      if(d_aux[my_address-1][0]<0) 
        sol[0] =0;
      if(d_aux[my_address-1][0]>255) 
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
      //Serial.print("u");
      //Serial.println(u_1_2_3);
      Matrix.Multiply((float*)P, (float*)z, N, N, 1, (float*)d_aux);
      //Matrix.Print((float*)d_aux,N,1,"Pz");
      Matrix.Transpose((float*)A_1,1,N,(float*)aux1);
      
      Matrix.Multiply((float*)P, (float*)aux1, N, N, 1, (float*)aux2);
      //Matrix.Print((float*)aux2,N,1,"PA^T");
      
      x_1_2_3=Matrix.Dot((float*)A_1,N,(float*)aux2);
      
      x_1_2_3=1/x_1_2_3;
      //Serial.print("1/x");
      //Serial.println(x_1_2_3);
      
      w_1_2_3=Matrix.Dot((float*)A_1,N,(float*)d_aux);   
      u_1_2_3=(u_1_2_3-w_1_2_3)*x_1_2_3;
      Matrix.Scale((float*)aux2,2,1,u_1_2_3);
      Matrix.Add((float*)d_aux,(float*)aux2,N,1,(float*)d_aux);
      
      if(d_aux[my_address-1][0]<0) 
        sol[1] =0;
      if(d_aux[my_address-1][0]>255) 
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
      Matrix.Print((float*)d_aux,N,1,"aqui?");
      
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
      if(d_aux[my_address-1][0]>255) 
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
      u_1_2_3=255;
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
      
      if(d_aux[my_address-1][0]>255) 
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
      u_4_5[1][0]=255;
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
      
      //Matrix.Print((float*)d_av,N,1,"dav");
      //Matrix.Print((float*)d,N,N,"d_1");
      //Serial.println(min_best);
      
      byte a[N];
      for(int l=0;l<N;++l)
        a[l]=(byte)d[l][my_address-1];
      //step2=1;
      
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
      //Matrix.Print((float*)y,N,1,"y_int");
      
      step2=1;
    }
  Matrix.Print((float*)d,N,N,"dbest");
  Matrix.Print((float*)d_av,N,1,"dav_final");
  //Matrix.Print((float*)y,N,1,"y_int");
  return  d_av[my_address-1][0];
}

void receiveEvent(int howMany)	{
  while(Wire.available() >	0)	{
    char phase=0;
    int h=0;
    int b=0;
    if(howMany>0) phase=Wire.read();
    if(howMany>1) h=Wire.read();
    if(howMany>2) b=Wire.read();
    //Serial.println(phase);
    //Serial.println(h);
    //Serial.println(b);
    if(phase==107){
      if(step!=my_address && step<=N){
        //dcycle=Wire.read();
        sensorValue=analogRead(analogInPin); //then reads in it's OWN LDR
	      sensorVoltage=map_double((double)sensorValue,0,1023,0,5);
	      sensorLux=lux_converter(sensorVoltage);
	      if(h>50){ 
	        other_sum+=(sensorLux-o)/h;
	        count_sum++;
	      }
	      count++;
	      if(h==256-1 && count>(C-1)*256+100){
	        K[step-1]=other_sum/count_sum;
	        count=0;
	        count_sum=0;
	        other_sum=0;
	        step++;
	      }
      }
    }
    if(phase==99){
      d[h][step2-1]=b;
      if(h==N-1) step2++;
    }
    if(phase==122){
      call_consensus=1;
    }
    if(phase==114){
      calibrated=0;
    }
    if(phase==115){
      if(h==(my_address-1)*2+1) {change_occupancy[1]=0; change_occupancy[0]=1;}
      if(h==(my_address-1)*2+2) {change_occupancy[1]=1; change_occupancy[0]=1;}
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
  //for(int l=0;l<N;++l)
  //  d_av[l][0]=20;
}

void loop() {
  if(!calibrated){
    calibrated=loop_calib();
    myPID.SetReference();
    Serial.println(o);
    Serial.println(K[0],5);
    Serial.println(K[1],5);
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
  myPID.Control(sensorLux);  //computes with input in lux
  //Serial.println(sensorLux);
  /*String message = String(myPID.GetReference());
  message += (" "+String(sensorLux));
  message += (" "+String(myPID.GetPWMPercent()));
  message += (" "+String(fftoggle));
  message += (" "+String(millis()));
  Serial.println(message);*/
  
  byte li=(byte)sensorLux;
  byte di=(byte)(myPID.GetFullResponse());
  byte occi=(byte)(myPID.GetOccupancy());
  Wire.beginTransmission(0);
  Wire.write(my_address);
  Wire.write(li);
  Wire.write(di);
  Wire.write(occi);
  Wire.endTransmission();

  unsigned long endTime=millis();                     //time at end of loop
  delay(sample-(endTime-startTime));                  //delay to avoid over computation of "control"
  
  while (Serial.available()) {
    byte serial_read=Serial.parseInt();
    if(serial_read!=0){
      if(serial_read==(my_address-1)*2+1) {change_occupancy[1]=0; change_occupancy[0]=1;}
      if(serial_read==(my_address-1)*2+2) {change_occupancy[1]=1; change_occupancy[0]=1;}
      Serial.println(serial_read);
      if(serial_read<N*2+1){
        Wire.beginTransmission(0);
        Wire.write("s");
        Wire.write(serial_read);
        Wire.endTransmission();
      }
      if(serial_read==N*2+1){ 
        Wire.beginTransmission(0);
        Wire.write("r");
        Wire.endTransmission();
        calibrated=0;
      }
    }
  }
  if(call_consensus) {myPID.SetReference(); call_consensus=0;}
  if(change_occupancy[0]) {myPID.SetOccupancy(change_occupancy[1]); change_occupancy[0]=0;}
  //Serial.flush();
}
