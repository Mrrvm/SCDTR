#include "PID.h"


float consensus(float L);

//Constructor
PID::PID(){
  
  //set all members to default parameters
  Previous_input=0;
  I=0;
  D=0;
  Previous_error=0;
  Previous_Time=millis()-Sampling_Time;
  feedforward_on = 1;
  occupancy=1;
  consensus_on=1;
  
  reference_change=false;

}

//Destructor
PID::~PID(){
  
}

//Control loop function
bool PID::Control(double input){ 
  
  if(!reference_change && (input-reference)>20){
    calibrated=0;
    return false;
  }
  
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

    int feedback=u/M;//K[my_address-1];  //total feedback (in PWM)
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

void PID::SetOccupancy(bool occ){
  if(occ!=occupancy){
    Wire.beginTransmission(0); 
	  Wire.write("z");
    Wire.endTransmission();
    //Serial.println("trocou ocupacao");
    occupancy=occ;
    SetReference();
  }
}


void PID::SetReference(){
  //Serial.println("Setting reference...");
  if(occupancy) reference=HIGH; //50
  else reference=LOW;           //25
  //Serial.println(reference);
  if(consensus_on)
    Consensus();
  else
    feedforward=reference/K[my_address-1];
  Serial.println(feedforward);
  //feedforward=gain;
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

void PID::SetConsensus(bool cons){
  if(consensus_on!=cons){
    consensus_on=cons;
    SetReference();
  }
}

double PID::GetReference(){
  return reference;
}

double PID::GetPWMPercent(){
  return ((double)full_response/255)*100;
}

int PID::GetFullResponse(){
  return full_response;
}

bool PID::GetOccupancy(){
  return occupancy;
}




void PID::Consensus(){
  //Serial.println("entering consensus...");
  //unsigned long now=millis();
  float L=reference;
  float y[N][1]={0};
  float c[N][1]={0};
  float Q[N][N]={0};
  float d_av[N][1]={0};
  float rho=0.01;
  //y[0][0]=0;
  //y[1][0]=0;
  //for(int l=0;l<N;++l)
    //d_av[l][0]=0;
  c[my_address-1][0]=1;
  Q[my_address-1][my_address-1]=0;
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
      //Matrix.Print((float*)d_aux,N,1,"aqui?");
      
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
      
      unsigned long start_while=millis();
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
        if(millis()-start_while>5000+1000*C*N){ //detect if arduino is dead
          for(int l=0;l<N;++l) {d[l][step2-1]=0;}
          step2++;
          start_while=millis();
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

  Matrix.Print((float*)d_av,N,1,"dav_final");
  
  float newref=Matrix.Dot((float*)K,N,(float*)d_av)+o;
  if(newref>L) reference=newref;
  
  byte ref=(byte)reference;
  byte ox=(byte)o;
  Wire.beginTransmission(0);
  Wire.write("i");
  Wire.write("a");
  Wire.write(my_address);
  Wire.write("t");
  Wire.write(ref);
  Wire.write("x");
  Wire.write(ox);
  Wire.endTransmission();
  feedforward=d_av[my_address-1][0];
  
}


