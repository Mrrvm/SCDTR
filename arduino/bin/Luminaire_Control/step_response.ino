const int analogInPin = A0;
const int analogOutPin = 9;
const double a=-0.61092;
const double b=4.69897;
int sensorValue=0;
double sensorVoltage=0;
double sensorLux=0;
int stop=0;

void setup() {
  Serial.begin(9600);  
}

double lux_converter(double volt){
  double l=pow(((5/volt)-1)*10000/pow(10,b),1/a);
  return l;
}

double map_double(double vi, int vi_min, int vi_max, int vo_min, int vo_max){
  return (double)(vi-vi_min)*(vo_max-vo_min)/(double)(vi_max-vi_min)+vo_min;
}

void loop() {

  if(stop==0){
  	for(int i=0;i<=250;i=i+10){   
	    analogWrite(analogOutPin,i);
	    //delay(500);
	    for(int j=0;j<50;++j){
	      sensorValue=analogRead(analogInPin);
	      sensorVoltage=map_double((double)sensorValue,0,1023,0,5);
	      //sensorLux=lux_converter(sensorVoltage);
	      //Serial.print("Sinal de entrada:");
	      Serial.println(sensorVoltage);
	      //Serial.print("PWM:");
	      //Serial.println(i);
	      //Serial.print("\n");
	      //Serial.print("Luminusidade:");
	      //Serial.println(sensorLux);
	      delay(10);
	    }
  	}
  	stop=1;
  }
  delay(10);
}