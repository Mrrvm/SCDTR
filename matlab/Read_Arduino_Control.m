clear all
clc
j = 0;
f = 0;
i = 1;
arduino=serial('COM10','BaudRate',9600);
fopen(arduino);

 while i>0
     A(:,i)=fscanf(arduino,'%f  %u', [2 100]);
     %if (y(i) == -1)
     %    break
     %end
     i = i +1;
 end
 tempo = fscanf(arduino,'%f');
 x=linspace(0,255,255);
 fclose(arduino);
 %y = y(1:i-1);
 plot(x,y);

