clear all
clc

arduino=serial('COM10','BaudRate',9600);

%if(fopen(arduino) ~= -1)
fopen(arduino);
    x=linspace(0,25*50*20,25*50);

    for(i=1:length(x))
        y(i)=fscanf(arduino,'%f');
    end
    fclose(arduino);
    plot(x,y);
%end


