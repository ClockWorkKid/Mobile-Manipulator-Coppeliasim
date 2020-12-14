%{

The purpose of this function is to quantify the deviation of the robot's head
with respect to the black line. 

Author: Shafin Bin Hamid 

%}

function [error,lastSensor] = calculate_error(self, sValues)

%calculate the number of sensors currently on the line
sumS = sum(sValues);

%if no sensor is on the line then return 10, otherwise calculate error
if (sumS==0)
    error=10; % otherwise 10 not possible because of discontinuity
else %Calculate line position
    sumWS = sValues(1)*10 + sValues(2)*20 + sValues(3)*30 + sValues(4)*40 + sValues(5)*50;
    linepos = sumWS/sumS;
    error = linepos - 30;
end

%Keep track of which on of the two extreme sensors was on the line, 1 == Leftmost sensor, 2 == rightmost sensor
if(sValues(1) == 1 && sValues(3) == 1 && sValues(5) == 1)
    lastSensor = 3;
elseif(sValues(5) == 1)
    lastSensor = 2;
elseif(sValues(1) == 1)
    lastSensor = 1;
else
    lastSensor = 0;
end

end