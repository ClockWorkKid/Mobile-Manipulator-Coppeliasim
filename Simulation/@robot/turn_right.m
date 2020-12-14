%{

The purpose of this function is to turn right at a junction. 

Author: Shafin Bin Hamid 

%}

function [] = turn_right(self)

%go forward until the extreme left and right sensors cannot see black
%anymore
while true
    self.set_wheel_velocity([3.5,3.5,3.5,3.5]);
    pause(4);
    sValues = readSensor(self);
    [error, lastSensor] = calculate_error(self, sValues);
    if lastSensor~=3
        break
    end
end

%stop
self.set_wheel_velocity(zeros(1,4));

%keep turning right till front camera goes out of black line
while sValues(3)==1
    self.set_wheel_velocity([-2,-2,2,2]);
    sValues = readSensor(self);
end

%stop
self.set_wheel_velocity(zeros(1,4));

%keep turning right till front camera sees black line again
while sValues(3)==0
    self.set_wheel_velocity([-2,-2,2,2]);
    sValues = readSensor(self);
end
self.set_wheel_velocity(zeros(1,4));

end