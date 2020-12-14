%{

The purpose of this function is to do a 180 degree rotation. 

Author: Shafin Bin Hamid 

%}

function [] = flip(self)

sValues = readSensor(self);

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