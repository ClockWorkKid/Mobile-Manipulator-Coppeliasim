%{

This is a subprocess under core_routine. The subprocess is called twice from
the core routine. In the first pass, it takes a given destination as input and
it ends when the robot reaches that destination following the black line.
After the robot has finished its work, it will need to come back to the
reference point. So, in the second pass, this subprocess is again called
and it ends when the robot reaches the reference point following the black
line.

In between, this code will call other functions to estimate it's current
position and control the motor based on that position. 

Author: Shafin Bin Hamid & Himaddri Roy

%}

function [] = lfr_routine(self, destination)

lastSensor = 0;
baseSpeed = 6;
Kp = 0.6;

%update the cameras 10 times to avoid frame_right error
for i=1:10
    self.update_cameras();
end

j=1;

while true
    
    [sValues, frame_front_binarized] = readSensor(self);
    %sValues
    
    %sliding door control logic
    sensor_door=~frame_front_binarized(1,:);
    sensor_door=all(sensor_door);
    
    if sensor_door  %main door control first time
        j=0;
        self.control_door(7,1);
        self.control_door(8,1);
    else
        %error calculation
        [error, lastSensor] = calculate_error(self, sValues);
        
        %control based on error
        if (error==10)
            if (lastSensor==1)
                self.set_wheel_velocity([2,2,-2,-2]); %first two values for right wheels
            elseif(lastSensor==2)
                self.set_wheel_velocity([-2,-2,2,2]);
            elseif(lastSensor==0)
                self.set_wheel_velocity(zeros(1,4));
                self.control_door(7,0);
                self.control_door(8,0);
                break;
            end
        else   
            if(lastSensor==3) %junction check
                self.set_wheel_velocity([0,0,0,0]);
                
                right_wall = check_wall(self, self.frame_right, destination);
                left_wall = check_wall(self, self.frame_left, destination);
                front_wall = check_wall(self, self.frame_front, destination);
                
                if right_wall
                    self.control_door(destination,1);
                    self.control_door(7,0);
                    self.control_door(8,0);
                    turn_right(self);
                elseif left_wall
                    self.control_door(destination,1);
                    self.control_door(7,0);
                    self.control_door(8,0);
                    turn_left(self);
                elseif front_wall
                    self.control_door(7,1);
                    self.control_door(8,1);
                    if mod(destination,2)==0 
                        self.control_door(destination-1,0);
                        turn_left(self);
                    elseif mod(destination,2)==1
                        self.control_door(destination+1,0);
                        turn_right(self);
                    end
                else
                    self.set_wheel_velocity([5,5,5,5]);
                    pause(2);
                end
            end
            
            delSpeed = Kp*error;
            self.set_wheel_velocity([baseSpeed-delSpeed, baseSpeed-delSpeed, baseSpeed+delSpeed, baseSpeed+delSpeed]);
        end
    end
end

end




