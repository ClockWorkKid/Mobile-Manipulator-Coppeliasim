%{

The purpose of this function is to read the current image captured by the
front camera and output a vector of length 5. This output essentially acts
like the digital output of 5 infrared sensors mounted on the head of a generic line following
robot, pointing down towards the surface on which the robot is moving.
 
The first index of the vector represents the leftmost sensor and the last index
of the vector represents the rightmost sensor. 

If the value at a certain index is 1, that means that sensor is currently
on top of a black line. And, if the value is 0, then that sensor is
cuurently on top of a white surface.

The binarized grayscale image is also returned to perform door control
logic in lfr_routine.

Author: Shafin Bin Hamid

%}

function [sValues, frame_front_binarized] = readSensor(self)

self.update_cameras();

frame_front = rgb2gray(self.frame_front);
frame_front_binarized = imbinarize(frame_front);

sensor_values = frame_front(end,:);
sensor_values = [sensor_values 127];

sValues = zeros(1,5);
for i=1:5
    sValues(i) = mean(sensor_values((i-1)*13+1 : i*13));
    if(sValues(i)<50) %Threshold of pixel intensity
        sValues(i) = 1;
    else
        sValues(i) = 0;
    end
end

end