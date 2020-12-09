function [wheel_velocity] = lfr_routine(self, destination)

    wheel_velocity = zeros(size(self.motors));
    gain_factor = 0.5;
    threshold = 0.1529;
    
    if destination == 1
        wheel_velocity = [6, 6, 6, 6];
    elseif destination == -1
        wheel_velocity = [-6, -6, -6, -6];
    end
    
    % Tracking
    % angVel= -gain_factor*(auxData(i,11)-thresold);
    % wL=(-Linear_vel-b*angVel)/0.5;
    % wR=(-Linear_vel+b*angVel)/0.5;

end