%{

The core routine runs a while loop. This routine handles all the necessary
inputs and outputs of the robot. This routine also holds the core control
structure of the robot (what to do, when to do, how to do)

In each iteration of the core routine, image and proximity data will be
recieved from the simulator, necessary sub-processes will be called, motor
and joint commands will be sent back to the simulator

Inputs:
1. Vision sensor (Camera Images)
2. Proximity sensor

Outputs:
1. Motor Velocity/Position
2. Arm Joint angle

Subprocesses 
1. lfr_routine.m (INPUTS: destination OUTPUTS: motor)
2. arm_routine.m (INPUTS: command(pick/place) OUTPUTS: joint angles)

*subprocesses can access images/proximity data from self as necessary

%}

function [] = core_routine(self) % NOT YET IMPLEMENTED, DEMO STRUCTURE

    figure()
    % demo code for going back and forth 5 times
    for i = 1:5 % change with while loop later
        
        tic
        self.update_cameras();
        subplot(131), imshow(squeeze(bot.frames(1, :, :, :)));
        subplot(132), imshow(squeeze(bot.frames(1, :, :, :)));
        subplot(133), imshow(squeeze(bot.frames(1, :, :, :)));
        drawnow;
        disp(1/toc)
        
        % self.update_proximity();
        
        [wheel_velocity] = self.lfr_routine(1);
        self.set_wheel_velocity(wheel_velocity);
        pause(2);
        
        [wheel_velocity] = self.lfr_routine(-1);
        self.set_wheel_velocity(wheel_velocity);
        pause(2);
        % [joint_target] = self.arm_routine();
        % self.set_joint_target(joint_target);
        

        pause(0.05);

    end
    
    % Halt robot after 5 iterations
    [wheel_velocity] = self.lfr_routine(0);
    self.set_wheel_velocity(wheel_velocity);

end

