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
        [wheel_velocity] = self.lfr_routine(-1);
        self.set_wheel_velocity(wheel_velocity);
        
        while (toc < 2)
            self.update_cameras();
            % self.save_images('D:/output_images');
            imshow(self.frame_left);
            drawnow;
        end
        
        % self.update_proximity();
        
        tic
        [wheel_velocity] = self.lfr_routine(1);
        self.set_wheel_velocity(wheel_velocity);
        
        while (toc < 2)
            self.update_cameras();
            % self.save_images('D:/output_images');
            imshow(self.frame_left);
            drawnow;
        end
        
        
        % [joint_target] = self.arm_routine();
        % self.set_joint_target(joint_target);
        

        % pause(0.05); % Not needed when running blocking statements

    end
    
    % Halt robot after 5 iterations
    [wheel_velocity] = self.lfr_routine(0);
    self.set_wheel_velocity(wheel_velocity);

end

