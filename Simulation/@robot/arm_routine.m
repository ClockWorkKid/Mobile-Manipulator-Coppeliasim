function [] = arm_routine(self, command)
    
    self.update_joint_angle();
    retracted_angles = self.joint_angle(1:4);
    % disp("Original angles" + num2str(retracted_angles));
    
    if command == "pick"
        [~, object_position] = detect_object(self);   % find object position
        disp("Object at: " + num2str(object_position));
        disp("Picking up object");
        % object_height = object_position(end);
        
        end_effector = forward_kinematics(self);      % find arm position
        object_position(end)=end_effector(end)+0.06;  % for horizontal movement
        move_arm(self, object_position);              % horizontally move arm to target
        
        object_position(end) = object_position(end) - 0.16;
        move_arm(self, object_position);              % vertically move arm
        
        gripper(self, "hold");                        % grip on object
        
        object_position(end) = object_position(end) + 0.16;
        move_arm(self, object_position);              % vertically move arm
        
        self.joint_angle = [retracted_angles, self.joint_angle(5:7)];
        % disp("Return to" + num2str(self.joint_angle));
        self.set_joint_position(self.joint_angle);    % move to original position
        pause(5);
        
        gripper(self, "release");                     % release object
    
    elseif command == "place"
        disp("Placing object");
        end_effector = forward_kinematics(self);      % find arm position
        object_position = end_effector;
        object_position(end)=object_position(end)-.06;% for vertical movement
        move_arm(self, object_position);              % vertically move arm to target
        
        gripper(self, "hold");                        % grip on object

        object_position(end)=object_position(end)+.06;% for vertical movement
        move_arm(self, object_position);              % vertically move arm
        move_arm(self, [-0.45 -0.25 0.1]);            % move to drop zone
        pause(4);
        gripper(self, "release");                     % release object
        
        self.joint_angle = [retracted_angles, self.joint_angle(5:7)];
        % disp("Return to" + num2str(self.joint_angle));
        self.set_joint_position(self.joint_angle);    % move to original position
        pause(4);
    end
    
end

function [] = move_arm(self, object_position)
    max_trans = 0.02;       % maximum distance arm can traverse in a loop
    self.update_joint_angle();
    % Update joint angles, calculate current position and take step
    while (1)
        %self.update_joint_angle();
        end_effector = forward_kinematics(self);
        displacement = object_position - end_effector;
        disp_norm = norm(displacement);
        if disp_norm > max_trans
            displacement = max_trans/disp_norm * displacement;
        end
        
        del_theta = inverse_kinematics(self, displacement);
        del_theta = [reshape(del_theta, 1, 4), 0, 0, 0];
        self.joint_angle = self.joint_angle + del_theta;
        self.set_joint_position(self.joint_angle);
        
        if disp_norm < max_trans
            break;
        end
    end
    
    self.update_joint_angle();
end

function [] = gripper(self, action)

    if action == "hold"
        % disp("Gripping object")
        grip_size = 0.02;
        self.sim.simxSetJointTargetPosition(self.clientID , self.joints(6), +3.041e-02 - grip_size, self.sim.simx_opmode_oneshot);
        self.sim.simxSetJointTargetPosition(self.clientID , self.joints(7), -6.081e-02 + grip_size, self.sim.simx_opmode_oneshot);
        pause(2);
    elseif action == "release"
        % disp("Releasing object")
        self.sim.simxSetJointTargetPosition(self.clientID , self.joints(6), +3.041e-02, self.sim.simx_opmode_oneshot);
        self.sim.simxSetJointTargetPosition(self.clientID , self.joints(7), -6.081e-02, self.sim.simx_opmode_oneshot);
        pause(2);
    end
end

function [err_code, object_position] = detect_object(self)

    [~, object_handle] = self.sim.simxGetObjectHandle(self.clientID,uint8(char("Object")),self.sim.simx_opmode_blocking);
    [err_code, object_position] = self.sim.simxGetObjectPosition(self.clientID , object_handle, self.bot_ref, self.sim.simx_opmode_blocking);
    
    %{
    self.update_cameras();
    pause(1);
    imshow(self.frame_left);
    err_code = 0;
    object_position = [0, 0];
    %}
end

function [] = demo_IK(bot)
    del_theta = inverse_kinematics(bot, [0.0, 0.0, 0.05]);
    del_theta = [reshape(del_theta, 1, 4), 0, 0, 0];
    bot.set_joint_position(bot.joint_angle + del_theta);
end




