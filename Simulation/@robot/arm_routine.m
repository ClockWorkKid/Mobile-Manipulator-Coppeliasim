function [] = arm_routine(self, command)
    
    if command == "pick"
        [~, object_position] = detect_object(self);
        disp("Object at: " + num2str(object_position));

        self.update_joint_angle();
        end_effector = forward_kinematics(self);
        object_position(end) = end_effector(end);
        move_arm_horizontal(self, object_position);
    end
end

function [] = move_arm_horizontal(self, object_position)
    max_trans = 0.02;       % maximum distance arm can traverse in a loop

    % Update joint angles, calculate current position and take step
    while (1)
        self.update_joint_angle();
        end_effector = forward_kinematics(self);
        displacement = object_position - end_effector;
        disp_norm = norm(displacement);
        if disp_norm > max_trans
            displacement = max_trans/disp_norm * displacement;
        end
        
        del_theta = inverse_kinematics(self, displacement);
        del_theta = [reshape(del_theta, 1, 4), 0, 0, 0];
        self.set_joint_position(self.joint_angle + del_theta);
        
        if disp_norm < max_trans
            break;
        end
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




