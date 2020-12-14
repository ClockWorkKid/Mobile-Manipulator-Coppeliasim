function [] = arm_routine(self, command)
    
    
    
    %{
    [~] = self.update_joint_angle();
    if command == "pick"
        angle = [0.5, 0.6, 0.7, 0.8, 0, 0, 0];
        self.set_joint_position(angle);
    elseif command == "place"
        angle = [0, 0, 0, 0, 0, 0, 0];
        self.set_joint_position(angle);
    end
        
    pause(2);
    %}
    
    % forward_kinematics(self)
    del_theta = inverse_kinematics(self, [0.0, 0.0, 0.05]);
    del_theta = [reshape(del_theta, 1, 4), 0, 0, 0];
    % disp(self.joint_angle)
    % disp(del_theta)
    self.set_joint_position(self.joint_angle + del_theta);
    
end




