function [joint_target] = arm_routine(self, command)

    joint_target = zeros(size(self.joint));
    if command == "pick"
        joint_target = [1, 1, 1, 1, 1, 1];
    elseif command == "place"
        joint_taget = [0, 1, 0, 1, 0, 1];
    end
    
end