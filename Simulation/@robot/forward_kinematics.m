function [end_effector] = forward_kinematics(bot)
     
    theta1 = bot.joint_angle(1);
    theta2 = bot.joint_angle(2);
    theta3 = bot.joint_angle(3);
    theta4 = bot.joint_angle(4);
    
    end_effector = double(subs(bot.fk))';
    
end
