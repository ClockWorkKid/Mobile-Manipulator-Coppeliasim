function [del_theta] = inverse_kinematics(bot, del_displacement)

    theta1 = bot.joint_angle(1);
    theta2 = bot.joint_angle(2);
    theta3 = bot.joint_angle(3);
    theta4 = bot.joint_angle(4);
    
    J_num = double(subs(bot.J));
    J_inv = pinv(J_num);

    del_theta = mod(J_inv * reshape(del_displacement, 3, 1), 2*pi);
    for i = 1:4
        if del_theta(i) > pi
            del_theta(i) = del_theta(i) - 2*pi;
        end
    end

end