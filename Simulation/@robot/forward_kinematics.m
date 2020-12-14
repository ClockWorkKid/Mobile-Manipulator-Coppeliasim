function [end_effector] = forward_kinematics(bot)
    
    bot.update_joint_angle();   % update to latest joint angles
 
    J5 = [0;0;0];   % Robot end effector in own frame
    
    T5 = bot.joint_position(5, :)';
    J4 = translate(T5, J5);
    %[~, end_effector] = bot.sim.simxGetObjectPosition(bot.clientID , bot.joints(5), bot.joints(4), bot.sim.simx_opmode_blocking);
    %disp("End effector in joint 4 frame");
    %disp(J4');
    %disp(end_effector);
    
    J3 = rotate(bot.joint_angle(4), J4);
    T4 = bot.joint_position(4, :)';
    J3 = translate(T4, J3); 
    %[~, end_effector] = bot.sim.simxGetObjectPosition(bot.clientID , bot.joints(5), bot.joints(3), bot.sim.simx_opmode_blocking);
    %disp("End effector in joint 3 frame");
    %disp(J3');
    %disp(end_effector);
    
    J2 = rotate(bot.joint_angle(3), J3);
    T3 = bot.joint_position(3, :)';
    J2 = translate(T3, J2);    
    %[~, end_effector] = bot.sim.simxGetObjectPosition(bot.clientID , bot.joints(5), bot.joints(2), bot.sim.simx_opmode_blocking);
    %disp("End effector in joint 2 frame");
    %disp(J2');
    %disp(end_effector);
    
    J1 = rotate(bot.joint_angle(2), J2);
    T2 = [0 0 -1; 0 1 0; 1 0 0] * bot.joint_position(2, :)';
    J1 = [0 0 1; 0 1 0; -1 0 0] * translate(T2, J1);  
    %[~, end_effector] = bot.sim.simxGetObjectPosition(bot.clientID , bot.joints(5), bot.joints(1), bot.sim.simx_opmode_blocking);
    %disp("End effector in joint 1 frame");
    %disp(J1');
    %disp(end_effector);
    
    J0 = rotate(bot.joint_angle(1), J1);
    T1 = [-1 0 0; 0 -1 0; 0 0 1] * bot.joint_position(1, :)';
    J0 = [-1 0 0; 0 -1 0; 0 0 1] * translate(T1, J0);
    %[~, end_effector] = bot.sim.simxGetObjectPosition(bot.clientID , bot.joints(5), bot.bot_ref, bot.sim.simx_opmode_blocking);
    %disp("End effector in bot_ref frame");
    %disp(J0');
    %disp(end_effector);
    
    end_effector = J0;
    
end

function Y = translate(T, X)
    A = eye(4);
    A(1:3, 4) = T;
    X = reshape(X, 3,1);
    X = [X; 1];
    Y = A*X;
    Y = Y(1:3);
end

function Y = rotate(theta, X)
    R = [cos(theta), -sin(theta), 0;
        sin(theta), cos(theta), 0;
        0, 0, 1];
    X = reshape(X, 3,1);
    Y = R*X;
end
