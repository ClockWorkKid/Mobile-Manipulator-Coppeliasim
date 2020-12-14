function [fk, J] = kinematics_symbolic(bot)
    
    syms theta1
    syms theta2
    syms theta3
    syms theta4
    
    J5 = [0;0;0];   % Robot end effector in own frame
    
    T5 = bot.joint_position(5, :)';
    J4 = translate(T5, J5);
    
    J3 = rotate(theta4, J4);
    T4 = bot.joint_position(4, :)';
    J3 = translate(T4, J3); 
    
    J2 = rotate(theta3, J3);
    T3 = bot.joint_position(3, :)';
    J2 = translate(T3, J2);    
    
    J1 = rotate(theta2, J2);
    T2 = [0 0 -1; 0 1 0; 1 0 0] * bot.joint_position(2, :)';
    J1 = [0 0 1; 0 1 0; -1 0 0] * translate(T2, J1);  
    
    J0 = rotate(theta1, J1);
    T1 = [-1 0 0; 0 -1 0; 0 0 1] * bot.joint_position(1, :)';
    J0 = [-1 0 0; 0 -1 0; 0 0 1] * translate(T1, J0);
    
    fk = J0;
    J = jacobian(fk, [theta1, theta2, theta3, theta4]);
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