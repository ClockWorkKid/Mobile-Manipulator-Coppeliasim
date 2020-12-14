clear all; close all; clc;

%% Initialization

bot = robot();

%% Coppeliasim

if (bot.clientID > -1)
    disp('Connection to robot successful');
    [~, ~, ~, ~] = bot.initialize_robot();
 
    bot.core_routine();
    %demo_door(bot)
    %demo_movement(bot)
    %demo_arm(bot)
    %forward_kinematics(bot)
   
    bot.terminate_robot();
    
else
    disp('Failed connecting to remote API server');
end

disp("Simulation ended");

%% Termination

bot.destructor();

disp('Connection terminated');

close all


%% Demonstration functions

% Demo door control
function [] = demo_door(bot)
    
    disp("Demo door control routine started");

    for i = 1:8
        bot.control_door(i, 1);
        pause(1);
        bot.control_door(i, 0);
        pause(1);
    end
    
    disp("Demo door control routine finished");

end

function [] = demo_movement(bot)
    disp("Demo movement routine started");

    figure()
    % demo code for going back and forth 5 times
    for i = 1:5 % change with while loop later
        
        tic
        wheel_velocity = [6, 6, 6, 6];
        bot.set_wheel_velocity(wheel_velocity);
        
        while (toc < 2)
            bot.update_cameras();
            % bot.save_images('D:/output_images');
            imshow(bot.frame_left);
            drawnow;
        end
        
        
        tic
        wheel_velocity = [-6, -6, -6, -6];
        bot.set_wheel_velocity(wheel_velocity);
        
        while (toc < 2)
            bot.update_cameras();
            imshow(bot.frame_left);
            drawnow;
        end
        
    end
    
    % Halt robot after 5 iterations
    [wheel_velocity] = bot.lfr_routine(0);
    bot.set_wheel_velocity(wheel_velocity);
    
    close all;
    disp("Demo movement routine finished");

end

% Demo arm control
function [] = demo_arm(bot)

    disp("Demo arm control routine started");
    
    [~] = bot.update_joint_angle();
    angle = [1, 0.8, 0.6, 0.4, 2, 0, 0];
    bot.set_joint_position(angle);
    pause(2);
    
    %{
    [~] = bot.update_joint_angle();
    angle = bot.joint_angle + [-1, -0.5, -0.5, 0, 0, 0, 0];
    bot.set_joint_position(angle);
    pause(2);
    
    [~] = bot.update_joint_angle();
    angle = [0, 0, 0, 0, 0, 0, 0];
    bot.set_joint_position(angle);
    pause(2);
    %}
     
    disp("Demo arm control routine finished");
    
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


function [] = forward_kinematics(bot)
    
    bot.update_joint_angle();   % update to latest joint angles
    
    J5 = [0;0;0];   % Robot end effector in own frame
    
    T4 = bot.joint_position(4, :)';
    J4 = translate(T4, J5);
    [~, end_effector] = bot.sim.simxGetObjectPosition(bot.clientID , bot.joints(5), bot.joints(4), bot.sim.simx_opmode_blocking);
    disp("End effector in joint 4 frame");
    disp(J4');
    disp(end_effector);
    
    J3 = rotate(bot.joint_angle(4), J4);
    T3 = bot.joint_position(3, :)';
    J3 = translate(T3, J3); 
    [~, end_effector] = bot.sim.simxGetObjectPosition(bot.clientID , bot.joints(5), bot.joints(3), bot.sim.simx_opmode_blocking);
    disp("End effector in joint 3 frame");
    disp(J3');
    disp(end_effector);
    
    J2 = rotate(bot.joint_angle(3), J3);
    T2 = bot.joint_position(2, :)';
    J2 = translate(T2, J2);    
    [~, end_effector] = bot.sim.simxGetObjectPosition(bot.clientID , bot.joints(5), bot.joints(2), bot.sim.simx_opmode_blocking);
    disp("End effector in joint 2 frame");
    disp(J2');
    disp(end_effector);
    
    J1 = rotate(bot.joint_angle(2), J2);
    T1 = [0 0 -1; 0 1 0; 1 0 0] * bot.joint_position(1, :)';
    J1 = [0 0 1; 0 1 0; -1 0 0] * translate(T1, J1);  
    [~, end_effector] = bot.sim.simxGetObjectPosition(bot.clientID , bot.joints(5), bot.joints(1), bot.sim.simx_opmode_blocking);
    disp("End effector in joint 1 frame");
    disp(J1');
    disp(end_effector);
    
end







