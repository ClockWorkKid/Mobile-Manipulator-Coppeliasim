clear all; close all; clc;

%% Initialization

bot = robot();

%% Coppeliasim

if (bot.clientID > -1)
    disp('Connection to robot successful');
    
    
    [~, ~, ~, ~] = bot.initialize_robot();
 
    % Demo door control
    for i = 1:8
        bot.control_door(i, 1);
        pause(1);
        bot.control_door(i, 0);
        pause(1);
    end
    
    % bot.core_routine();
    
    bot.terminate_robot();
    
else
    disp('Failed connecting to remote API server');
end

disp("Simulation ended");

%% Termination

bot.destructor();

disp('Connection terminated');

close all
