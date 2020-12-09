clear all; close all; clc;

%% Initialization

bot = robot();

%% Coppeliasim

if (bot.clientID > -1)
    disp('Connection to robot successful');
    
    [~, ~, ~] = bot.initialize_robot();

    bot.core_routine();
    
    bot.terminate_robot();
    
else
    disp('Failed connecting to remote API server');
end

disp("Simulation ended");

%% Termination

bot.destructor();

disp('Connection terminated');

close all
