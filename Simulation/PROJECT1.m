clear all;close all;clc;
%% Initialization
sim=remApi('remoteApi');
sim.simxFinish(-1);
clientID=sim.simxStart('127.0.0.1',19999,true,true,5000,5);

%% Connecting to Coppeliasim

if (clientID>-1)
    disp('Connected hoisi mama');
    
    %% Code
    
    Wheeljoints_Handles=zeros(1,4);
    Wheel_names=["rollingJoint_fl","rollingJoint_rl", "rollingJoint_rr", "rollingJoint_fr"];
    
    for i=1:4
        [~,Wheeljoints_Handles(i)]=sim.simxGetObjectHandle(clientID,uint8(char(Wheel_names(i))),sim.simx_opmode_blocking);
    end
    
    
    Linear_vel=-3;

    [fl_speed rl_speed rr_speed fr_speed]=deal(Linear_vel);
    velvector=[fl_speed rl_speed rr_speed fr_speed];
    
    for i=1:4
        [~]=sim.simxSetJointTargetVelocity(clientID,Wheeljoints_Handles(i),velvector(i),sim.simx_opmode_blocking);
    end
    
    [ReturnCode,Sensor]=sim.simxGetObjectHandle(clientID,'Vision_sensor',sim.simx_opmode_blocking);
    [ReturnCode,detectionState,~,~]=sim.simxReadVisionSensor(clientID,Sensor,sim.simx_opmode_streaming);
    
    %% Asynchronous operation
    i=1;
    gain_factor=0.5;thresold=0.1529;
    
    while(1)
        
        if (sim.simxGetObjectHandle(clientID,'Vision_sensor',sim.simx_opmode_blocking))==sim.simx_return_ok
            [ReturnCode,detectionState,auxData(i,:),auxPacketInfo]=sim.simxReadVisionSensor(clientID,Sensor,sim.simx_opmode_buffer);
        end
        
        disp(auxData(i,11));
        if auxData(i,11)>0.2
            disp('SORE GESE')
        end
        
        % Tracking
%         angVel= -gain_factor*(auxData(i,11)-thresold);
%         wL=(-Linear_vel-b*angVel)/0.5;
%         wR=(-Linear_vel+b*angVel)/0.5;
%         
%         From Coppeliasim script
%         
%         sim.setJointTargetVelocity(wheelJoints[1],-forwBackVel-leftRightVel-rotVel)
%         sim.setJointTargetVelocity(wheelJoints[2],-forwBackVel+leftRightVel-rotVel)
%         sim.setJointTargetVelocity(wheelJoints[3],-forwBackVel-leftRightVel+rotVel)
%         sim.setJointTargetVelocity(wheelJoints[4],-forwBackVel+leftRightVel+rotVel)
%         
%         sim.simxSetJointTargetVelocity(clientID,Wheeljoints_Handles(1),,sim.simx_opmode_blocking);
%         sim.simxSetJointTargetVelocity(clientID,Wheeljoints_Handles(2),,sim.simx_opmode_blocking);
%         sim.simxSetJointTargetVelocity(clientID,Wheeljoints_Handles(3),,sim.simx_opmode_blocking);
%         sim.simxSetJointTargetVelocity(clientID,Wheeljoints_Handles(4),,sim.simx_opmode_blocking);
        
        i=i+1;
        pause(0.05);
        
    end
    
    %     angVel= -gain_factor*(lineColor-grey)
    
    %
    %     return wL,wR
    
    %% Synchronous operation
    
    %     sim.simxSynchronous(clientID,true);
    %     sim.simxStartSimulation(clientID,sim.simx_opmode_oneshot);
    %
    %     while(sim.simxSynchronousTrigger(clientID))
    %
    %         [ReturnCode,detectionState,auxData,auxPacketInfo]=sim.simxReadVisionSensor(clientID,Sensor,sim.simx_opmode_buffer);
    %
    %         disp(auxData(11));
    %         pause(0.3);
    %     end
    
    
    %% close the connection to CoppeliaSim
    sim.simxFinish(clientID);
    
else
    disp('Failed connecting to remote API server');
end

sim.delete(); % call the destructor!

disp('Program ended');