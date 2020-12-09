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
    
    