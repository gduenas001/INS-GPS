
classdef GPSClass < handle
    
    properties (SetAccess = immutable) % parameters to be built with constructor
        num_readings
        time
        timeInit
        R
        msmt
        R_NE % rot matrix from ECEF to Nav-frame
        
    end
    
    methods
        function obj= GPSClass(timeStatic, params)
            
            numEpochStaticGPS= ceil(timeStatic);
            
            % loads "data" variable
            load(params.file_name_gps);
            
            obj.num_readings= length(data);
            obj.time= data(:,4);
            posX=     data(:,5);
            posY=     data(:,6);
            posZ=     data(:,7);
            velX=     data(:,8);
            velY=     data(:,9);
            velZ=     data(:,10);
            sigPosX=  data(:,11);
            sigPosY=  data(:,12);
            sigPosZ=  data(:,13);
            sigVelX=  data(:,14);
            sigVelY=  data(:,15);
            sigVelZ=  data(:,16);
            
            % Save the initial time as reference for other sensors
            obj.timeInit= obj.time(1);
            
            % Make time start at zero
            obj.time= obj.time - obj.time(1);
            
            % create variables
            obj.msmt= [posX, posY, posZ, velX, velY, velZ]';
            obj.R= ([sigPosX, sigPosY, sigPosZ, sigVelX, sigVelY, sigVelZ].^2)';
            
            % Use initial position as reference
            muX= mean(posX(1:numEpochStaticGPS));
            muY= mean(posY(1:numEpochStaticGPS));
            muZ= mean(posZ(1:numEpochStaticGPS));
            obj.msmt(1:3,:)= obj.msmt(1:3,:) - [muX;muY;muZ];
            
            % Convert from ECEF to Navigation-frame
            [phi, lambda, ~]=  ecef2lla(muX,muY,muZ) ;
            
            obj.R_NE= [-sin(phi)*cos(lambda), -sin(phi)*sin(lambda),  cos(phi);
                       -sin(lambda),           cos(lambda),           0;
                       -cos(phi)*cos(lambda), -cos(phi)*sin(lambda), -sin(phi)];
            
            R_NE_block= blkdiag( obj.R_NE, obj.R_NE );
            obj.msmt= R_NE_block* obj.msmt;
            
            for i= 1:length(obj.time)
                obj.R(:,i)= diag( R_NE_block * diag( obj.R(:,i) ) * R_NE_block' );
            end
            
            
            
            
            % increase GPS variance
            obj.R(1:3,:)= obj.R(1:3,:)*(params.multFactorPoseGPS^2); %%%%%%%%%%%%%%%%%% CAREFUL
            obj.R(4:6,:)= obj.R(4:6,:)*(params.multFactorVelGPS^2);  %%%%%%%%%%%%%%%%%% CAREFUL
            
        end
    end
end








