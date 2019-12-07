
classdef SMClass < handle %This is a Scan Matching Substitution to GPS Class
    
    properties (SetAccess = immutable) % parameters to be built with constructor
        num_readings
        time
        timeInit
        R    % covariance matrices for msmts
        msmt
        R_NE % rot matrix from ECEF to Nav-frame
        
    end
    
    properties
        IS_SM_AVAILABLE
    end
    
    methods
        % ----------------------------------------------
        % ----------------------------------------------
        function obj= SMClass(timeStatic, params)
            
            % this indicator is used for collecting gps data recieved at lidar epoch (for FG)
            obj.IS_SM_AVAILABLE= 0;
            
            if params.SWITCH_SIM
                obj.time= 0;
                return
            end
            
            numEpochStaticSM= ceil(timeStatic);
            
            % loads "data" variable
            load(params.file_name_SM);
            
            obj.num_readings= length(data);
            obj.time= data(:,1);
            posX=     data(:,2);
            posY=     data(:,3);
            posZ=     data(:,4);
            roll =    data(:,8);
            pitch=    data(:,9);
            yaw  =    data(:,10);
            sigPosX = 0.02+zeros(obj.num_readings,1);
            sigPosY = 0.02+zeros(obj.num_readings,1);
            sigPosZ = 0.02+zeros(obj.num_readings,1);
            sigroll = 0.01+zeros(obj.num_readings,1);
            sigpitch = 0.01+zeros(obj.num_readings,1);
            sigyaw = 0.01+zeros(obj.num_readings,1);
            % Save the initial time as reference for other sensors
            obj.timeInit= obj.time(1);
            
            % Make time start at zero
            obj.time= obj.time - obj.time(1);
            
            % create variables
            obj.msmt= [posX, posY, posZ, roll, pitch, yaw]';
            obj.R= ([sigPosX, sigPosY, sigPosZ,sigroll, sigpitch, sigyaw].^2)';
            
            % Use initial position as reference
            muX= mean(posX(1:numEpochStaticSM));
            muY= mean(posY(1:numEpochStaticSM));
            muZ= mean(posZ(1:numEpochStaticSM));
            muroll  = mean(roll(1:numEpochStaticSM));
            mupitch = mean(pitch(1:numEpochStaticSM));
            muyaw   = mean(yaw(1:numEpochStaticSM));
            obj.msmt(1:6,:)= obj.msmt(1:6,:) - [muX;muY;muZ;muroll;mupitch;muyaw];  
        end
        % ----------------------------------------------
        % ----------------------------------------------
    end
end








