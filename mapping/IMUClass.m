
classdef IMUClass < handle
    properties (SetAccess = immutable)
        
        num_readings
        time
        
        inc_msmt % msmts for the inclinometers
        msmt     % msmts for the accs & gyros
        
    end
    
    methods
        function obj= IMUClass(params, initial_time)
            % DATAREAD Reads a datafile and outputs a series of vectors of the data
            % Data id in columns
            
            % load "data" for the IMU
            load(params.file_name_imu);
            
            g0= 9.80665; % m/s^2 -- by manufacturerw -- TODO: include in parameters
            
            obj.time= data(:,4);
            gyroX   = deg2rad( data(:,5) );
            gyroY   = deg2rad( data(:,6) );
            gyroZ   = deg2rad( data(:,7) );
            % gyroSts = data(:,8);
            accX    = data(:,9)  *g0;
            accY    = data(:,10) *g0;
            accZ    = data(:,11) *g0;
            % accSts  = data(:,12);
            incX    = data(:,13) *g0;
            incY    = data(:,14) *g0;
            incZ    = data(:,15) *g0;
            % incSts  = data(:,16);
            
            
            % Use the first GPS reading time as reference
            obj.time= obj.time - initial_time;
            
            % Set paramters
            obj.inc_msmt= [incX, incY, incZ]';
            obj.msmt= [accX, accY, accZ, gyroX, gyroY, gyroZ]';
            
            % num of readings
            obj.num_readings= size(obj.msmt,2);
            
            
            
            
            % ------------ Initial calibration ------------
            load(params.file_name_calibration); % loads iinvC, invC, ib_0, b_0
            invC= [invC, zeros(3); zeros(3), eye(3)];
            
            % Initial rotation to get: x=foward & z=down
            R_init= [ 0, -1, 0;
                      -1, 0, 0;
                      0, 0, -1];
            R_init_block= blkdiag(R_init,R_init);
            
            obj.inc_msmt= (iinvC * obj.inc_msmt) - ib_0;
            obj.msmt= (invC * obj.msmt) - b_0;
            obj.msmt= R_init_block * obj.msmt;
            obj.inc_msmt= R_init * obj.inc_msmt;
            % -------------------------------------------
            
            
            % reduce the number of epochs to test
            if params.SWITCH_NUM_of_LOOPS
                obj.num_readings= params.numEpochStatic + 5000;
            elseif params.SWITCH_NUM_of_LOOPS
                obj.num_readings= 46500; %%%%%%%%%%%%% CAREFUL
            end 
        end
    end
end







