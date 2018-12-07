
classdef EstimatorClass < handle
    properties
        XX= zeros(15,1)
        PX= zeros(15)
        
        appearances
        
    end
    
    methods
        % ----------------------------------------------
        % ----------------------------------------------
        function obj= EstimatorClass(imu, params)
            
            % Initial attitude
            obj.initialize_pitch_and_roll(imu, params)
            obj.XX(9)= deg2rad(params.initial_yaw_angle);
            
            % initialize covariance
            obj.PX(10:12, 10:12)= diag( [params.sig_ba,params.sig_ba,params.sig_ba] ).^2;
            obj.PX(13:15, 13:15)= diag( [params.sig_bw,params.sig_bw,params.sig_bw] ).^2;
            obj.appearances= zeros(1,300); % if there are more than 300 landmarks, something's wrong
            
        end
        % ----------------------------------------------
        % ----------------------------------------------
        function initialize_pitch_and_roll(obj, imu, params)
            % calculates the initial pitch and roll
            
            % compute gravity from static IMU measurements
            g_bar= mean( imu.msmt(1:3, params.numEpochInclCalibration), 2 );
            
            % Books method
            g_bar= -g_bar;
            obj.XX(7)= atan2( g_bar(2),g_bar(3) );
            obj.XX(8)= atan2( -g_bar(1), sqrt( g_bar(2)^2 + g_bar(3)^2 ) );
            
            % My method -- works for z-axis pointing down (accz < 0)
            % theta=  atan2( g_bar(1) , abs(g_bar(3)) );
            % phi=   -atan2( g_bar(2) , abs(g_bar(3)) );
        end
        % ----------------------------------------------
        % ----------------------------------------------
    end
end



