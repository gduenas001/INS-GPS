
classdef EstimatorClass < handle
    properties (SetAccess = immutable)
        landmark_map
    end
    properties
        XX= zeros(15,1)
        PX= zeros(15)
        
        n_k
        
        gamma_k
        Y_k
        H_k
        L_k
        Phi_k   % state evolution matrix
        D_bar % covariance increase for the state evolution
        
        
        num_landmarks
        
        
        initial_attitude
        appearances
        
    end
    
    methods
        % ----------------------------------------------
        % ----------------------------------------------
        function obj= EstimatorClass(imu_calibration_msmts, params)
            
            % Initial attitude
            obj.initialize_pitch_and_roll(imu_calibration_msmts)
            obj.XX(9)= deg2rad(params.initial_yaw_angle);
            
            % save initial attitude for calibration
            obj.initial_attitude= obj.XX(7:9);
            
            % initialize covariance
            obj.PX(10:12, 10:12)= diag( [params.sig_ba,params.sig_ba,params.sig_ba] ).^2;
            obj.PX(13:15, 13:15)= diag( [params.sig_bw,params.sig_bw,params.sig_bw] ).^2;
            obj.appearances= zeros(1,300); % if there are more than 300 landmarks, something's wrong
            
            % load map if exists
            try
                data= load(strcat( params.path, 'landmark_map.mat' ));
                obj.landmark_map= data.landmark_map;
                obj.num_landmarks= size(obj.landmark_map, 1);
            catch
            end
            
        end
        % ----------------------------------------------
        % ----------------------------------------------
        function initialize_pitch_and_roll(obj, imu_calibration_msmts)
            % calculates the initial pitch and roll
            
            % compute gravity from static IMU measurements
            g_bar= mean( imu_calibration_msmts, 2 );
            
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
        function calibration(obj, imu_msmt, params)
            % create a fake msmt and do a KF update to calibrate biases.
            % Also increase D_bar so that biases keep changing (not too
            % small variance)
            
            % create a fake msmt and make a KF update
            z= [zeros(6,1); obj.initial_attitude];
            
            % Calibration msmt update
            L= obj.PX(1:15,1:15) * params.H_cal' /...
                (params.H_cal*obj.PX(1:15,1:15)*params.H_cal' + params.R_cal);
            z_hat= params.H_cal * obj.XX(1:15);
            innov= z - z_hat;
            innov(end)= pi_to_pi(innov(end));
            obj.XX(1:15)= obj.XX(1:15) + L*innov;
            obj.PX(1:15,1:15)= obj.PX(1:15,1:15) - L * params.H_cal * obj.PX(1:15,1:15);
            
            % linearize and discretize after every non-IMU update
            obj.linearize_discretize( imu_msmt, params.dt_imu, params);
            
            % If GPS is calibrating initial biases, increse bias variance
            obj.D_bar(10:12,10:12)= obj.D_bar(10:12,10:12) +...
                diag( [params.sig_ba,params.sig_ba,params.sig_ba] ).^2;
            obj.D_bar(13:15,13:15)= obj.D_bar(13:15,13:15) +...
                diag( [params.sig_bw,params.sig_bw,params.sig_bw] ).^2;
        end
        % ----------------------------------------------
        % ----------------------------------------------
        function imu_update( obj, imu_msmt, params )
            % updates the state with the IMU reading, NO cov update
            
            % Create variables (for clarity)
            v= obj.XX(4:6);
            phi= obj.XX(7); theta= obj.XX(8); psi= obj.XX(9);
            b_f= obj.XX(10:12);
            b_w= obj.XX(13:15);
            f= imu_msmt(1:3);
            w= imu_msmt(4:6);
            
            if params.SWITCH_CALIBRATION
                taua= params.taua_calibration;
                tauw= params.tauw_calibration;
            else
                taua= params.taua_normal_operation;
                tauw= params.tauw_normal_operation;
            end
            
            % Calculate parameters
            R_NB= R_NB_rot (phi,theta,psi);
            Q_BE= Q_BE_fn  (phi,theta);
            
            r_dot= v;
            v_dot= R_NB * ( f - b_f ) + params.g_N;
            E_dot= Q_BE * ( w - b_w );
            b_f_dot= -eye(3) / taua * b_f;
            b_w_dot= -eye(3) / tauw * b_w;
            x_dot= [r_dot; v_dot; E_dot; b_f_dot; b_w_dot];
            
            % udpate estimate
            obj.XX(1:15)= obj.XX(1:15) + params.dt_imu * x_dot;    
            obj.PX(1:15,1:15)= obj.Phi_k * obj.PX(1:15,1:15) * obj.Phi_k' + obj.D_bar;
        end
        % ----------------------------------------------
        % ----------------------------------------------
        function yaw_update(obj, w, params)
            
            
            n_L= (length(obj.XX) - 15) / 2;
            H= zeros(1, 15 + 2*n_L);
            H(9)= 1;
            
            z= obj.yawMeasurement(w, params);
            
            R= params.R_yaw_fn( norm(obj.XX(4:6)));
            L= obj.PX*H' / (H*obj.PX*H' + R);
            innov= z - H*obj.XX;
            innov= pi_to_pi(innov);
            obj.XX(9)= pi_to_pi(obj.XX(9));
            obj.XX= obj.XX + L*innov;
            obj.PX= obj.PX - L*H*obj.PX;
        end
        % ----------------------------------------------
        % ----------------------------------------------
        function yaw= yawMeasurement(obj, w, params)
            
            r= [-params.r_IMU2rearAxis; 0; 0];
            v_o= obj.XX(4:6);
            R_NB= R_NB_rot( obj.XX(7), obj.XX(8), obj.XX(9));
            
            v_a= v_o + R_NB * cross(w,r);
            v_a= v_a / norm(v_a);
            
            yaw= atan2(v_a(2),v_a(1));
        end
        % ----------------------------------------------
        % ----------------------------------------------
        function vel_update_z(obj, R)
            
            % Normalize yaw
            obj.XX(9)= pi_to_pi( obj.XX(9) );
            
            % Update
            R_BN= R_NB_rot( obj.XX(7), obj.XX(8), obj.XX(9) )';
            H= [0,0,1] * R_BN * [zeros(3),eye(3),zeros(3,9)];
            L= obj.PX*H' / (H*obj.PX*H' + R);
            z_hat= H * obj.XX;
            innov= 0 - z_hat;
            
            obj.XX= obj.XX + L*innov;
            obj.PX= obj.PX - L*H*obj.PX;
            
            % This is a different option to do the update in Z, but it is more
            % computationally expensive and does not offer better results in my case
            %{
                R_BN= R_NB_rot( x(7,k+1), x(8,k+1), x(9,k+1) )';
                H_virt= H_fn(x(4,k+1), x(5,k+1), x(6,k+1), x(7,k+1), x(8,k+1), x(9,k+1));
                L= P*H_virt' / (H_virt*P*H_virt' + R_virt_Z);
                z= 0;
                z_hat= [0,0,1] * R_BN * x(4:6,k+1);
                innov= z - z_hat;
                x(:,k+1)= x(:,k+1) + L*innov;
                P= P - L*H_virt*P;
            %}
        end
        % ----------------------------------------------
        % ----------------------------------------------
        function gps_update(obj, z, R, params)
            
           
            n_L= (length(obj.XX) - 15) / 2;
            
            if norm(z(4:6)) > params.min_vel_gps && params.SWITCH_GPS_VEL_UPDATE % sense velocity
                R= diag( R );
                H= [eye(6), zeros(6,9), zeros(6,n_L*2)];
                disp('GPS velocity')
            else
                z= z(1:3);
                R= diag( R(1:3) );
                H= [eye(3), zeros(3,12), zeros(3,n_L*2)];
                disp('-------- no GPS velocity ---------')
            end
            
            obj.XX(9)= pi_to_pi( obj.XX(9) );
            L= obj.PX*H' / (H*obj.PX*H' + R);
            innov= z - H*obj.XX;
            
            obj.XX= obj.XX + L*innov;
            obj.PX= obj.PX - L*H*obj.PX;
        end
        % ----------------------------------------------
        % ----------------------------------------------
        function association= nearest_neighbor(obj, z, params)
            
            n_F= size(z,1);
            n_L= (length(obj.XX) - 15) / 2;
            
            association= ones(1,n_F) * (-1);
            
            if n_F == 0 || n_L == 0, return, end
            
            
            spsi= sin(obj.XX(9));
            cpsi= cos(obj.XX(9));
            zHat= zeros(2,1);
            % Loop over extracted features
            for i= 1:n_F
                minY= params.threshold_new_landmark;
                
                for l= 1:n_L
                    ind= (15 + (2*l-1)):(15 + 2*l);
                    
                    dx= obj.XX(ind(1)) - obj.XX(1);
                    dy= obj.XX(ind(2)) - obj.XX(2);
                    
                    zHat(1)=  dx*cpsi + dy*spsi;
                    zHat(2)= -dx*spsi + dy*cpsi;
                    gamma= z(i,:)' - zHat;
                    
                    H= [-cpsi, -spsi, -dx*spsi + dy*cpsi,  cpsi, spsi;
                        spsi, -cpsi, -dx*cpsi - dy*spsi, -spsi, cpsi];
                    
                    Y= H * obj.PX([1:2,9,ind],[1:2,9,ind]) * H' + params.R_lidar;
                    
                    y2= gamma' / Y * gamma;
                    
                    if y2 < minY
                        minY= y2;
                        association(i)= l;
                    end
                end
                
                % If the minimum value is very large --> new landmark
                if minY > params.T_NN && minY < params.threshold_new_landmark
                    association(i)= 0;
                end
            end
            
            % Increase appearances counter
            for i= 1:n_F
                if association(i) ~= -1 && association(i) ~= 0
                    obj.appearances(association(i))= obj.appearances(association(i)) + 1;
                end
            end
        end
        % ----------------------------------------------
        % ----------------------------------------------
        function association= nearest_neighbor_localization(obj, z, params)
            
            n_F= size(z,1);
            n_L= obj.num_landmarks;
            
            % initialize with zero, if SLAM --> initialize with (-1)
            association= zeros(1,n_F);
            
            if n_F == 0 || n_L == 0, return, end
            
            spsi= sin(obj.XX(9));
            cpsi= cos(obj.XX(9));
            zHat= zeros(2,1);
            % Loop over extracted features
            for i= 1:n_F
                minY= params.T_NN;
                
                for l= 1:n_L
                    landmark= obj.landmark_map(l,:);
                    
                    dx= landmark(1) - obj.XX(1);
                    dy= landmark(2) - obj.XX(2);
                    
                    zHat(1)=  dx*cpsi + dy*spsi;
                    zHat(2)= -dx*spsi + dy*cpsi;
                    gamma= z(i,:)' - zHat;
                    
                    H= [-cpsi, -spsi, -dx*spsi + dy*cpsi;
                        spsi, -cpsi, -dx*cpsi - dy*spsi ];
                    
                    Y= H * obj.PX([1:2,9],[1:2,9]) * H' + params.R_lidar;
                    
                    y2= gamma' / Y * gamma;
                    
                    if y2 < minY
                        minY= y2;
                        association(i)= l;
                    end
                end
                
                % If the minimum value is very large --> ignore
                if minY >= params.T_NN
                    association(i)= 0;
                else % Increase appearances counter
                    obj.appearances(association(i))= obj.appearances(association(i)) + 1;
                end
            end
        end
        % ----------------------------------------------
        % ----------------------------------------------
        function lidar_update_slam(obj, z, association, params)
            
            R= params.R_lidar;
            
            
            obj.XX(9)= pi_to_pi( obj.XX(9) );
            
            if all(association == -1), return; end
            
            % Eliminate the non-associated features
            z(association == -1 | association == 0,:)= [];
            association(association == -1 | association == 0) = [];
            
            lenz= length(association);
            lenx= length(obj.XX);
            
            
            R= kron( R,eye(lenz) );
            H= zeros(2*lenz,lenx);
            
            %Build Jacobian H
            spsi= sin(obj.XX(9));
            cpsi= cos(obj.XX(9));
            zHat= zeros(2*lenz,1);
            for i= 1:length(association)
                % Indexes
                indz= 2*i + (-1:0);
                indx= 15 + 2*association(i) + (-1:0);
                
                dx= obj.XX(indx(1)) - obj.XX(1);
                dy= obj.XX(indx(2)) - obj.XX(2);
                
                % Predicted measurement
                zHat(indz)= [dx*cpsi + dy*spsi;
                    -dx*spsi + dy*cpsi];
                
                % Jacobian -- H
                H(indz,1)= [-cpsi; spsi];
                H(indz,2)= [-spsi; -cpsi];
                H(indz,9)= [-dx * spsi + dy * cpsi;
                    -dx * cpsi - dy * spsi];
                H(indz,indx)= [cpsi, spsi;
                    -spsi, cpsi];
                
            end
            
            % Update
            Y= H*obj.PX*H' + R;
            L= obj.PX * H' / Y;
            zVector= z'; zVector= zVector(:);
            innov= zVector - zHat;
            
            % If it is calibrating, update only landmarks
            if params.SWITCH_CALIBRATION
                XX0= obj.XX(1:15);
                PX0= obj.PX(1:15,1:15);
                obj.XX= obj.XX + L*innov;
                obj.PX= obj.PX - L*H*obj.PX;
                obj.XX(1:15)= XX0;
                obj.PX(1:15,1:15)= PX0;
            else
                obj.XX= obj.XX + L*innov;
                obj.PX= obj.PX - L*H*obj.PX;
            end
        end
        % ----------------------------------------------
        % ----------------------------------------------     
        function lidar_update_localization(obj, z, association, params)
            
            R= params.R_lidar; % TODO: move to the kron
            
            obj.XX(9)= pi_to_pi( obj.XX(9) );
            
            if all(association == 0), return; end
            
            % Eliminate the non-associated features
            z(association == 0, :)= [];
            association(association == 0) = [];
            
            obj.n_k= length(association) * params.m_F;
            lenx= length(obj.XX);
            
            R= kron( R, eye( obj.n_k / params.m_F ) );
            obj.H_k= zeros(obj.n_k, lenx);
            
            %Build Jacobian H
            spsi= sin(obj.XX(9));
            cpsi= cos(obj.XX(9));
            zHat= zeros(obj.n_k,1);
            for i= 1:length(association)
                % Indexes
                indz= 2*i + (-1:0);
                
                dx= obj.landmark_map(association(i), 1) - obj.XX(1);
                dy= obj.landmark_map(association(i), 2) - obj.XX(2);
                
                % Predicted measurement
                zHat(indz)= [dx*cpsi + dy*spsi;
                            -dx*spsi + dy*cpsi];
                
                % Jacobian -- H
                obj.H_k(indz,1)= [-cpsi; spsi];
                obj.H_k(indz,2)= [-spsi; -cpsi];
                obj.H_k(indz,9)= [-dx * spsi + dy * cpsi;
                            -dx * cpsi - dy * spsi];
            end
            
            % Update
            obj.Y_k= obj.H_k * obj.PX * obj.H_k' + R;
            obj.L_k= obj.PX * obj.H_k' / obj.Y_k;
            zVector= z'; zVector= zVector(:);
            obj.gamma_k= zVector - zHat;
            obj.XX= obj.XX + obj.L_k * obj.gamma_k;
            obj.PX= obj.PX - obj.L_k * obj.H_k * obj.PX;
        end
        % ----------------------------------------------
        % ----------------------------------------------     
        function increase_landmarks_cov(obj, minPXLM)
            
            if length(obj.PX) == 15, return, end
            
            PXLM= diag( obj.PX(16:end,16:end) );
            
            minPXLM= minPXLM * ones(length(PXLM),1);
            
            newDiagLM= max(PXLM,minPXLM);
            
            diffDiagLM= PXLM - newDiagLM;
            
            obj.PX(16:end,16:end)= obj.PX(16:end,16:end) - diag( diffDiagLM );
            
        end
        % ----------------------------------------------
        % ----------------------------------------------
        function addNewLM(obj, z, R)
            
            % Number of landmarks to add
            n_L= size(z,1);
            
            % Add new landmarks to state vector
            z= body2nav(z,obj.XX(1:9));
            zVector= z'; zVector= zVector(:);
            obj.XX(end+1:end+2*n_L)= zVector;
            
            spsi= sin(obj.XX(9));
            cpsi= cos(obj.XX(9));
            for i= 1:n_L
                ind= (15 + (2*i-1)):(15 + 2*i);
                
                dx= obj.XX(ind(1)) - obj.XX(1);
                dy= obj.XX(ind(2)) - obj.XX(2);
                
                H= [-cpsi, -spsi, -dx*spsi + dy*cpsi;
                    spsi,  -cpsi, -dx*cpsi - dy*spsi];
                
                Y= H * obj.PX([1:2,9],[1:2,9]) * H' + R;
                
                obj.PX(end+1:end+2, end+1:end+2)= Y;
            end         
        end
        % ----------------------------------------------
        % ----------------------------------------------
        function linearize_discretize(obj, u, dT, params)
            % updates Phi_k & D_bar
            
            if params.SWITCH_CALIBRATION
                taua= params.taua_calibration;
                tauw= params.tauw_calibration;
                S= params.S_cal;
            else
                taua= params.taua_normal_operation;
                tauw= params.tauw_normal_operation;
                S= params.S;
            end
            
            % Compute the F and G matrices (linear continuous time)
            [F,G]= FG_fn(u(1),u(2),u(3),u(5),u(6),...
                obj.XX(7),obj.XX(8),obj.XX(9),obj.XX(10),obj.XX(11),obj.XX(12),obj.XX(14),obj.XX(15),...
                taua,tauw);
            
            % Discretize system for IMU time (only for variance calculations)
            obj.discretize(F, G, S, dT);
        end
        % ----------------------------------------------
        % ----------------------------------------------
        function discretize(obj, F, G, S, dT)
            % MATRICES2DISCRETE This function discretize the continuous time model. It
            % works for either the GPS or IMU discretization times.
            % updates Phi_k & D_bar
            
            % sysc= ss(F, zeros(15,1), zeros(1,15), 0);
            % sysd= c2d(sysc, dT);
            % Phi_k= sysd.A;
            
            % Methdo to obtain covariance matrix for dicrete system
            C= [-F, G*S*G';
                zeros(15), F'];
            
            % Proper method
            EXP= expm(C*dT);
            obj.Phi_k= EXP(16:end,16:end)';
%             obj.D_bar= Phi_k * EXP(1:15,16:end);
            
            % Simplified method
            obj.D_bar= (G*dT) * (S/dT) * (G*dT)'; % simplified version
        end
    end
    % ----------------------------------------------
    % ----------------------------------------------
end



