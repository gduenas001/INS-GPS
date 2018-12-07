
classdef EstimatorClass < handle
    properties
        XX= zeros(15,1)
        PX= zeros(15)
        
        initial_attitude
        appearances
        
    end
    
    methods
        % ----------------------------------------------
        % ----------------------------------------------
        function obj= EstimatorClass(imu, params)
            
            % Initial attitude
            obj.initialize_pitch_and_roll(imu, params)
            obj.XX(9)= deg2rad(params.initial_yaw_angle);
            
            % save initial attitude for calibration
            obj.initial_attitude= obj.XX(7:9);
            
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
        function calibration(obj, z, H, R)
            
            % Calibration msmt update
            L= obj.PX(1:15,1:15)*H' / (H*obj.PX(1:15,1:15)*H' + R);
            z_hat= H*obj.XX(1:15);
            innov= z - z_hat;
            innov(end)= pi_to_pi(innov(end));
            obj.XX(1:15)= obj.XX(1:15) + L*innov;
            obj.PX(1:15,1:15)= obj.PX(1:15,1:15) - L*H*obj.PX(1:15,1:15);
        end
        % ----------------------------------------------
        % ----------------------------------------------
        function imu_update( obj, imu_msmt, taua, tauw, params )
            % updates the state with the IMU reading, NO cov update
            
            % Create variables (for clarity)
            v= obj.XX(4:6);
            phi= obj.XX(7); theta= obj.XX(8); psi= obj.XX(9);
            b_f= obj.XX(10:12);
            b_w= obj.XX(13:15);
            f= imu_msmt(1:3);
            w= imu_msmt(4:6);
            
            % Calculate parameters
            R_NB= R_NB_rot (phi,theta,psi);
            Q_BE= Q_BE_fn  (phi,theta);
            
            r_dot= v;
            v_dot= R_NB * ( f - b_f ) + params.g_N;
            E_dot= Q_BE * ( w - b_w );
            b_f_dot= -eye(3) / taua * b_f;
            b_w_dot= -eye(3) / tauw * b_w;
            x_dot= [r_dot; v_dot; E_dot; b_f_dot; b_w_dot];
            
            % Return new pose
            obj.XX(1:15)= obj.XX(1:15) + params.dT_IMU * x_dot;    
        end
        % ----------------------------------------------
        % ----------------------------------------------
        function yaw_update(obj, w, R, r_IMU2rearAxis) % TODO: optimize
            
            n_L= (length(obj.XX) - 15) / 2;
            H= zeros(1, 15 + 2*n_L);
            H(9)= 1;
            
            z= obj.yawMeasurement(w, r_IMU2rearAxis);
            
            L= obj.PX*H' / (H*obj.PX*H' + R);
            innov= z - H*obj.XX;
            innov= pi_to_pi(innov);
            obj.XX(9)= pi_to_pi(obj.XX(9));
            obj.XX= obj.XX + L*innov;
            obj.PX= obj.PX - L*H*obj.PX;
        end
        % ----------------------------------------------
        % ----------------------------------------------
        function yaw= yawMeasurement(obj, w, r_IMU2rearAxis) % TODO: optimize
            
            r= [-r_IMU2rearAxis;0;0];
            v_o= obj.XX(4:6);
            R_NB= R_NB_rot( obj.XX(7), obj.XX(8), obj.XX(9));
            
            v_a= v_o + R_NB * cross(w,r);
            v_a= v_a / norm(v_a);
            
            yaw= atan2(v_a(2),v_a(1));
        end
        % ----------------------------------------------
        % ----------------------------------------------
        function gps_update(obj, z, R, minVelocityGPS, flag_velocity)
            
           
            n_L= (length(obj.XX) - 15) / 2;
            
            if norm(z(4:6)) > minVelocityGPS && flag_velocity % sense velocity
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
        function [association]= nearest_neighbor(obj, z, R, T, T_newLM)
            
            n_F= size(z,1);
            n_L= (length(obj.XX) - 15) / 2;
            
            association= ones(1,n_F) * (-1);
            
            if n_F == 0 || n_L == 0, return, end
            
            
            spsi= sin(obj.XX(9));
            cpsi= cos(obj.XX(9));
            zHat= zeros(2,1);
            % Loop over extracted features
            for i= 1:n_F
                minY= T_newLM;
                
                for l= 1:n_L
                    ind= (15 + (2*l-1)):(15 + 2*l);
                    
                    dx= obj.XX(ind(1)) - obj.XX(1);
                    dy= obj.XX(ind(2)) - obj.XX(2);
                    
                    zHat(1)=  dx*cpsi + dy*spsi;
                    zHat(2)= -dx*spsi + dy*cpsi;
                    gamma= z(i,:)' - zHat;
                    
                    H= [-cpsi, -spsi, -dx*spsi + dy*cpsi,  cpsi, spsi;
                        spsi, -cpsi, -dx*cpsi - dy*spsi, -spsi, cpsi];
                    
                    Y= H * obj.PX([1:2,9,ind],[1:2,9,ind]) * H' + R;
                    
                    y2= gamma' / Y * gamma;
                    
                    if y2 < minY
                        minY= y2;
                        association(i)= l;
                    end
                end
                
                % If the minimum value is very large --> new landmark
                if minY > T && minY < T_newLM
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
        function lidarUpdate(obj, z, association, R, SWITCH_CALIBRATION)
            
            obj.XX(9)= pi_to_pi( obj.XX(9) );
            
            if all(association == -1), return; end
            
            % Eliminate the non-associated features
            z(association == -1 | association == 0,:)= [];
            association(association == -1 | association == 0) = [];
            
            lenz= length(association);
            lenx= length(obj.XX);
            
            R= kron(R,eye(lenz));
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
            if SWITCH_CALIBRATION
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
        function [Phi,D_bar]= linearize_discretize(obj, u, S, taua, tauw, dT)
            
            % Compute the F and G matrices (linear continuous time)
            [F,G]= FG_fn(u(1),u(2),u(3),u(5),u(6),...
                obj.XX(7),obj.XX(8),obj.XX(9),obj.XX(10),obj.XX(11),obj.XX(12),obj.XX(14),obj.XX(15),...
                taua,tauw);
            
            % Discretize system for IMU time (only for variance calculations)
            [Phi,D_bar]= obj.discretize(F, G, S, dT);
        end
        % ----------------------------------------------
        % ----------------------------------------------
        function [Phi, D_bar]= discretize(~, F, G, S, dT)
            %MATRICES2DISCRETE This function discretize the continuous time model. It
            %works for either the GPS or IMU discretization times.
            
            
            % sysc= ss(F, zeros(15,1), zeros(1,15), 0);
            % sysd= c2d(sysc, dT);
            % Phi= sysd.A;
            
            % Methdo to obtain covariance matrix for dicrete system
            C= [-F, G*S*G';
                zeros(15), F'];
            
            % Proper method
            EXP= expm(C*dT);
            Phi= EXP(16:end,16:end)';
%             D_bar= Phi * EXP(1:15,16:end);
            
            
            % Simplified method
            D_bar= (G*dT) * (S/dT) * (G*dT)'; % simplified version
        end
    end
    % ----------------------------------------------
    % ----------------------------------------------
end



