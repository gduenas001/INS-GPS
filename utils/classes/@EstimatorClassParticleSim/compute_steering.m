function compute_steering(obj, params)

% determine if current waypoint reached
while 1
    current_wp= params.way_points(:, obj.current_wp_ind);
    d= sqrt( ( current_wp(1)-  obj.XX_predict(1) )^2 + ( current_wp(2) - obj.XX_predict(2) )^2 );
    
    % check current distance to the waypoint
    if d < params.min_distance_to_way_point
        obj.current_wp_ind= obj.current_wp_ind + 1; % next way point
        
        % reached final waypoint ---> flag and return
        if obj.current_wp_ind > size( params.way_points, 2 )
            obj.goal_is_reached= 1;
            return
        end
    else
        break;
    end
end

% compute change in G to point towards current waypoint
delta_steering= pi_to_pi(atan2( current_wp(2) - obj.XX_predict(2), current_wp(1) - obj.XX_predict(1) ) - obj.XX_predict(3));
delta_steering= pi_to_pi(delta_steering - obj.steering_angle);

% % weighting factor for the delta steering
% d_min= 0;
% d_max= 100;
% weight_min= 1;
% weight_max= 100;
% slope= (weight_max - weight_min) / (d_max - d_min);
% if d <= d_min
%     weight= weight_min;
% elseif d > d_min && d < d_max
%     weight= weight_min + slope * (d - d_min);
% elseif d >= d_max
%     weight= weight_max;
% end
% delta_steering= delta_steering / weight;

% limit rate
max_delta= params.max_delta_steering * params.dt_sim;
if abs(delta_steering) > max_delta
    delta_steering= sign( delta_steering ) * max_delta;
end

% limit angle
obj.steering_angle= pi_to_pi( obj.steering_angle + delta_steering );
if abs(obj.steering_angle) > params.max_steering
    obj.steering_angle= sign( obj.steering_angle ) * params.max_steering;
end




