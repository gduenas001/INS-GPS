function compute_steering(obj, params)

if params.SWITCH_OFFLINE
    xx= obj.x_true;
else
    xx= obj.XX;
end


% determine if current waypoint reached
while 1
    current_wp= params.way_points(:, obj.current_wp_ind);
    d2= (current_wp(1)-xx(1))^2 + (current_wp(2)-xx(2))^2;
    
    % check current distance to the waypoint
    if d2 < params.min_distance_to_way_point^2
        obj.current_wp_ind= obj.current_wp_ind+1; % switch to next
        % reached final waypoint ---> flag and return
        if obj.current_wp_ind > size( params.way_points, 2 )
            obj.goal_is_reached= 1;
            return
        end
        current_wp= params.way_points( :, obj.current_wp_ind ); % next waypoint
    else
        break;
    end
end

% compute change in G to point towards current waypoint
delta_steering= pi_to_pi( atan2( current_wp(2) - xx(2), current_wp(1) - xx(1) )...
                         - xx(3) - obj.steering_angle );

% limit rate
max_delta= params.max_delta_steering * params.dt_sim;
if abs(delta_steering) > max_delta
    delta_steering= sign( delta_steering ) * max_delta;
end

% limit angle
obj.steering_angle= obj.steering_angle + delta_steering;
if abs(obj.steering_angle) > params.max_steering
    obj.steering_angle= sign( obj.steering_angle ) * params.max_steering;
end




