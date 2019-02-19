function compute_steering(obj, params)

% determine if current waypoint reached
while 1
    current_way_point= params.way_points(:, obj.index_current_way_point);
    d2= (current_way_point(1)-obj.x_true(1))^2 + (current_way_point(2)-obj.x_true(2))^2;
    
    % check current distance to the waypoint
    if d2 < params.min_distance_to_way_point^2
        obj.index_current_way_point= obj.index_current_way_point+1; % switch to next
        % reached final waypoint ---> flag and return
        if obj.index_current_way_point > size( params.way_points, 2 )
            obj.goal_is_reached= 1;
            return
        end
        current_way_point= params.way_points( :, obj.index_current_way_point ); % next waypoint
    else
        break;
    end
end

% compute change in G to point towards current waypoint
delta_steering= pi_to_pi( atan2( current_way_point(2) - obj.x_true(2), current_way_point(1) - obj.x_true(1) )...
                         - obj.x_true(3) - obj.steering_angle );

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




