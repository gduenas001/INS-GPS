function angle = pi_to_pi(angle)

%function angle = pi_to_pi(angle)


if angle < -2*pi || angle > 2*pi
%    warning('pi_to_pi() error: angle outside 2-PI bounds.')
    angle= mod(angle, 2*pi);
end

if angle > pi
    angle= angle - 2*pi;
elseif angle < -pi
    angle= angle + 2*pi;
end