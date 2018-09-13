function angle = pi_to_pi(angle)

%function angle = pi_to_pi(angle)

if length(angle) == 1
if angle < -2*pi || angle > 2*pi
%    warning('pi_to_pi() error: angle outside 2-PI bounds.')
    angle= mod(angle, 2*pi);
end

if angle > pi
    angle= angle - 2*pi;
elseif angle < -pi
    angle= angle + 2*pi;
end

else
i= find(angle<-2*pi | angle>2*pi); % replace with a check
if ~isempty(i) 
%    warning('pi_to_pi() error: angle outside 2-PI bounds.')
    angle(i) = mod(angle(i), 2*pi);
end

i= find(angle>pi);
angle(i)= angle(i)-2*pi;

i= find(angle<-pi);
angle(i)= angle(i)+2*pi;
end