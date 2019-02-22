
function test()
options = optimoptions('fminunc','Algorithm','trust-region',...
    'SpecifyObjectiveGradient',true,'HessianFcn','objective');

options = optimoptions('fminunc','Algorithm','trust-region',...
    'SpecifyObjectiveGradient',true);


[x fval] = fminunc(@rosenboth,[-1;2],options)

end

function [f, g, H] = rosenboth(x)
% Calculate objective f
f = 100*(x(2) - x(1)^2)^2 + (1-x(1))^2;

if nargout > 1 % gradient required
    g = [-400*(x(2)-x(1)^2)*x(1)-2*(1-x(1));
        200*(x(2)-x(1)^2)];
    
%     if nargout > 2 % Hessian required
%         H = [1200*x(1)^2-400*x(2)+2, -400*x(1);
%             -400*x(1), 200];  
%     end

end
end