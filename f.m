% Function f(x) in dynamics. This case is particular to the forced Van der
% Pol oscillator.
function fx = f(x)
    
    mu = 0.2;

    fx = [x(2); mu*(1-x(1)^2)*x(2)-x(1)];

end