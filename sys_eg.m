% Function for system dynamics, calling functions f and g.
function xdot = sys_eg(t,x,u)

    xdot = f(x) + g(x)*u;

end