function dx = plant(t,x,u)
%SATELLITE Summary of this function goes here
%   Detailed explanation goes here
        f    = [0;0;0];
        g1   = [cos(x(3));sin(x(3));0];
        g2   = [0;0;1];
        dx   = f + g1*u(1) + g2*u(2);

end


