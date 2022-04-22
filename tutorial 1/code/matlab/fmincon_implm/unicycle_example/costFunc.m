function J = costFunc(u,x,xr,Q,R,np)
%COSTFUNC Summary of this function goes here
%   Detailed explanation goes here
    J = 0;
    for i=1:np
        J =  J +  (x(:,i)-xr(:,i))'*Q*(x(:,i)-xr(:,i)) + u(:,i)'*R*u(:,i);
    end
end

