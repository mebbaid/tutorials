function xpred = predictionModel(u,x, np,delta)
%SATELLITE Summary of this function goes here
%   Detailed explanation goes here
    ct    = u(:,1);
    xpred = sym('xpred',[length(x),np],'real');
    xpred(:,1) = x;    
    for j=2:np  
        f    = [0;0;0];
        g1   = [cos(xpred(3,j-1));sin(xpred(3,j-1));0];
        g2   = [0;0;1];
        xpred(:,j) = xpred(:,j-1) + delta*( f + g1*ct(1) + g2*ct(2));
        ct = u(:,j);
    end 
end


