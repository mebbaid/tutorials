function st = prediction(x0,uopt, Np, T)
%     import casadi.*
%     st = SX.sym('st', length(x0), Np + 1);
    st = zeros(length(x0), Np+1);
    st(:,1) = x0;
    for k = 1:Np
       kyn_current  = [uopt(1,k)*cos(st(3,k));uopt(1,k)*sin(st(3,k));uopt(2,k)];
       st(:,k+1) = st(:,k) + T*kyn_current;
    end
end

