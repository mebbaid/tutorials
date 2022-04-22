function z = ChainedForm_mr(x, u, Ts)
delta = Ts;
A    = [1 0; 2*delta*u(1) 1];
R    = [delta delta; 1.5*delta^2*u(1) 0.5*delta^2*u(1)];
z1   = x(1) + 2*delta*u(1);
z2   = A*[x(2); x(3)] + R*[u(2);u(3)];
z    = [z1;z2];
end

