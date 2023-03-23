% computing third order
clc
clearvars;

z = sym('z',[2 1]); assume(z, 'real');
zd = sym('zd',[2 1]); assume(zd, 'real');
u = sym('u',[1 1]); assume(u, 'real');
delta = sym('delta', 'real');

f = [z(2);-z(1) - z(2) + z(1)*z(2)];

g = [0;1];
 
F = f + g*u;

%%--------- Single rate sampling ------------%%
p   = 3;
z_1 = z;
F_sd = z_1; 
f1_tmp = F_sd; 

for i = 1:p
    j1 = jacobian(f1_tmp, z_1)*F;
    f1_tmp = j1;
    F_sd = simplify(F_sd + delta^i/factorial(i)*f1_tmp);
end

T = [1 -delta/2;0 1];
F_tilde = subs(F_sd, T*z, zd);
% F_tilde = simplify(subs(F_tilde, z(1), zd(1)+delta/2*zd(2)));
% F_tilde = simplify(subs(F_tilde, z(2), zd(2)));


