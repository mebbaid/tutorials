% example sympolic computations

clc;
clearvars;

x = sym('x',[3 1]); assume(x, 'real');
u = sym('u',[2 1]); assume(u, 'real');
delta = sym('delta', 'real');

f = [0;0;0];

g = [cos(x(3)) 0;
     sin(x(3)) 0;
     0         1];
 
F = f + g*u;



%%--------- Single rate sampling ------------%%
p   = 2;
z_1 = x;
F_sd = z_1; 
f1_tmp = F_sd; 

for i = 1:p
    j  = jacobian(f1_tmp, z_1)*F;
    f1_tmp = j;
    F_sd = simplify(F_sd + delta^i/factorial(i)*f1_tmp);
end


%%--------- Multi rate sampling ------------%%
u11 = sym('u11', 'real'); 
u21 = sym('u21', 'real'); 
u22 = sym('u22', 'real'); 

u_1 = [u11; u21]; 
u_2 = [u11; u22]; 
u_mr = [u_1 u_2]; 

F_MR = z_1; 

for i = 1:2
    F_MR = simplify(subs(F_sd, z_1, F_MR));
    F_MR = simplify(subs(F_MR, u, u_mr(:, i)));
end
