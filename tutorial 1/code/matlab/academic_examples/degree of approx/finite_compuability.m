% example sympolic computations

clc;
clearvars;

z = sym('z',[3 1]); assume(z, 'real');
v = sym('v',[2 1]); assume(v, 'real');
delta = sym('delta', 'real');

f = [0;0;0];

g = [1 0;
     0 1;
     z(2)  0];
 
F = f + g*v;



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
