
clc;
clearvars;

Np=6;

Ts=0.5;
% syms Ts real;

A     = [0 1 0 ;0 0 1;0 0 0];B = [0;0;1]; C = [1 0 0];
sys   = ss(A,B,C,[]);
sys_d = c2d(sys,Ts);
Ad    = sys_d.A;
Bd    = sys_d.B;
Cd    = sys_d.C;

% Ad = [1 Ts Ts^2/2; 0 1 Ts;0 0 1];
% Bd = [Ts^3/6;Ts^2/2;Ts];
% Cd = [1 0 0];
plotoptions = pzoptions;
plotoptions.Grid = 'on';
p = [];
i = [];
figure
for Nc = Np:-1:Np-3
    [p_new, Acl_new] = eig_closedloop(Ad,Bd,Cd,Np,Nc);
    p = [p p_new];
    i = [i Nc];
    sys = ss(double(Acl_new),Bd,Cd,[],Ts);
    subplot(2,2,Np-Nc+1)
    zplane(tzero(sys),pole(sys));
end




