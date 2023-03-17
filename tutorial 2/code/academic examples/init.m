% computing a coordinates change

clc;
clearvars;
close all;


simTime    = 10;
simStep    = 10^-4;
Delta      = 0.2;
x0   = [-1;0];


k1   = 1; 
k2   = 1;
A    = [0 1;-1 -1];    b = [0;1];
Gain = place(A,b, [-3 , -10]); % pole placement for ct system
As   = [1 Delta;0 1];  bs = [Delta^2/2; Delta];
Gains = place(As, bs, [0.2 , 0.1]);  % pole placement for dt system

out =  sim('motivation.slx','StartTime','0','StopTime',num2str(simTime),'FixedStep',num2str(simStep));
xc  = out.x.Data;
yc  = out.y.Data;
uc  = out.u.Data;
vc  = out.v.Data;
ref = out.r.Data;




tc = 0:simStep:simTime;


emulationFlag = 0;
SingleRateFlag = 1;


%------------------------ plots------------------------
figure('Name','Continuous time')
subplot(2,1,1)
plot(tc, yc,  'LineWidth', 2);
hold on; grid on;
plot(tc, ref, '--'  ,'LineWidth', 2);
plot(tc, xc(2,:),  'LineWidth', 2);
l = legend('$x_1$', '$y_d$' , '$x_2$');
set(l,'Interpreter','Latex');
l = xlabel('Time (s)'); 
set(l,'Interpreter','Latex');
l = ylabel('States'); 
set(l,'Interpreter','Latex');
l.FontSize = 30;

subplot(2,1,2)
plot(tc, uc,  'LineWidth', 2);
hold on; grid on;
plot(tc, vc,  'LineWidth', 2);
l = legend('$u$', '$v$');
set(l,'Interpreter','Latex');
l = xlabel('Time (s)'); 
set(l,'Interpreter','Latex');
l = ylabel('Controls'); 
set(l,'Interpreter','Latex');
l.FontSize = 30;

if emulationFlag ~= 0
    figure('Name','Emulation')
    subplot(2,1,1)
    plot(tc, out.ye.Data,  'LineWidth', 2);
    hold on; grid on;
    plot(tc, ref, '--'  ,'LineWidth', 2);
    plot(tc, out.xe.Data(2,:),  'LineWidth', 2);
    l = legend('$x_1$', '$y_d$' , '$x_2$');
    set(l,'Interpreter','Latex');
    l = xlabel('Time (s)'); 
    set(l,'Interpreter','Latex');
    l = ylabel('States'); 
    set(l,'Interpreter','Latex');
    l.FontSize = 30;
    
    subplot(2,1,2)
    plot(tc, out.ue.Data,  'LineWidth', 2);
    hold on; grid on;
    plot(tc, out.ve.Data,  'LineWidth', 2);
    l = legend('$u$', '$v$');
    set(l,'Interpreter','Latex');
    l = xlabel('Time (s)'); 
    set(l,'Interpreter','Latex');
    l = ylabel('Controls'); 
    set(l,'Interpreter','Latex');
    l.FontSize = 30;
end


if SingleRateFlag ~=0
    figure('Name','Approximate single rate sampled-data')
    subplot(2,1,1)
    plot(tc, out.ys.Data,  'LineWidth', 2);
    hold on; grid on;
    plot(tc, ref, '--'  ,'LineWidth', 2);
    plot(tc, out.xs.Data(2,:),  'LineWidth', 2);
    l = legend('$x_1$', '$y_d$' , '$x_2$');
    set(l,'Interpreter','Latex');
    l = xlabel('Time (s)'); 
    set(l,'Interpreter','Latex');
    l = ylabel('States'); 
    set(l,'Interpreter','Latex');
    l.FontSize = 30;
    
    subplot(2,1,2)
    plot(tc, out.us.Data,  'LineWidth', 2);
    hold on; grid on;
    plot(tc, out.vs.Data,  'LineWidth', 2);
    l = legend('$u$', '$v$');
    set(l,'Interpreter','Latex');
    l = xlabel('Time (s)'); 
    set(l,'Interpreter','Latex');
    l = ylabel('Controls'); 
    set(l,'Interpreter','Latex');
    l.FontSize = 30;
end
