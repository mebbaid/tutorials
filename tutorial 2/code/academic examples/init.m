% computing a coordinates change

clc;
clearvars;
close all;


simTime    = 3;
simStep    = 10^-4;
Delta      = 0.3;
x0   = [-1;1];


k1   = 1; 
k2   = 1;
A    = [0 1;-1 -1];    b = [0;1];
p1 = -3; p2 = -10;
Gain = place(A,b, [p1 , p2]); % pole placement for ct system
As   = [1 Delta;0 1];  bs = [Delta^2/2; Delta];
Gains = place(As, bs, [exp(p1*Delta) , exp(p2*Delta)]);  % pole placement for dt system

emulationFlag = 1;
emulationRedesignFlag = 1;
SingleRateFlag = 1;
refType = 0;

out =  sim('motivation.slx','StartTime','0','StopTime',num2str(simTime),'FixedStep',num2str(simStep));
xc  = out.x.Data;
yc  = out.y.Data;
uc  = out.u.Data;
vc  = out.v.Data;
ref = out.r.Data;




tc = 0:simStep:simTime;





%------------------------ plots------------------------
figure('Name','Continuous time')
subplot(2,1,1)
plot(tc, xc(1,:),  'LineWidth', 2);
hold on; grid on;
plot(tc, yc,  'LineWidth', 2);
plot(tc, ref, '--'  ,'LineWidth', 2);
plot(tc, xc(2,:),  'LineWidth', 2);
l = legend('$x_1$', '$y$', '$y_d$' , '$x_2$');
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
    plot(tc, out.xe.Data(1,:),  'LineWidth', 2);
    hold on; grid on;
    plot(tc, out.ye.Data,  'LineWidth', 2);
    plot(tc, ref, '--'  ,'LineWidth', 2);
    plot(tc, out.xe.Data(2,:),  'LineWidth', 2);
    l = legend('$x_1$', '$y$','$y_d$' , '$x_2$');
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


if emulationRedesignFlag ~= 0
    figure('Name','Emulation Redesign')
    subplot(2,1,1)
    plot(tc, out.xe1.Data(1,:),  'LineWidth', 2);
    hold on; grid on;
    plot(tc, out.ye1.Data,  'LineWidth', 2);
    plot(tc, ref, '--'  ,'LineWidth', 2);
    plot(tc, out.xe1.Data(2,:),  'LineWidth', 2);
    l = legend('$x_1$', '$y$','$y_d$' , '$x_2$');
    set(l,'Interpreter','Latex');
    l = xlabel('Time (s)'); 
    set(l,'Interpreter','Latex');
    l = ylabel('States'); 
    set(l,'Interpreter','Latex');
    l.FontSize = 30;
    
    subplot(2,1,2)
    plot(tc, out.ue1.Data,  'LineWidth', 2);
    hold on; grid on;
    plot(tc, out.ve1.Data,  'LineWidth', 2);
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
    plot(tc, out.xs.Data(1,:),  'LineWidth', 2);
    hold on; grid on;
    plot(tc, out.ys.Data,  'LineWidth', 2);
    plot(tc, ref, '--'  ,'LineWidth', 2);
    plot(tc, out.xs.Data(2,:),  'LineWidth', 2);
    l = legend('$x_1$','$y$', '$y_d$' , '$x_2$');
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
