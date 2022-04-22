

clc;
clearvars; 
close all;


delta = 0.1;
simTime = 3;
x0    = [0;0;0];



out = sim('sampling_example');

xc  = out.xc;
p1  = out.xs1;
p2  = out.xs2;
p5  = out.xs5;
ex  = out.xexact;


t   = 0:10^-3:simTime;
ts  = 0:delta:simTime;

figure('Name','Comparisons of single_rate models')
subplot(3,2,1)
plot(t, xc(:,1), 'k', 'LineWidth', 1.5);
hold on; grid on;
plot(t, p1(:,1),  'LineWidth', 1.5);
plot(t, p2(:,1),  'LineWidth', 1.5);
plot(t, p5(:,1),  'LineWidth', 1.5);
l = legend('Continous-time plant', 'Approx $p = 1$', 'Approx $p = 2$', 'Approx $p = 5$');
set(l,'Interpreter','Latex');
l = xlabel('Time (s)'); 
set(l,'Interpreter','Latex');
l = ylabel('$x(t)$'); 
set(l,'Interpreter','Latex');
l.FontSize = 30;

subplot(3,2,2)
hold on; grid on;
plot(t, xc(:,1) - p1(:,1),  'LineWidth', 1.5);
plot(t, xc(:,1) - p2(:,1),  'LineWidth', 1.5);
plot(t, xc(:,1) - p5(:,1),  'LineWidth', 1.5);
l = legend('Approx $p = 1$', 'Approx $p = 2$', 'Approx $p = 5$');
set(l,'Interpreter','Latex');
l = xlabel('Time (s)'); 
set(l,'Interpreter','Latex');
l = ylabel('Error on $x(t)$'); 
set(l,'Interpreter','Latex');
l.FontSize = 30;

subplot(3,2,3)
plot(t, xc(:,2), 'k', 'LineWidth', 1.5);
hold on; grid on;
plot(t, p1(:,2),  'LineWidth', 1.5);
plot(t, p2(:,2),  'LineWidth', 1.5);
plot(t, p5(:,2),  'LineWidth', 1.5);
l = legend('Continous-time plant', 'Approx $p = 1$', 'Approx $p = 2$', 'Approx $p = 5$');
set(l,'Interpreter','Latex');
l = xlabel('Time (s)'); 
set(l,'Interpreter','Latex');
l = ylabel('$y(t)$'); 
set(l,'Interpreter','Latex');
l.FontSize = 30;


subplot(3,2,4)
hold on; grid on;
plot(t, xc(:,2) - p1(:,2),  'LineWidth', 1.5);
plot(t, xc(:,2) - p2(:,2),  'LineWidth', 1.5);
plot(t, xc(:,2) - p5(:,2),  'LineWidth', 1.5);
l = legend('Approx $p = 1$', 'Approx $p = 2$', 'Approx $p = 5$');
set(l,'Interpreter','Latex');
l = xlabel('Time (s)'); 
set(l,'Interpreter','Latex');
l = ylabel('Error on $y(t)$'); 
set(l,'Interpreter','Latex');
l.FontSize = 30;

subplot(3,2,5)
plot(t, xc(:,3), 'k', 'LineWidth', 1.5);
hold on; grid on;
plot(t, p1(:,3),  'LineWidth', 1.5);
plot(t, p2(:,3),  'LineWidth', 1.5);
plot(t, p5(:,3),  'LineWidth', 1.5);
l = legend('Continous-time plant', 'Approx $p = 1$', 'Approx $p = 2$', 'Approx $p = 5$');
set(l,'Interpreter','Latex');
l = xlabel('Time (s)'); 
set(l,'Interpreter','Latex');
l = ylabel('$\theta(t)$'); 
set(l,'Interpreter','Latex');
l.FontSize = 30;

subplot(3,2,6)
hold on; grid on;
plot(t, xc(:,3) - p1(:,3),  'LineWidth', 1.5);
plot(t, xc(:,3) - p2(:,3),  'LineWidth', 1.5);
plot(t, xc(:,3) - p5(:,3),  'LineWidth', 1.5);
l = legend('Approx $p = 1$', 'Approx $p = 2$', 'Approx $p = 5$');
set(l,'Interpreter','Latex');
l = xlabel('Time (s)'); 
set(l,'Interpreter','Latex');
l = ylabel('Error on $\theta(t)$'); 
set(l,'Interpreter','Latex');
l.FontSize = 30;


figure('Name','Comparisons of exact and approximate sr models')

subplot(3,2,1)
plot(t, xc(:,1), 'k', 'LineWidth', 1.5);
hold on; grid on;
plot(t, p5(:,1),  'LineWidth', 1.5);
plot(t, ex(:,1),  'LineWidth', 1.5);
l = legend('Continous-time plant', 'Approx $p = 5$', 'Exact SD');
set(l,'Interpreter','Latex');
l = xlabel('Time (s)'); 
set(l,'Interpreter','Latex');
l = ylabel('$x(t)$'); 
set(l,'Interpreter','Latex');
l.FontSize = 30;

subplot(3,2,2)
hold on; grid on;
plot(t, ex(:,1) - p5(:,1),  'LineWidth', 1.5);
l = legend('$x_{p} - x_e $');
set(l,'Interpreter','Latex');
l = xlabel('Time (s)'); 
set(l,'Interpreter','Latex');
l = ylabel('Error on $x(t)$'); 
set(l,'Interpreter','Latex');
l.FontSize = 30;

subplot(3,2,3)
plot(t, xc(:,2), 'k', 'LineWidth', 1.5);
hold on; grid on;
plot(t, p5(:,2),  'LineWidth', 1.5);
plot(t, ex(:,2),  'LineWidth', 1.5);
l = legend('Continous-time plant', 'Approx $p = 5$', 'Exact SD');
set(l,'Interpreter','Latex');
l = xlabel('Time (s)'); 
set(l,'Interpreter','Latex');
l = ylabel('$y(t)$'); 
set(l,'Interpreter','Latex');
l.FontSize = 30;


subplot(3,2,4)
hold on; grid on;
plot(t, ex(:,2) - p5(:,2),  'LineWidth', 1.5);
l = legend('$y_{p} - y_e $');
set(l,'Interpreter','Latex');
l = xlabel('Time (s)'); 
set(l,'Interpreter','Latex');
l = ylabel('Error on $y(t)$'); 
set(l,'Interpreter','Latex');
l.FontSize = 30;

subplot(3,2,5)
plot(t, xc(:,3), 'k', 'LineWidth', 1.5);
hold on; grid on;
plot(t, p5(:,3),  'LineWidth', 1.5);
plot(t, ex(:,3),  'LineWidth', 1.5);
l = legend('Continous-time plant', 'Approx $p = 5$', 'Exact SD');
set(l,'Interpreter','Latex');
l = xlabel('Time (s)'); 
set(l,'Interpreter','Latex');
l = ylabel('$\theta(t)$'); 
set(l,'Interpreter','Latex');
l.FontSize = 30;

subplot(3,2,6)
hold on; grid on;
plot(t, ex(:,3) - p5(:,3),  'LineWidth', 1.5);
l = legend('$\theta_{p} - \theta_e $');
set(l,'Interpreter','Latex');
l = xlabel('Time (s)'); 
set(l,'Interpreter','Latex');
l = ylabel('Error on $\theta(t)$'); 
set(l,'Interpreter','Latex');
l.FontSize = 30;
