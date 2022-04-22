clc;
close all;


figure('Name','Comparison with multi-rate deadbeat')
subplot(3,2,1);
l = title('Movemment on the plane');
set(l,'Interpreter','Latex');
CarPlots(x_mpc,ympc,thetamr); 
l = xlabel('x'); 
set(l,'Interpreter','Latex');
l = ylabel('y'); 
set(l,'Interpreter','Latex');
% 
% l = title('');
% set(l,'Interpreter','Latex');
% hold on; grid on;
% plot(t, u1, 'r', 'LineWidth', 2);
% plot(t, u2, 'b', 'LineWidth', 2);
% l = xlabel('Time (s)'); 
% set(l,'Interpreter','Latex');
% l = legend('$u_1(t)$   MR-MPC','$u_2(t)$ MR-MPC');
% set(l,'Interpreter','Latex');
% 
% 
subplot(3,2,2);
%  
l = title('Movemment on the plane');
set(l,'Interpreter','Latex');
CarPlots(xmr,ymr,thetamr); 
l = xlabel('x'); 
set(l,'Interpreter','Latex');
l = ylabel('y'); 
set(l,'Interpreter','Latex');
% 
% set(l,'Interpreter','Latex');
% hold on; grid on;
% plot(t, u1m, 'r--', 'LineWidth', 2);
% plot(t, u2m, 'b--', 'LineWidth', 2);
% l = xlabel('Time (s)'); 
% set(l,'Interpreter','Latex');
% l = legend('$u_1(t)$- MR','$u_2(t)$- MR');
% set(l,'Interpreter','Latex');
% 
% 
subplot(3,2,3);
l = title('States trajectory MPC');
set(l,'Interpreter','Latex');
mpc=plot(t, xmpc(:,1), 'r', 'LineWidth', 2);
hold on; grid on;
plot(t, xmpc(:,2), 'b', 'LineWidth', 2);
plot(t, xm(:,3), 'g', 'LineWidth', 2);
% mr=plot(t, xm(:,1), 'r--', 'LineWidth', 2);
% plot(t, xm(:,2), 'b--', 'LineWidth', 2);
% plot(t, xm(:,3), 'g--', 'LineWidth', 2);
% ref = plot(tref, xref, 'ro', 'LineWidth', 1);
% plot(tref, yref, 'bo', 'LineWidth', 1);
% plot(tref, thetaref, 'go', 'LineWidth', 1);
% l = legend ([mpc mr ref],'MR MPC','multi-rate', 'reference');
% set(l,'Interpreter','Latex');
% l = xlabel('Time (s)'); 
% set(l,'Interpreter','Latex');

subplot(3,2,4);
l = title('States trajectory MPC');
set(l,'Interpreter','Latex');
hold on; grid on;
mr=plot(t, xm(:,1), 'r--', 'LineWidth', 2);
plot(t, xm(:,2), 'b--', 'LineWidth', 2);
plot(t, xm(:,3), 'g--', 'LineWidth', 2);
ref = plot(tref, xref, 'ro', 'LineWidth', 1);
% mpc=plot(t, xmpc(:,1), 'r', 'LineWidth', 2);
% hold on; grid on;
% plot(t, xmpc(:,2), 'b', 'LineWidth', 2);
% plot(t, xmpc(:,3), 'g', 'LineWidth', 2);
l = xlabel('Time (s)'); 
set(l,'Interpreter','Latex');


subplot(3,2,5);
set(l,'Interpreter','Latex');
hold on; grid on;
plot(t, u1, 'r', 'LineWidth', 2);
plot(t, u2m, 'b', 'LineWidth', 2);
% % plot(t, u1m, 'r--', 'LineWidth', 2);
% % plot(t, u2m, 'b--', 'LineWidth', 2);
% % 
l = xlabel('Time (s)'); 
set(l,'Interpreter','Latex');
% % l = legend('$u_1(t)$   MR-MPC','$u_2(t)$ MR-MPC', '$u_1(t)$- MR','$u_2(t)$- MR');
% % set(l,'Interpreter','Latex');
% 
% l = title('States trajectory MR');
% set(l,'Interpreter','Latex');
% plot(t, xm(:,1), 'r--', 'LineWidth', 2);
% hold on; grid on;
% plot(t, xm(:,2), 'b--', 'LineWidth', 2);
% plot(t, xm(:,3), 'g--', 'LineWidth', 2);
% hold on; grid on;
% plot(tref, xref, 'r*', 'LineWidth', 1);
% plot(tref, yref, 'b*', 'LineWidth', 1);
% plot(tref, thetaref, 'g*', 'LineWidth', 1);
% l = legend ('MR $x(t)$ ', 'MR $y(t)$', 'MR $\theta(t)$' );
% set(l,'Interpreter','Latex');
% l = xlabel('Time (s)'); 
% set(l,'Interpreter','Latex');
% 
% 

subplot(3,2,6);
set(l,'Interpreter','Latex');
hold on; grid on;
% plot(t, u1, 'r', 'LineWidth', 2);
% plot(t, u2, 'b', 'LineWidth', 2);
plot(t, u1m, 'r--', 'LineWidth', 2);
plot(t, u2m, 'b--', 'LineWidth', 2);
% % 
l = xlabel('Time (s)'); 
set(l,'Interpreter','Latex');


 second comparison

figure('Name','Comparison with standard MPC')

subplot(3,2,1);
CarPlots(x_mpc,ympc,thetampc); 
l = xlabel('x'); 
set(l,'Interpreter','Latex');
l = ylabel('y'); 
set(l,'Interpreter','Latex');



subplot(3,2,2);  
CarPlots(xsr,ysr,thetasr); 
l = xlabel('x'); 
set(l,'Interpreter','Latex');
l = ylabel('y'); 
set(l,'Interpreter','Latex');


subplot(3,2,3);
mpc=plot(t, xmpc(:,1), 'r', 'LineWidth', 2);
hold on; grid on;
plot(t, xmpc(:,2), 'b', 'LineWidth', 2);
plot(t, xmpc(:,3), 'g', 'LineWidth', 2);



subplot(3,2,4);
hold on; grid on;
mr=plot(t, xs(:,1), 'r--', 'LineWidth', 2);
plot(t, xs(:,2), 'b--', 'LineWidth', 2);
plot(t, xs(:,3), 'g--', 'LineWidth', 2);
ref = plot(tref, xref, 'ro', 'LineWidth', 1);

subplot(3,2,5);
set(l,'Interpreter','Latex');
hold on; grid on;
plot(t, u1, 'r', 'LineWidth', 2);
plot(t, u2, 'b', 'LineWidth', 2); 
l = xlabel('Time (s)'); 
set(l,'Interpreter','Latex');


subplot(3,2,6);
set(l,'Interpreter','Latex');
hold on; grid on;
plot(t, u1s, 'r--', 'LineWidth', 2);
plot(t, u2s, 'b--', 'LineWidth', 2);
l = xlabel('Time (s)'); 
set(l,'Interpreter','Latex');



% subplot(2,2,1);
% % l = title('Movemment on the plane');
% % set(l,'Interpreter','Latex');
% % CarPlots(x_mpc,ympc,thetampc); 
% % l = xlabel('x'); 
% % set(l,'Interpreter','Latex');
% % l = ylabel('y'); 
% % set(l,'Interpreter','Latex');
% 
% l = title('');
% set(l,'Interpreter','Latex');
% hold on; grid on;
% plot(t, u1, 'r', 'LineWidth', 2);
% plot(t, u2, 'b', 'LineWidth', 2);
% l = xlabel('Time (s)'); 
% set(l,'Interpreter','Latex');
% l = legend('$u_1(t)$   MR-MPC','$u_2(t)$ MR-MPC');
% set(l,'Interpreter','Latex');
% 
% 
%  subplot(2,2,2);
% % l = title('Movemment on the plane');
% % set(l,'Interpreter','Latex');
% % CarPlots(xsr,ysr,thetasr); 
% % l = xlabel('x'); 
% % set(l,'Interpreter','Latex');
% % l = ylabel('y'); 
% % set(l,'Interpreter','Latex');
% 
% l = title('');
% set(l,'Interpreter','Latex');
% hold on; grid on;
% plot(t, u1s, 'r--', 'LineWidth', 2);
% plot(t, u2s, 'b--', 'LineWidth', 2);
% l = xlabel('Time (s)'); 
% set(l,'Interpreter','Latex');
% l = legend('$u_1(t)$ SR-MPC','$u_2(t)$ SR-MPC');
% set(l,'Interpreter','Latex');
% 
% 
% subplot(2,2,3);
% l = title('States trajectory MPC');
% set(l,'Interpreter','Latex');
% plot(t, xmpc(:,1), 'r', 'LineWidth', 2);
% hold on; grid on;
% plot(t, xmpc(:,2), 'b', 'LineWidth', 2);
% plot(t, xmpc(:,3), 'g', 'LineWidth', 2);
% hold on; grid on;
% plot(tref, xref, 'ro', 'LineWidth', 1);
% plot(tref, yref, 'bo', 'LineWidth', 1);
% plot(tref, thetaref, 'go', 'LineWidth', 1);
% l = legend ('MPC MR $x(t)$ ', 'MPC MR $y(t)$',' MPC MR $\theta(t)$', ' $x_{ref}(t)$',  '$y_{ref}(t)$' ,' $\theta_{ref}(t)$');
% set(l,'Interpreter','Latex');
% l = xlabel('Time (s)'); 
% set(l,'Interpreter','Latex');
%  
%  subplot(2,2,4);
% 
% 
% l = title('States trajectory MR');
% set(l,'Interpreter','Latex');
% plot(t, xs(:,1), 'r--', 'LineWidth', 2);
% hold on; grid on;
% plot(t, xs(:,2), 'b--', 'LineWidth', 2);
% plot(t, xs(:,3), 'g--', 'LineWidth', 2);
% hold on; grid on;
% plot(tref, xref, 'r*', 'LineWidth', 1);
% plot(tref, yref, 'b*', 'LineWidth', 1);
% plot(tref, thetaref, 'g*', 'LineWidth', 1);
% l = legend ('SR-MPC $x(t)$ ', 'SR-MPC $y(t)$', 'SR-MPC $\theta(t)$' );
% set(l,'Interpreter','Latex');
% l = xlabel('Time (s)'); 
% set(l,'Interpreter','Latex');



%% third comparison

figure('Name','Comparison with  SRMPC + MR planner')



subplot(2,2,1);
% l = title('Movemment on the plane');
% set(l,'Interpreter','Latex');
% CarPlots(x_p,y_p,theta_p); 
% l = xlabel('x'); 
% set(l,'Interpreter','Latex');
% l = ylabel('y'); 
% set(l,'Interpreter','Latex');

l = title('');
set(l,'Interpreter','Latex');
hold on; grid on;
plot(t, u1, 'r', 'LineWidth', 2);
plot(t, u2, 'b', 'LineWidth', 2);
l = xlabel('Time (s)'); 
set(l,'Interpreter','Latex');
l = legend('$u_1(t)$   MR-MPC','$u_2(t)$ MR-MPC');
set(l,'Interpreter','Latex');


 subplot(2,2,2);
% l = title('Movemment on the plane');
% set(l,'Interpreter','Latex');
% CarPlots(xsr,ysr,thetasr); 
% l = xlabel('x'); 
% set(l,'Interpreter','Latex');
% l = ylabel('y'); 
% set(l,'Interpreter','Latex');

l = title('');
set(l,'Interpreter','Latex');
hold on; grid on;
plot(t, u1p, 'r--', 'LineWidth', 2);
plot(t, u2p, 'b--', 'LineWidth', 2);
l = xlabel('Time (s)'); 
set(l,'Interpreter','Latex');
l = legend('$u_1(t)$ SR-MPC','$u_2(t)$ SR-MPC');
set(l,'Interpreter','Latex');

subplot(2,2,3);
l = title('States trajectory MPC');
set(l,'Interpreter','Latex');
plot(t, xmpc(:,1), 'r', 'LineWidth', 2);
hold on; grid on;
plot(t, xmpc(:,2), 'b', 'LineWidth', 2);
plot(t, xmpc(:,3), 'g', 'LineWidth', 2);
hold on; grid on;
plot(tref, xref, 'ro', 'LineWidth', 1);
plot(tref, yref, 'bo', 'LineWidth', 1);
plot(tref, thetaref, 'go', 'LineWidth', 1);
l = legend ('MPC MR $x(t)$ ', 'MPC MR $y(t)$',' MPC MR $\theta(t)$', ' $x_{ref}(t)$',  '$y_{ref}(t)$' ,' $\theta_{ref}(t)$');
set(l,'Interpreter','Latex');
l = xlabel('Time (s)'); 
set(l,'Interpreter','Latex'); 
 subplot(2,2,4);

l = title('States trajectory MR');
set(l,'Interpreter','Latex');
plot(t, xp(:,1), 'r--', 'LineWidth', 2);
hold on; grid on;
plot(t, xp(:,2), 'b--', 'LineWidth', 2);
plot(t, xp(:,3), 'g--', 'LineWidth', 2);
hold on; grid on;
plot(tref, yhat(:,1), 'r*', 'LineWidth', 1);
plot(tref, yhat(:,3), 'b*', 'LineWidth', 1);
plot(tref, thetaref, 'g*', 'LineWidth', 1);
l = legend ('SR-MPC with MR Planner $x(t)$ ', 'SR-MPC with MR planner $y(t)$', 'SR-MPC with MR planner $\theta(t)$' );
set(l,'Interpreter','Latex');
l = xlabel('Time (s)'); 
set(l,'Interpreter','Latex');

