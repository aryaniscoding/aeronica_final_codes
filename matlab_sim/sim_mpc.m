%% =========================================================================
%  DRONE EXPLICIT MPC SIMULATION (static gain, same as STM32 code)
%  Run: run('sim_mpc.m')
%  =========================================================================
clear; clc; close all;

Ts = 0.002; T_end = 10; t = 0:Ts:T_end; N = length(t);
Ixx = 0.0142; Iyy = 0.0142; Izz = 0.0284;
n = 6; m = 3;

A_c = [0 0 0 1 0 0; 0 0 0 0 1 0; 0 0 0 0 0 1;
       0 0 0 -0.5/Ixx 0 0; 0 0 0 0 -0.5/Iyy 0; 0 0 0 0 0 -0.5/Izz];
B_c = [zeros(3,3); diag([1/Ixx, 1/Iyy, 1/Izz])];
Ad = eye(n)+A_c*Ts; Bd = B_c*Ts;

% DARE (manual)
Q = diag([120 120 100 12 12 10]); R = diag([1 1 1]);
P = Q;
for iter = 1:500
    P_old = P;
    Kt = (R+Bd'*P*Bd)\(Bd'*P*Ad);
    P = Q + Ad'*P*(Ad-Bd*Kt);
    if max(abs(P(:)-P_old(:)))<1e-10
        fprintf('DARE converged in %d iterations\n', iter);
        break;
    end
end
K_mpc = (R+Bd'*P*Bd)\(Bd'*P*Ad);
fprintf('Explicit MPC Gain K_mpc (3x6):\n'); disp(K_mpc);

% Reference
ref = zeros(n,N);
ref(1,t>=1&t<5)=0.1; ref(2,t>=2&t<6)=0.08; ref(3,t>=3&t<7)=0.15;

% Simulate
x = zeros(n,1); prev_u=zeros(m,1); U_MAX=500; SLEW=50;
log_x=zeros(n,N); log_m=zeros(4,N);

for k = 1:N
    e = x-ref(:,k);
    u = max(-U_MAX,min(U_MAX,-K_mpc*e));
    for j=1:m
        d=u(j)-prev_u(j);
        if d>SLEW, u(j)=prev_u(j)+SLEW; elseif d<-SLEW, u(j)=prev_u(j)-SLEW; end
    end
    prev_u=u;
    m1=max(1000,min(2000,1500-u(1)+u(2)-u(3)));
    m2=max(1000,min(2000,1500+u(1)+u(2)+u(3)));
    m3=max(1000,min(2000,1500-u(1)-u(2)+u(3)));
    m4=max(1000,min(2000,1500+u(1)-u(2)-u(3)));
    log_x(:,k)=x; log_m(:,k)=[m1;m2;m3;m4];
    x = Ad*x + Bd*u;
    if k==round(4/Ts), x(4)=x(4)+0.5; x(5)=x(5)+0.3; end
end

%% ---- PLOTS ----
set(0,'DefaultAxesFontSize',12,'DefaultLineLineWidth',2,...
      'DefaultTextColor','k','DefaultAxesXColor','k','DefaultAxesYColor','k');

figure('Name','Explicit MPC Simulation','NumberTitle','off','Position',[50 50 1200 750],'Color','w');

subplot(2,1,1);
plot(t, rad2deg(log_x(1,:)), 'Color',[0.9 0.1 0.2], 'LineWidth',2.5); hold on;
plot(t, rad2deg(log_x(2,:)), 'Color',[0.1 0.7 0.3], 'LineWidth',2.5);
plot(t, rad2deg(log_x(3,:)), 'Color',[0.2 0.4 0.9], 'LineWidth',2.5);
plot(t, rad2deg(ref(1,:)), 'Color',[0.9 0.1 0.2], 'LineWidth',1.5, 'LineStyle','--');
plot(t, rad2deg(ref(2,:)), 'Color',[0.1 0.7 0.3], 'LineWidth',1.5, 'LineStyle','--');
plot(t, rad2deg(ref(3,:)), 'Color',[0.2 0.4 0.9], 'LineWidth',1.5, 'LineStyle','--');
xline(4,'k--','Wind Gust','LineWidth',1.5);
xlabel('Time (s)','FontWeight','bold'); ylabel('Angle (deg)','FontWeight','bold');
title('Explicit MPC - Attitude Response','FontSize',14,'FontWeight','bold');
legend('Roll','Pitch','Yaw','Roll ref','Pitch ref','Yaw ref','Location','best');
grid on; box on;

subplot(2,1,2);
plot(t, log_m(1,:), 'Color',[0.9 0.1 0.2], 'LineWidth',2); hold on;
plot(t, log_m(2,:), 'Color',[0.1 0.7 0.3], 'LineWidth',2);
plot(t, log_m(3,:), 'Color',[0.2 0.4 0.9], 'LineWidth',2);
plot(t, log_m(4,:), 'Color',[0.8 0.2 0.8], 'LineWidth',2);
xlabel('Time (s)','FontWeight','bold'); ylabel('PWM (us)','FontWeight','bold');
title('Explicit MPC - Motor PWM Outputs','FontSize',14,'FontWeight','bold');
legend('Motor 1','Motor 2','Motor 3','Motor 4','Location','best');
ylim([900 2100]); grid on; box on;

sgtitle('Drone Explicit MPC Simulation','FontSize',16,'FontWeight','bold');
fprintf('Explicit MPC Simulation complete.\n');
