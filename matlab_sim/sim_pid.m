%% =========================================================================
%  DRONE PID CONTROLLER SIMULATION
%  Run: run('sim_pid.m')
%  =========================================================================
clear; clc; close all;

Ts = 0.002; T_end = 10; t = 0:Ts:T_end; N = length(t);
Ixx = 0.0142; Iyy = 0.0142; Izz = 0.0284;

% PID Gains
Kp_roll=4.0; Ki_roll=0.02; Kd_roll=1.5;
Kp_pitch=4.0; Ki_pitch=0.02; Kd_pitch=1.5;
Kp_yaw=6.0; Ki_yaw=0.05; Kd_yaw=0.5;
integral_limit=200; d_filter_alpha=0.1;

% Reference
ref_roll=zeros(1,N); ref_pitch=zeros(1,N); ref_yaw=zeros(1,N);
ref_roll(t>=1 & t<5)=0.1; ref_pitch(t>=2 & t<6)=0.08; ref_yaw(t>=3 & t<7)=0.15;

% State
roll=0; pitch=0; yaw=0; p=0; q=0; r=0;
int_r=0; pe_r=0; pd_r=0; int_p=0; pe_p=0; pd_p=0; int_y=0; pe_y=0; pd_y=0;

log_roll=zeros(1,N); log_pitch=zeros(1,N); log_yaw=zeros(1,N);
log_m1=zeros(1,N); log_m2=zeros(1,N); log_m3=zeros(1,N); log_m4=zeros(1,N);

for k = 1:N
    % Roll PID
    err=ref_roll(k)-roll;
    int_r=max(-integral_limit,min(integral_limit,int_r+err*Ts));
    rd=(err-pe_r)/Ts; fd_r=d_filter_alpha*rd+(1-d_filter_alpha)*pd_r;
    u_r=Kp_roll*err+Ki_roll*int_r+Kd_roll*fd_r; pe_r=err; pd_r=fd_r;

    % Pitch PID
    err=ref_pitch(k)-pitch;
    int_p=max(-integral_limit,min(integral_limit,int_p+err*Ts));
    rd=(err-pe_p)/Ts; fd_p=d_filter_alpha*rd+(1-d_filter_alpha)*pd_p;
    u_p=Kp_pitch*err+Ki_pitch*int_p+Kd_pitch*fd_p; pe_p=err; pd_p=fd_p;

    % Yaw PID
    err=ref_yaw(k)-yaw;
    int_y=max(-integral_limit,min(integral_limit,int_y+err*Ts));
    rd=(err-pe_y)/Ts; fd_y=d_filter_alpha*rd+(1-d_filter_alpha)*pd_y;
    u_y=Kp_yaw*err+Ki_yaw*int_y+Kd_yaw*fd_y; pe_y=err; pd_y=fd_y;

    % Motor mixing
    m1=max(1000,min(2000, 1500 - u_r + u_p - u_y));
    m2=max(1000,min(2000, 1500 + u_r + u_p + u_y));
    m3=max(1000,min(2000, 1500 - u_r - u_p + u_y));
    m4=max(1000,min(2000, 1500 + u_r - u_p - u_y));

    log_roll(k)=roll; log_pitch(k)=pitch; log_yaw(k)=yaw;
    log_m1(k)=m1; log_m2(k)=m2; log_m3(k)=m3; log_m4(k)=m4;

    % Plant
    p = p + (u_r/Ixx - 0.5*p)*Ts;
    q = q + (u_p/Iyy - 0.5*q)*Ts;
    r = r + (u_y/Izz - 0.5*r)*Ts;
    roll=roll+p*Ts; pitch=pitch+q*Ts; yaw=yaw+r*Ts;

    if k==round(4/Ts), p=p+0.5; q=q+0.3; end
end

%% ---- PLOTS (vibrant, bold, white background) ----
set(0,'DefaultAxesFontSize',12,'DefaultLineLineWidth',2,...
      'DefaultTextColor','k','DefaultAxesXColor','k','DefaultAxesYColor','k');

figure('Name','PID Simulation','NumberTitle','off','Position',[50 50 1200 750],'Color','w');

subplot(2,1,1);
plot(t, rad2deg(log_roll), 'Color',[0.9 0.1 0.2], 'LineWidth',2.5); hold on;
plot(t, rad2deg(log_pitch), 'Color',[0.1 0.7 0.3], 'LineWidth',2.5);
plot(t, rad2deg(log_yaw), 'Color',[0.2 0.4 0.9], 'LineWidth',2.5);
plot(t, rad2deg(ref_roll), 'Color',[0.9 0.1 0.2], 'LineWidth',1.5, 'LineStyle','--');
plot(t, rad2deg(ref_pitch), 'Color',[0.1 0.7 0.3], 'LineWidth',1.5, 'LineStyle','--');
plot(t, rad2deg(ref_yaw), 'Color',[0.2 0.4 0.9], 'LineWidth',1.5, 'LineStyle','--');
xline(4,'k--','Wind Gust','LineWidth',1.5);
xlabel('Time (s)','FontWeight','bold'); ylabel('Angle (degrees)','FontWeight','bold');
title('PID Controller - Attitude Response','FontSize',14,'FontWeight','bold');
legend('Roll','Pitch','Yaw','Roll ref','Pitch ref','Yaw ref','Location','best');
grid on; box on;

subplot(2,1,2);
plot(t, log_m1, 'Color',[0.9 0.1 0.2], 'LineWidth',2); hold on;
plot(t, log_m2, 'Color',[0.1 0.7 0.3], 'LineWidth',2);
plot(t, log_m3, 'Color',[0.2 0.4 0.9], 'LineWidth',2);
plot(t, log_m4, 'Color',[0.8 0.2 0.8], 'LineWidth',2);
xlabel('Time (s)','FontWeight','bold'); ylabel('PWM (us)','FontWeight','bold');
title('PID Controller - Motor PWM Outputs','FontSize',14,'FontWeight','bold');
legend('Motor 1','Motor 2','Motor 3','Motor 4','Location','best');
ylim([900 2100]); grid on; box on;

sgtitle('Drone PID Control Simulation','FontSize',16,'FontWeight','bold');
fprintf('PID Simulation complete.\n');
