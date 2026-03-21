%% =========================================================================
%  COMPARE ALL 4 CONTROLLERS: PID vs LQR vs Explicit MPC vs Dynamic MPC
%  NO TOOLBOXES REQUIRED
%  Run: run('compare_all.m')
%  =========================================================================
clear; clc; close all;
fprintf('=== Comparing PID vs LQR vs Explicit MPC vs Dynamic MPC ===\n\n');

%% ---- Common Setup ----
Ts = 0.002; T_end = 10; t = 0:Ts:T_end; N = length(t);
Ixx = 0.0142; Iyy = 0.0142; Izz = 0.0284;
n = 6; m = 3;
A_c = [0 0 0 1 0 0; 0 0 0 0 1 0; 0 0 0 0 0 1;
       0 0 0 -0.5/Ixx 0 0; 0 0 0 0 -0.5/Iyy 0; 0 0 0 0 0 -0.5/Izz];
B_c = [zeros(3,3); diag([1/Ixx, 1/Iyy, 1/Izz])];
Ad = eye(n)+A_c*Ts; Bd = B_c*Ts;

ref = zeros(n,N);
ref(1,t>=1&t<5)=0.1; ref(2,t>=2&t<6)=0.08; ref(3,t>=3&t<7)=0.15;
dist_k = round(4/Ts);

%% ======== 1. PID ========
fprintf('Running PID...\n');
Kp=[4;4;6]; Ki=[0.02;0.02;0.05]; Kd=[1.5;1.5;0.5];
integ=zeros(3,1); perr=zeros(3,1); pder=zeros(3,1);
al=0.1; il=200; U_MAX=500;
x = zeros(n,1); log_pid = zeros(3,N);
for k=1:N
    err=ref(1:3,k)-x(1:3);
    integ=max(-il,min(il,integ+err*Ts));
    rd=(err-perr)/Ts; fd=al*rd+(1-al)*pder;
    u=Kp.*err+Ki.*integ+Kd.*fd;
    perr=err; pder=fd;
    u=max(-U_MAX,min(U_MAX,u));
    log_pid(:,k)=x(1:3);
    x=Ad*x+Bd*u;
    if k==dist_k, x(4)=x(4)+0.5; x(5)=x(5)+0.3; end
end

%% ======== 2. LQR ========
fprintf('Running LQR...\n');
Q_lqr=diag([100 100 80 10 10 8]); R_lqr=diag([1 1 1]);
P=Q_lqr;
for i=1:500
    Po=P; Kt=(R_lqr+Bd'*P*Bd)\(Bd'*P*Ad);
    P=Q_lqr+Ad'*P*(Ad-Bd*Kt);
    if max(abs(P(:)-Po(:)))<1e-10, break; end
end
K_lqr=(R_lqr+Bd'*P*Bd)\(Bd'*P*Ad);
x=zeros(n,1); log_lqr=zeros(3,N);
for k=1:N
    e=x-ref(:,k); u=max(-U_MAX,min(U_MAX,-K_lqr*e));
    log_lqr(:,k)=x(1:3);
    x=Ad*x+Bd*u;
    if k==dist_k, x(4)=x(4)+0.5; x(5)=x(5)+0.3; end
end

%% ======== 3. Explicit MPC (static, with slew limit) ========
fprintf('Running Explicit MPC...\n');
Q_mpc=diag([120 120 100 12 12 10]); R_mpc=diag([1 1 1]);
P=Q_mpc;
for i=1:500
    Po=P; Kt=(R_mpc+Bd'*P*Bd)\(Bd'*P*Ad);
    P=Q_mpc+Ad'*P*(Ad-Bd*Kt);
    if max(abs(P(:)-Po(:)))<1e-10, break; end
end
K_exp=(R_mpc+Bd'*P*Bd)\(Bd'*P*Ad);
x=zeros(n,1); pu=zeros(m,1); SLEW=50;
log_empc=zeros(3,N);
for k=1:N
    e=x-ref(:,k); u=max(-U_MAX,min(U_MAX,-K_exp*e));
    for j=1:m
        d=u(j)-pu(j);
        if d>SLEW,u(j)=pu(j)+SLEW; elseif d<-SLEW,u(j)=pu(j)-SLEW; end
    end
    pu=u;
    log_empc(:,k)=x(1:3);
    x=Ad*x+Bd*u;
    if k==dist_k, x(4)=x(4)+0.5; x(5)=x(5)+0.3; end
end

%% ======== 4. Dynamic MPC (online QP, TIGHT constraints) ========
% KEY DIFFERENCE: Dynamic MPC uses TIGHT input constraints (+-150)
% This forces the QP to actively manage constraints, unlike explicit MPC
% which just saturates. This shows MPC's advantage: it optimally
% distributes control effort within tight limits.
fprintf('Running Dynamic MPC (QP, tight constraints)...\n');
N_hz = 10;
U_LIM_DYN = 150;   % Much tighter than explicit MPC's 500!

Q_dyn = diag([200 200 160 20 20 16]);  % More aggressive state cost
R_dyn = diag([0.5 0.5 0.5]);           % Lower control cost

P_d = Q_dyn;
for i=1:500
    Po=P_d; Kt=(R_dyn+Bd'*P_d*Bd)\(Bd'*P_d*Ad);
    P_d=Q_dyn+Ad'*P_d*(Ad-Bd*Kt);
    if max(abs(P_d(:)-Po(:)))<1e-10, break; end
end

% Build prediction matrices
Phi=zeros(N_hz*n,n); Gamma=zeros(N_hz*n,N_hz*m);
for i=1:N_hz
    Phi((i-1)*n+1:i*n,:)=Ad^i;
    for j=1:i, Gamma((i-1)*n+1:i*n,(j-1)*m+1:j*m)=Ad^(i-j)*Bd; end
end
Q_bar=blkdiag(kron(eye(N_hz-1),Q_dyn),P_d);
R_bar=kron(eye(N_hz),R_dyn);
H=Gamma'*Q_bar*Gamma+R_bar; H=(H+H')/2;
nU=N_hz*m;
lb=ones(nU,1)*(-U_LIM_DYN);
ub=ones(nU,1)*U_LIM_DYN;
alpha_qp=1/max(sum(abs(H),2));

x=zeros(n,1); log_dmpc=zeros(3,N); log_tqp=zeros(1,N);
for k=1:N
    tic;
    X_ref=zeros(N_hz*n,1);
    for i=1:N_hz
        idx=min(k+i,N);
        X_ref((i-1)*n+1:i*n)=ref(:,idx);
    end
    fq=Gamma'*Q_bar*(Phi*x-X_ref);

    % QP solve (projected Nesterov gradient)
    U=H\(-fq); U=max(lb,min(ub,U));
    U_prev=U;
    for it=1:200
        V=U+(it-1)/(it+2)*(U-U_prev);
        U_prev=U;
        U=max(lb,min(ub,V-alpha_qp*(H*V+fq)));
        if max(abs(U-U_prev))<1e-6, break; end
    end
    log_tqp(k)=toc;
    u=U(1:m);
    log_dmpc(:,k)=x(1:3);
    x=Ad*x+Bd*u;
    if k==dist_k, x(4)=x(4)+0.5; x(5)=x(5)+0.3; end
end
fprintf('  Avg QP time: %.3f ms\n\n', mean(log_tqp)*1000);

%% =========================================================================
%  PLOTS
%  =========================================================================
set(0,'DefaultAxesFontSize',12,'DefaultLineLineWidth',2,...
      'DefaultTextColor','k','DefaultAxesXColor','k','DefaultAxesYColor','k');

% Distinct colors
C_pid  = [0.90 0.15 0.15];   % RED
C_lqr  = [0.00 0.75 0.75];   % CYAN / TEAL (distinct from green/blue)
C_empc = [0.10 0.35 0.85];   % DEEP BLUE
C_dmpc = [0.80 0.20 0.80];   % MAGENTA
C_ref  = [0.45 0.45 0.45];   % GREY

labels = {'Roll','Pitch','Yaw'};

% ---- Figure 1: Attitude Comparison ----
figure('Name','All Controllers','NumberTitle','off','Position',[30 30 1300 850],'Color','w');
for ax = 1:3
    subplot(3,1,ax);
    h1=plot(t,rad2deg(log_pid(ax,:)),'Color',C_pid,'LineWidth',2.5); hold on;
    h2=plot(t,rad2deg(log_lqr(ax,:)),'Color',C_lqr,'LineWidth',3.0,'LineStyle','-');
    h3=plot(t,rad2deg(log_empc(ax,:)),'Color',C_empc,'LineWidth',2.5,'LineStyle','-');
    h4=plot(t,rad2deg(log_dmpc(ax,:)),'Color',C_dmpc,'LineWidth',2.5,'LineStyle','-');
    h5=plot(t,rad2deg(ref(ax,:)),'Color',C_ref,'LineWidth',2,'LineStyle','--');
    xline(4,'k:','Wind','LineWidth',1.5,'FontSize',10);
    ylabel([labels{ax} ' (deg)'],'FontWeight','bold');
    grid on; box on; set(gca,'FontSize',11);
    if ax==1
        title('PID vs LQR vs Explicit MPC vs Dynamic MPC','FontSize',14,'FontWeight','bold');
        legend([h1 h2 h3 h4 h5],{'PID (red)','LQR (cyan)','Explicit MPC (blue)','Dynamic MPC (magenta)','Reference'},...
               'Location','northeast','FontSize',10);
    end
    if ax==3, xlabel('Time (s)','FontWeight','bold'); end
end
sgtitle('Drone Attitude Control — 4 Algorithm Comparison','FontSize',16,'FontWeight','bold');

% ---- Figure 2: Zoomed Step + Recovery ----
figure('Name','Zoomed','NumberTitle','off','Position',[60 60 1300 500],'Color','w');

subplot(1,2,1);
plot(t,rad2deg(log_pid(1,:)),'Color',C_pid,'LineWidth',2.5); hold on;
plot(t,rad2deg(log_lqr(1,:)),'Color',C_lqr,'LineWidth',3.0);
plot(t,rad2deg(log_empc(1,:)),'Color',C_empc,'LineWidth',2.5);
plot(t,rad2deg(log_dmpc(1,:)),'Color',C_dmpc,'LineWidth',2.5);
plot(t,rad2deg(ref(1,:)),'Color',C_ref,'LineWidth',2,'LineStyle','--');
xlim([0.8 3]); xlabel('Time (s)','FontWeight','bold'); ylabel('Roll (deg)','FontWeight','bold');
title('Roll Step Response (Zoomed)','FontSize',13,'FontWeight','bold');
legend('PID','LQR','Exp MPC','Dyn MPC','Ref','Location','best');
grid on; box on;

subplot(1,2,2);
plot(t,rad2deg(log_pid(1,:)),'Color',C_pid,'LineWidth',2.5); hold on;
plot(t,rad2deg(log_lqr(1,:)),'Color',C_lqr,'LineWidth',3.0);
plot(t,rad2deg(log_empc(1,:)),'Color',C_empc,'LineWidth',2.5);
plot(t,rad2deg(log_dmpc(1,:)),'Color',C_dmpc,'LineWidth',2.5);
xline(4,'k:','Wind','LineWidth',1.5);
xlim([3.8 6]); xlabel('Time (s)','FontWeight','bold'); ylabel('Roll (deg)','FontWeight','bold');
title('Disturbance Recovery (Zoomed)','FontSize',13,'FontWeight','bold');
legend('PID','LQR','Exp MPC','Dyn MPC','Location','best');
grid on; box on;

sgtitle('Settling & Recovery Performance','FontSize',15,'FontWeight','bold');

% ---- Figure 3: Bar Charts ----
figure('Name','Metrics','NumberTitle','off','Position',[90 40 1000 500],'Color','w');

rng_track = (t>=1 & t<5);
rms_pid = sqrt(mean((log_pid(1,rng_track)-ref(1,rng_track)).^2));
rms_lqr = sqrt(mean((log_lqr(1,rng_track)-ref(1,rng_track)).^2));
rms_empc= sqrt(mean((log_empc(1,rng_track)-ref(1,rng_track)).^2));
rms_dmpc= sqrt(mean((log_dmpc(1,rng_track)-ref(1,rng_track)).^2));

subplot(1,2,1);
b=bar([rms_pid rms_lqr rms_empc rms_dmpc]*1000,'FaceColor','flat');
b.CData=[C_pid;C_lqr;C_empc;C_dmpc];
set(gca,'XTickLabel',{'PID','LQR','Exp MPC','Dyn MPC'},'FontSize',12);
ylabel('RMS Error (mrad)','FontWeight','bold');
title('Roll Tracking Error','FontSize',13,'FontWeight','bold');
grid on; box on;

rng_rec = find(t>=4);
thresh = 0.5*pi/180;
rec=[NaN NaN NaN NaN];
logs_all = {log_pid, log_lqr, log_empc, log_dmpc};
for a=1:4
    for ii=1:length(rng_rec)
        kk=rng_rec(ii);
        if abs(logs_all{a}(1,kk)-ref(1,kk))<thresh, rec(a)=t(kk)-4; break; end
    end
end

subplot(1,2,2);
b2=bar(rec*1000,'FaceColor','flat');
b2.CData=[C_pid;C_lqr;C_empc;C_dmpc];
set(gca,'XTickLabel',{'PID','LQR','Exp MPC','Dyn MPC'},'FontSize',12);
ylabel('Recovery Time (ms)','FontWeight','bold');
title('Disturbance Recovery Speed','FontSize',13,'FontWeight','bold');
grid on; box on;

sgtitle('Quantitative Comparison','FontSize',15,'FontWeight','bold');

%% ---- Print ----
fprintf('===========================================================\n');
fprintf('  ALGORITHM COMPARISON\n');
fprintf('===========================================================\n');
fprintf('  %-15s  %8s  %12s\n', 'Algorithm', 'RMS(mrad)', 'Recovery(ms)');
fprintf('  %-15s  %8.2f  %12.1f\n', 'PID',rms_pid*1000,rec(1)*1000);
fprintf('  %-15s  %8.2f  %12.1f\n', 'LQR',rms_lqr*1000,rec(2)*1000);
fprintf('  %-15s  %8.2f  %12.1f\n', 'Explicit MPC',rms_empc*1000,rec(3)*1000);
fprintf('  %-15s  %8.2f  %12.1f\n', 'Dynamic MPC',rms_dmpc*1000,rec(4)*1000);
fprintf('===========================================================\n');
fprintf('  Dynamic MPC constraint: +-%d (tight)\n', U_LIM_DYN);
fprintf('  Explicit MPC constraint: +-%d (loose)\n', U_MAX);
fprintf('  Dynamic MPC avg QP: %.3f ms\n', mean(log_tqp)*1000);
fprintf('===========================================================\n');
fprintf('\n  WHY DYNAMIC MPC IS DIFFERENT:\n');
fprintf('  It has TIGHT constraints (+-%d vs +-%d)\n', U_LIM_DYN, U_MAX);
fprintf('  The QP solver optimally distributes effort within limits.\n');
fprintf('  Explicit MPC just clips — no look-ahead planning.\n');
