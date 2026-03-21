%% =========================================================================
%  DYNAMIC MPC — Online QP Solver for Quadcopter Attitude Control
%  NO TOOLBOXES REQUIRED — custom QP solver + manual DARE
%  Run: run('sim_mpc_dynamic.m')
%  =========================================================================
clear; clc; close all;
fprintf('=== Dynamic MPC with Online QP Solver ===\n');
fprintf('    (No Toolboxes Required)\n\n');

%% ---- Parameters ----
Ixx = 0.0142; Iyy = 0.0142; Izz = 0.0284;
damp_c = 0.5;
Ts = 0.01; T_end = 10;
t = 0:Ts:T_end; N_sim = length(t);
n = 6; m = 3;

A_c = [0 0 0 1 0 0; 0 0 0 0 1 0; 0 0 0 0 0 1;
       0 0 0 -damp_c/Ixx 0 0; 0 0 0 0 -damp_c/Iyy 0; 0 0 0 0 0 -damp_c/Izz];
B_c = [zeros(3,3); diag([1/Ixx, 1/Iyy, 1/Izz])];
Ad = eye(n) + A_c*Ts;  Bd = B_c*Ts;

fprintf('Plant: %d states, %d inputs, Ts=%.0f ms\n', n, m, Ts*1000);

%% ---- MPC Design ----
N_hz = 10;
Q = diag([150 150 120 15 15 12]);
R = diag([1 1 1]);

% Solve DARE manually (iterative Riccati)
P_dare = Q;
for iter = 1:500
    P_old = P_dare;
    Kt = (R + Bd'*P_dare*Bd)\(Bd'*P_dare*Ad);
    P_dare = Q + Ad'*P_dare*(Ad - Bd*Kt);
    if max(abs(P_dare(:) - P_old(:))) < 1e-10
        fprintf('DARE converged in %d iterations\n', iter);
        break;
    end
end

u_lo = -500; u_hi = 500;
fprintf('Horizon N=%d, constraints u in [%d, %d]\n\n', N_hz, u_lo, u_hi);

%% ---- Prediction Matrices ----
Phi = zeros(N_hz*n, n);
Gamma = zeros(N_hz*n, N_hz*m);
for i = 1:N_hz
    Phi((i-1)*n+1:i*n, :) = Ad^i;
    for j = 1:i
        Gamma((i-1)*n+1:i*n, (j-1)*m+1:j*m) = Ad^(i-j)*Bd;
    end
end
Q_bar = blkdiag(kron(eye(N_hz-1), Q), P_dare);
R_bar = kron(eye(N_hz), R);
H = Gamma'*Q_bar*Gamma + R_bar;
H = (H+H')/2;

nU = N_hz*m;
lb = ones(nU,1)*u_lo;
ub = ones(nU,1)*u_hi;

% Step size for projected gradient
alpha = 1 / max(sum(abs(H),2));

fprintf('QP: %d variables, H is %dx%d\n', nU, nU, nU);

%% ---- Reference ----
ref = zeros(n, N_sim);
ref(1, t>=1 & t<5) = 0.1;
ref(2, t>=2 & t<6) = 0.08;
ref(3, t>=3 & t<7) = 0.15;

%% ---- Simulation ----
fprintf('Running %d steps...\n', N_sim);
x = zeros(n,1);
log_x = zeros(n,N_sim);  log_u = zeros(m,N_sim);
log_m = zeros(4,N_sim);   log_t_qp = zeros(1,N_sim);

for k = 1:N_sim
    tic;

    % Stack reference over horizon
    X_ref = zeros(N_hz*n,1);
    for i = 1:N_hz
        idx = min(k+i, N_sim);
        X_ref((i-1)*n+1:i*n) = ref(:,idx);
    end

    % QP linear cost
    f_qp = Gamma'*Q_bar*(Phi*x - X_ref);

    % ---- Solve QP: Projected Nesterov Gradient ----
    U = H\(-f_qp);                     % Unconstrained solution
    U = max(lb, min(ub, U));            % Project

    U_prev = U;
    for it = 1:200
        V = U + (it-1)/(it+2)*(U - U_prev);   % Nesterov momentum
        U_prev = U;
        U = V - alpha*(H*V + f_qp);            % Gradient step
        U = max(lb, min(ub, U));                % Project
        if max(abs(U - U_prev)) < 1e-6, break; end
    end

    log_t_qp(k) = toc;

    % First control only (receding horizon)
    u_k = U(1:m);

    % Motor mixing
    m1 = 1500 - u_k(1) + u_k(2) - u_k(3);
    m2 = 1500 + u_k(1) + u_k(2) + u_k(3);
    m3 = 1500 - u_k(1) - u_k(2) + u_k(3);
    m4 = 1500 + u_k(1) - u_k(2) - u_k(3);

    log_x(:,k) = x; log_u(:,k) = u_k;
    log_m(:,k) = max(1000, min(2000, [m1;m2;m3;m4]));

    x = Ad*x + Bd*u_k;
    if k == round(4/Ts)
        x(4) = x(4)+0.5; x(5) = x(5)+0.3;
        fprintf('  Wind gust at t=4s\n');
    end
end
fprintf('Done! Avg QP: %.3f ms, Max: %.3f ms\n\n', mean(log_t_qp)*1000, max(log_t_qp)*1000);

%% ---- Also run Explicit MPC for comparison ----
K_exp = (R + Bd'*P_dare*Bd)\(Bd'*P_dare*Ad);
x2 = zeros(n,1); log_x2 = zeros(n,N_sim);
for k = 1:N_sim
    e = x2 - ref(:,k);
    u2 = max(u_lo, min(u_hi, -K_exp*e));
    log_x2(:,k) = x2;
    x2 = Ad*x2 + Bd*u2;
    if k==round(4/Ts), x2(4)=x2(4)+0.5; x2(5)=x2(5)+0.3; end
end

%% ---- PLOTS (vibrant, bold, visible) ----
set(0,'DefaultAxesFontSize',12,'DefaultLineLineWidth',2,...
      'DefaultTextColor','k','DefaultAxesXColor','k','DefaultAxesYColor','k');

% Colors
C_dyn  = [0.0 0.6 1.0];   % bright blue
C_exp  = [1.0 0.4 0.0];   % bright orange
C_ref  = [0.5 0.5 0.5];   % grey
C_m1 = [0.9 0.1 0.2]; C_m2 = [0.1 0.7 0.3]; C_m3 = [0.2 0.4 0.9]; C_m4 = [0.8 0.2 0.8];

% Fig 1: Attitude
figure('Name','Dynamic MPC Simulation','NumberTitle','off','Position',[50 50 1200 800],'Color','w');
ax_labels = {'Roll','Pitch','Yaw'};
for a = 1:3
    subplot(3,1,a);
    plot(t, rad2deg(log_x(a,:)), 'Color', C_dyn, 'LineWidth', 2.5); hold on;
    plot(t, rad2deg(log_x2(a,:)), 'Color', C_exp, 'LineWidth', 2, 'LineStyle','--');
    plot(t, rad2deg(ref(a,:)), 'Color', C_ref, 'LineWidth', 1.5, 'LineStyle',':');
    xline(4, 'k--', 'LineWidth', 1);
    ylabel([ax_labels{a} ' (deg)'], 'FontWeight','bold');
    set(gca, 'FontSize', 11); grid on; box on;
    if a==1
        title('Dynamic MPC (QP online) vs Explicit MPC (static gain)', 'FontSize', 14);
        legend('Dynamic MPC (QP)', 'Explicit MPC (static)', 'Reference', 'Location','best');
    end
    if a==3, xlabel('Time (s)', 'FontWeight','bold'); end
end
sgtitle('Quadcopter Dynamic MPC Simulation', 'FontSize', 16, 'FontWeight', 'bold');

% Fig 2: Motors
figure('Name','Motor PWM','NumberTitle','off','Position',[80 40 1200 500],'Color','w');
plot(t, log_m(1,:), 'Color', C_m1, 'LineWidth', 2); hold on;
plot(t, log_m(2,:), 'Color', C_m2, 'LineWidth', 2);
plot(t, log_m(3,:), 'Color', C_m3, 'LineWidth', 2);
plot(t, log_m(4,:), 'Color', C_m4, 'LineWidth', 2);
xline(4, 'k--', 'Wind', 'LineWidth', 1);
xlabel('Time (s)', 'FontWeight','bold'); ylabel('PWM (us)', 'FontWeight','bold');
title('Dynamic MPC - Motor PWM Outputs', 'FontSize', 14, 'FontWeight','bold');
legend('Motor 1 (FL)','Motor 2 (FR)','Motor 3 (RL)','Motor 4 (RR)','Location','best');
ylim([900 2100]); grid on; box on; set(gca,'FontSize',11);

% Fig 3: Control + Constraints
figure('Name','Control Inputs','NumberTitle','off','Position',[110 30 1200 600],'Color','w');
clr_u = {[0.9 0.1 0.2],[0.1 0.7 0.3],[0.2 0.4 0.9]};
ulbl = {'Roll torque','Pitch torque','Yaw torque'};
for a = 1:3
    subplot(3,1,a);
    plot(t, log_u(a,:), 'Color', clr_u{a}, 'LineWidth', 2); hold on;
    yline(u_hi, 'r--', 'u_{max}', 'LineWidth', 1.5);
    yline(u_lo, 'r--', 'u_{min}', 'LineWidth', 1.5);
    ylabel(ulbl{a}, 'FontWeight','bold'); grid on; box on;
    set(gca, 'FontSize', 11);
    if a==1, title('Control Inputs with Constraint Boundaries', 'FontSize', 14, 'FontWeight','bold'); end
    if a==3, xlabel('Time (s)', 'FontWeight','bold'); end
end

% Fig 4: QP timing
figure('Name','QP Timing','NumberTitle','off','Position',[140 20 1200 400],'Color','w');
subplot(1,2,1);
plot(t, log_t_qp*1000, '.', 'Color', C_dyn, 'MarkerSize', 5);
yline(Ts*1000, 'r--', sprintf('Limit %.0fms', Ts*1000), 'LineWidth', 2);
xlabel('Time (s)'); ylabel('Solve time (ms)');
title('QP Solve Time per Step', 'FontWeight','bold'); grid on;
subplot(1,2,2);
histogram(log_t_qp*1000, 50, 'FaceColor', C_dyn, 'EdgeColor','w');
xline(Ts*1000, 'r--', 'Limit', 'LineWidth', 2);
xlabel('Solve time (ms)'); ylabel('Count');
title('QP Solve Time Distribution', 'FontWeight','bold'); grid on;
sgtitle('QP Solver Performance', 'FontSize', 14, 'FontWeight','bold');

fprintf('\n=== WHY EXPLICIT MPC ON STM32 ===\n');
fprintf('QP takes ~%.1f ms on PC.\n', mean(log_t_qp)*1000);
fprintf('On STM32 (84MHz) it would take ~50-200ms = too slow.\n');
fprintf('Explicit MPC = one matrix multiply = ~0.01ms.\n');
