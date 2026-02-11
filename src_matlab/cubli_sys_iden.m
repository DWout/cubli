

% Get cubli constants
run('init_cubli_constants.m');

% Get edge constants for perticular edge
be = 1; % Edge to balance on (1,2,3, alinging with e axis)

% Define state space model for LQR controller
A = [0, 1, 0;
     Th_T\M*g_0, 0, Th_T\C_w;
     -Th_T\M*g_0, 0, -C_w*(inv(T_wT) + inv(Th_T))];
B = [-Th_T\K_m; 0; K_m*(inv(T_wT) + inv(Th_T))];
C = eye(3);
D = zeros(3,1); 

% Define weights for LQR controller
Q = diag([100, 1, 1]);
R = 0.01;

% Create LQR controller
K = lqr(A, B, Q, R);

Order         = [3 1 3];               % Model orders [ny nu nx].
Parameters    = [0.5; 0.003; 0.019; ...
                 9.81; 0.25; 0.016];   % Initial parameter vector.
InitialStates = [0; 0.1];              % Initial values of initial states.
nlgr_m    = idnlgrey('nlode_edge', Order, Parameters, InitialStates, 0)


function [x_dot, y] = nlode_edge(~, x, K_m_est, C_w_est, Th_T_est, T_wT_est, r_h_est, K, m_h, m_w, r_w)
    % Define M and g_0
    M = m_h*r_h_est + m_w*r_w; %only r_h_est is estimated in this equation, since rest can be measured
    g_0 = 9.81; %Gravity

    % Input
    u = K*x;

    % Calculate state derivatives non-linear
    beta_dot = x(2);
    omega_h_dot = T_wT_est\(M*g_0*sin(x(1))-(K_m_est*u - C_w_est * x(3)));
    omega_w_dot = -K_m_est*(inv(T_wT_est) + inv(Th_T_est))*u - C_w_est*(inv(T_wT_est) + inv(Th_T_est))*x(3) - Th_T_est\(M*g_0*sin(x(1)));

    % define x_dot and y
    x_dot = [beta_dot; omega_h_dot; omega_w_dot];
    y = x;
end
