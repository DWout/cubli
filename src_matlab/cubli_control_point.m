%% Constants
init_cubli_constants;

%% Make linear model

%Define edge for balancing
equi_up = [0; -2*atan((1 + sqrt(2))*(-2 + sqrt(2) + sqrt(9 - 6*sqrt(2)))); pi/4];
equi_state_up = [equi_up; zeros([6,1])];

% Linear models for equilibrium
F_up = get_F(equi_up(2),equi_up(3));

% Get rotation matrix around equilibrium
dgdphi_up = get_dgdphi(equi_up(2),equi_up(3));

% State space matrices
A_up = [zeros(3), F_up, zeros(3);
    II_hat\M*dgdphi_up, zeros(3), C_w*inv(II_hat);
    -II_hat\M*dgdphi_up, zeros(3), -C_w*(inv(II_hat) + inv(II_w))];
B_up = [zeros(3);
    -inv(II_hat)*K_m;
    (inv(II_hat) + inv(II_w))*K_m];

% Eliminiate unobservable yaw angle alpha and get total system
A_hat = A_up(2:end,2:end);
B_hat = B_up(2:end,:);
C_hat = eye(8);
D_hat = zeros([8,3]);
cubli_ss = ss(A_hat,B_hat,C_hat,D_hat);

% Put the system in canonical form to eliminate uncontrollable state
[cubli_ss_canon, T_canon] = canon(cubli_ss,"modal");

% Eliminate uncontrollable state
cubli_ss_canon_red = modred(cubli_ss_canon,8,'truncate');

%% Create controller
% Define Q, penalty for the states
Q_cubli = eye(7);

% Define R, penalty for the input
R_cubli = eye(3);

% Calculate LQR controller
[K_canon,~,CLP_canon] = lqr(cubli_ss_canon_red,Q_cubli,R_cubli);
[K_canon_d,~,CLP_canon_d] = lqrd(cubli_ss_canon_red.A, cubli_ss_canon_red.B, Q_cubli, R_cubli, Ts);

%% Init values
phi_0 = [0; 0; 0]; 
w_h_0 = [0; 0; 0];
w_w_0 = [0; 0; 0];

x_0 = [phi_0; w_h_0; w_w_0];

%% Run simulation for 20 sec
[t,y] = ode45(@(t,y) odefcn(t,y,II_hat,II,M,II_w,K_m,C_w,K_canon,T_canon,equi_state_up), [0 10], x_0);
sim_ts_phi = timeseries(y(:,1:3),t); % timeseries of position of housing
sim_ts_w_w = timeseries(y(:,7:9),t); % timeseries of speed of flying wheels
plot(t,y(:,7:9))

function x_dot = odefcn(~, x, II_hat, II, M, II_w, K_m, C_w, K_canon, T_canon, equi_state_up)
    % Input
    u = get_control_input(x, K_canon, T_canon, equi_state_up);

    % Get the alpha (a), beta (b) and gamma (g) coordinates in the I frame
    a = x(1); b = x(2); g = x(3);
    g_p = 9.81*[sin(b);-sin(g)*cos(b);-cos(g)*cos(b)];

    % Get angular acceleration of the housing
    w_h_dot = II_hat\(II*cross(x(4:6),x(4:6)) + M*g_p + II_w*cross(x(7:9),x(4:6)) - (K_m*u - C_w*x(7:9)));

    % Get angular acceleration of the flying wheels
    w_w_dot = II_w\(K_m*u - C_w*x(7:9) - II_w*w_h_dot);

    % Get the angels in the I frame
    phi_dot = get_F(b,g) * x(4:6);

    % Get states_dot
    x_dot = [phi_dot; w_h_dot; w_w_dot];
end

function u = get_control_input(x, K_canon, T_canon, equi_state_up)
    % get x_equi_hat
    x_equi_hat = x-equi_state_up;
    
    % Remove unobservable state   
    x_equi_hat_obs = x_equi_hat(2:end);

    % Get canonical form of x
    x_canon = T_canon*x_equi_hat_obs;

    % Remove uncontrollable state, since it was also not used during
    x_canon_trunc = x_canon(1:end-1);

    % u = -K*x, following LQR controller
    u = -K_canon*x_canon_trunc;
end

function F = get_F(b,g)
    F = [0, sin(g)/cos(b), cos(g)/cos(b); 0, cos(g), -sin(g); 1, sin(g)*sin(b)/cos(b), cos(g)*sin(b)/cos(b)];
end

function dgdphi = get_dgdphi(b,g)
    dgdphi = [0, cos(b), 0; 0, sin(b)*sin(g), -cos(b)*cos(g); 0, sin(b)*cos(g), cos(b)*sin(g)];
end

function m = create_skew_sym_matrix_3(v)
    m = [0,     -v(3),    v(2);
        v(3),   0,        -v(1);
        -v(2),  v(1),     0];
end
