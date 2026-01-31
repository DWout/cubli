clc; clear all;

%% Load constants
init_cubli_constants;

%% Define controller

%Define edge for balancing
balancing_edge = 1;
equi_up = get_equilibrium_angles(balancing_edge);
equi_state_up = [equi_up; zeros([6,1])];

%% Define linear model for total system
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

%% Get subsection of system for edge balancing
ss_edge = get_edge_ss(A_up, B_up, balancing_edge);

%% Create controller
% Define Q, penalty for the states
Q_edge = eye(3);

% Define R, penalty for the input
R_edge = 100000000;

% Calculate LQR controller
[K_edge,~,CLP_edge] = lqr(ss_edge,Q_edge,R_edge);
[K_edge_d,~,CLP_edge_d] = lqrd(ss_edge.A, ss_edge.B,Q_edge,R_edge,Ts);

%% Print control values
disp("Edge " + string(balancing_edge) + ", TS " + string(Ts))
disp(strjoin(string(K_edge_d), ','))

%% Init values
if balancing_edge == 3
    phi_0 = [0; -pi/4+pi/4; pi/2]; 
else
    phi_0 = [0; 0; 0]; 
end
w_h_0 = [0; 0; 0];
w_w_0 = [0; 0; 0];

x_0 = [phi_0; w_h_0; w_w_0];

%% Run simulation for x sec
[t,y] = ode45(@(t,y) odefcn_edge(t,y,II_hat,II,M,II_w,K_m,C_w,K_edge,equi_state_up,balancing_edge), [0 10], x_0);
sim_ts_phi = timeseries(y(:,1:3),t); % timeseries of position of housing
sim_ts_w_w = timeseries(y(:,7:9),t); % timeseries of speed of flying wheels

u_vec = zeros(length(t),3);
for c = 1:length(t)
    u_vec(c,:) = get_control_input(y(c,:)', equi_state_up, K_edge, balancing_edge)';
end

plot(u_vec);
legend('Input 1','Input 2','Input 3')

plot(t,y(:,get_relevant_states(balancing_edge)))
legend('Phi angle','Rad/s housing', 'Rad/s flywheel')

figure
plot(t,y(:,[2,3,4,5,6]))
legend('Beta','Gamma','Rad/s H e1','Rad/s H e2','Rad/s H e3')
function x_dot = odefcn_edge(~, x, II_hat, II, M, II_w, K_m, C_w, K_edge, equi_state_up, balancing_edge)
    % Input
    u = get_control_input(x, equi_state_up, K_edge, balancing_edge);

    % Get the alpha (a), beta (b) and gamma (g) coordinates in the I frame
    a = x(1); b = x(2); g = x(3);
    g_p = 9.81*[sin(b);-sin(g)*cos(b);-cos(g)*cos(b)];

    % Get angular acceleration of the housing
    w_h_dot = II_hat\(II*cross(x(4:6),x(4:6)) + M*g_p + II_w*cross(x(7:9),x(4:6)) - (K_m*u - C_w*x(7:9)));
    if balancing_edge == 1
        w_h_dot = [w_h_dot(1); 0; 0];
    elseif balancing_edge == 2
        w_h_dot = [0; w_h_dot(2); 0];
    elseif balancing_edge == 3
        w_h_dot = [0; 0; w_h_dot(3)];    
    end

    % Get angular acceleration of the flying wheels
    w_w_dot = II_w\(K_m*u - C_w*x(7:9) - II_w*w_h_dot);
    if balancing_edge == 1
        w_w_dot = [w_w_dot(1); 0; 0];
    elseif balancing_edge == 2
        w_w_dot = [0; w_w_dot(2); 0];
    elseif balancing_edge == 3
        w_w_dot = [0; 0; w_w_dot(3)];
    end

    % Get the angels in the I frame
    phi_dot = get_F(b,g) * x(4:6);

    % Get states_dot
    x_dot = [phi_dot; w_h_dot; w_w_dot];
end

function u = get_control_input(x, equi_state_up, K_edge, balancing_edge)
    x_hat = x - equi_state_up;

    % Define 0 input
    u = [0;0;0];

    % Get relevant states for balancing edge
    rs = get_relevant_states(balancing_edge);

    % Extract relevant states from state vector
    x_rel = x_hat(rs);

    % Define input for balancing on edge
    u_rel = -K_edge * x_rel;

    % Change respective input
    u(balancing_edge) = u_rel;
end

function F = get_F(b,g)
    F = [0, sin(g)/cos(b), cos(g)/cos(b); 0, cos(g), -sin(g); 1, sin(g)*sin(b)/cos(b), cos(g)*sin(b)/cos(b)];
end

function dgdphi = get_dgdphi(b,g)
    dgdphi = [0, cos(b), 0; 0, sin(b)*sin(g), -cos(b)*cos(g); 0, sin(b)*cos(g), cos(b)*sin(g)];
end

function edge_ss = get_edge_ss(A_hat, B_hat, balancing_edge)
    rs = get_relevant_states(balancing_edge);

    A_edge = A_hat(rs,rs);
    B_edge = B_hat(rs,balancing_edge);
    C_edge = eye(3);
    D_edge = zeros([3,1]);

    edge_ss = ss(A_edge, B_edge, C_edge, D_edge);
end

function relevant_states = get_relevant_states(balancing_edge)
    if balancing_edge == 1
        relevant_states = [3,4,7];
    elseif balancing_edge == 2
        relevant_states = [2,5,8];
    elseif balancing_edge == 3
        relevant_states = [2,6,9];
    end
end

function equi = get_equilibrium_angles(balancing_edge)
    % Angels are defines as euler angles in the body fixed frame, relative
    % to the interital frame.
    % alpha (yaw) = rotation about inertial Z
    % beta (pitch) = rotation about new Y-axis
    % gamma (roll) = rotation about new X-axis

    if balancing_edge == 1
        equi = [0;0;pi/4];
    elseif balancing_edge == 2
        equi = [0;-pi/4;0];
    elseif balancing_edge == 3
        equi = [0;-pi/4;pi/2];
    end
end
