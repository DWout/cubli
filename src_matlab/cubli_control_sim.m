%% Run specific constants
be = 4; % Balancing edge (1-3 are edged, 4 is point)
add_sinus_input = false; %Add various sinusoidal distrubances to motor input to identify model

%% Define what part of file should be ran
run_sim_design_constants = true;
run_sim_pertubated_constants = true;
plot_figures = true;

%% Get constants.
init_cubli_constants;
init_cubli_constants_uncertain;

%% Create linear model
[balancing_edge,equi_state_up,rs,T_canon,cubli_ss_canon] = generate_linear_cubli_model(be, II_hat, M, C_w, II_w, K_m);

%% Create controller
[K_canon, ~, ~, ~] = create_lqr_controller(cubli_ss_canon, Ts);

%% Init values
x_0 = get_simu_initial_values(balancing_edge);

%% Run simulation on either design constants or pertubated constants
if run_sim_design_constants == true
    % Run simulation for 10 sec on model with correct constants
    [t,y] = ode45(@(t,y) odefcn(t,y,II_hat,II,M,II_w,K_m,C_w,K_canon,T_canon,equi_state_up, balancing_edge, rs), [0 10], x_0);

    if plot_figures == true
        % Plot figures corresponding to simulation on design constants    
        sim_ts_phi = timeseries(y(:,1:3),t,'Name','Housing position design constants'); % timeseries of position of housing
        sim_ts_w_w = timeseries(y(:,7:9),t,'Name','Flywheel speed design constants'); % timeseries of speed of flying wheels
        figure('Name','Simulation of design constants');plot(sim_ts_phi)
        legend('Yaw (alpha)','Pitch (beta)','Roll (gamma)');
    end
end

if run_sim_pertubated_constants == true
    % Run simulation for 10 sec on model with pertubated constants
    [t_pert,y_pert] = ode45(@(t_pert,y_pert) odefcn(t_pert,y_pert,II_hat_pert,II_pert,M_pert,II_w_pert,K_m_pert,C_w_pert,K_canon,T_canon,equi_state_up, balancing_edge, rs), [0 10], x_0);
    if plot_figures == true
        % Plot figures corresponding to simulation on pertubated constants
        sim_ts_phi_pert = timeseries(y_pert(:,1:3),t_pert,'Name','Housing position pertubated constants'); % timeseries of position of housing
        sim_ts_w_w_pert = timeseries(y_pert(:,7:9),t_pert,'Name','Flywheel speed pertubated constants'); % timeseries of speed of flying wheels
        figure('Name','Simulation of pertubated constants'); plot(sim_ts_phi_pert)
        legend('Yaw (alpha)','Pitch (beta)','Roll (gamma)');
    end
end

function x_dot = odefcn(~, x, II_hat, II, M, II_w, K_m, C_w, K_canon, T_canon, equi_state_up, balancing_edge, rs)
    % Input
    u = get_control_input(x, K_canon, T_canon, equi_state_up, balancing_edge, rs);

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
    
    if balancing_edge ~= 4
        x_dot_full = zeros(9,1);
        x_dot_full(rs) = x_dot(rs);
        x_dot = x_dot_full;
    end

end

function u = get_control_input(x, K_canon, T_canon, equi_state_up, balancing_edge, rs)
    % get x_equi_hat
    x_equi_hat = x-equi_state_up;
    
    % Remove unobservable state   
    x_equi_hat_obs = x_equi_hat(rs);

    % Get canonical form of x
    x_canon = T_canon*x_equi_hat_obs;

    % Remove uncontrollable state, since it was also not used during
    if balancing_edge == 4
        x_canon = x_canon(1:end-1);
    end

    % u = -K*x, following LQR controller
    u = -K_canon*x_canon;
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
	elseif balancing_edge == 4
		equi = [0; -2*atan((1 + sqrt(2))*(-2 + sqrt(2) + sqrt(9 - 6*sqrt(2)))); pi/4]; %upright balancing
    end
end

function [relevant_states, relevant_inputs] = get_relevant_states_inputs(balancing_edge)
    if balancing_edge == 1
        relevant_states = [3,4,7];
        relevant_inputs = 1;
    elseif balancing_edge == 2
        relevant_states = [2,5,8];
        relevant_inputs = 2;
    elseif balancing_edge == 3
        relevant_states = [2,6,9];
        relevant_inputs = 3;
    elseif balancing_edge == 4
        relevant_states = 2:9;
        relevant_inputs = 1:3;
    end
end

function [T_canon, cubli_ss_canon] = get_reduced_canon_system(A_up, B_up, balancing_edge, rs, ri)

% Get total system without unobservable/relevant states (e.g. yaw in point)
A_hat = A_up(rs,rs);
B_hat = B_up(rs,ri);
C_hat = eye(length(rs));
D_hat = zeros([length(rs),length(ri)]);
cubli_ss = ss(A_hat,B_hat,C_hat,D_hat);

% Put the system in canonical form to eliminate uncontrollable state
[cubli_ss_canon, T_canon] = canon(cubli_ss,"modal");

% Eliminate uncontrollable state
if balancing_edge == 4 %point
    cubli_ss_canon = modred(cubli_ss_canon,length(rs),'truncate');
end
end

function [balancing_edge,equi_state_up,rs,T_canon,cubli_ss_canon] = generate_linear_cubli_model(be, II_hat, M, C_w, II_w, K_m)
%% Make linear model

%Define edge for balancing
balancing_edge = be; %4 for point balancing, 1-3 are edges
equi_up = get_equilibrium_angles(balancing_edge);
equi_state_up = [equi_up; zeros([6,1])];

% Linear models for equilibrium
F_up = get_F(equi_up(2),equi_up(3));

% Get rotation matrix around equilibrium
dgdphi_up = get_dgdphi(equi_up(2),equi_up(3));

% Get linearized state dynamics
% State space matrices
A_up = [zeros(3), F_up, zeros(3);
    II_hat\M*dgdphi_up, zeros(3), C_w*inv(II_hat);
    -II_hat\M*dgdphi_up, zeros(3), -C_w*(inv(II_hat) + inv(II_w))];
B_up = [zeros(3);
    -inv(II_hat)*K_m;
    (inv(II_hat) + inv(II_w))*K_m];

% Get relevant states for control (removed uncontrollable/relevant states)
[rs, ri] = get_relevant_states_inputs(balancing_edge);

% Form complete system with A, B, without un controllable/observable states
[T_canon, cubli_ss_canon] = get_reduced_canon_system(A_up, B_up, balancing_edge, rs, ri);
end

function [K_canon, CLP_canon, K_canon_d, CLP_canon_d] = create_lqr_controller(cubli_ss_canon, Ts)
% Define Q, penalty for the states
Q_cubli = eye(length(cubli_ss_canon.A));

% Define R, penalty for the input
R_cubli = eye(size(cubli_ss_canon.D,2));

% Calculate LQR controller
[K_canon,~,CLP_canon] = lqr(cubli_ss_canon,Q_cubli,R_cubli);
[K_canon_d,~,CLP_canon_d] = lqrd(cubli_ss_canon.A, cubli_ss_canon.B, Q_cubli, R_cubli, Ts);
end

function x_0 = get_simu_initial_values(balancing_edge)
if balancing_edge == 3
    phi_0 = [0; -pi/4+pi/4; pi/2]; 
else
    phi_0 = [0; 0; 0]; 
end

w_h_0 = [0; 0; 0];
w_w_0 = [0; 0; 0];

x_0 = [phi_0; w_h_0; w_w_0];
end