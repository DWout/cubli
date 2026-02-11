%% Constants

% System parameters
l = 0.15; %Side length of cubli
m_h = 0.4; % Mass of housing [kg]
m_w = 0.15; % Mass of flywheel [kg]
I_h_xx = 2e-3; % Moment of intertia of housing [kg.m2]
I_w_xx = 1e-4; % Moment of inertia of flywheel - rotational axis [kg.m2]
I_w_yy = 4e-5; % Moment of inertia of flywheel - other axis [kg.m2]

K_m = 1; % Gain factor motor of flywheels
C_w = 0.1; % Damping factor of motor of flywheels

% Vector to COG
r_h = [0.5*l, 0.5*l, 0.5*l]; % Center of gravity of housing

r_w_1 = [0, 0.5*l, 0.5*l];% Center of gravity of flywheel 1
r_w_2 = [0.5*l, 0, 0.5*l];% Center of gravity of flywheel 2
r_w_3 = [0.5*l, 0.5*l, 0];% Center of gravity of flywheel 3

r_h_t = create_skew_sym_matrix_3(r_h); % Skew symetrix matrix of housing intertia
r_w_1_t = create_skew_sym_matrix_3(r_w_1); % Skew symetrix matrix of flywheel 1
r_w_2_t = create_skew_sym_matrix_3(r_w_2); % Skew symetrix matrix of flywheel 2
r_w_3_t = create_skew_sym_matrix_3(r_w_3); % Skew symetrix matrix of flywheel 3

% Tensors & matrices from system parameters
II_h = diag([I_h_xx,I_h_xx,I_h_xx]); % Inertia tensor h 

II_w_1 = diag([I_w_xx,I_w_yy,I_w_yy]); % Inertia tensor w1
II_w_2 = diag([I_w_yy,I_w_xx,I_w_yy]); % Inertia tensor w2
II_w_3 = diag([I_w_yy,I_w_yy,I_w_xx]); % Inertia tensor w3

II_w = I_w_xx*eye(3); % Inertia tensor w
II = II_h + II_w_1 + II_w_2 + II_w_3 - (m_h*r_h_t^2 + m_w*r_w_1_t^2 + m_w*r_w_2_t^2 + m_w*r_w_3_t^2); % Inertia tensor around O
II_hat = II - II_w; % Delta inertia tensor

M = m_h*r_h_t + m_w*r_w_1_t + m_w*r_w_2_t + m_w*r_w_3_t;

% Control:
Ts = 0.01; %Sec = 100Hz

% Define that this file ran completely
init_cubli_constants_ran = 1;

function m = create_skew_sym_matrix_3(v)
    m = [0,     -v(3),    v(2);
        v(3),   0,        -v(1);
        -v(2),  v(1),     0];
end