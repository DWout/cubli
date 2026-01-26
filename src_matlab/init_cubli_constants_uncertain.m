%% Perturbation constants
noise_scale_constants = 10; % The higher the value, the lower the noise
R_perturb = create_R_pertub(); % Create random rotation matrix for inertia tensors

% System parameters with perturbations from other init file
m_h_pert = m_h + m_h/noise_scale_constants * randn(); % Mass of housing [kg]
m_w_pert = m_w + m_w/noise_scale_constants * randn(); % Mass of flywheel [kg]
I_h_xx_pert = I_h_xx + I_h_xx/noise_scale_constants * randn(); % Moment of intertia of housing [kg.m2]
I_w_xx_pert = I_w_xx + I_w_xx/noise_scale_constants * randn(); % Moment of inertia of flywheel - rotational axis [kg.m2]
I_w_yy_pert = I_w_yy + I_w_yy/noise_scale_constants * randn(); % Moment of inertia of flywheel - other axis [kg.m2]

K_m_pert = K_m + K_m/noise_scale_constants * randn(); % Gain factor motor of flywheels
C_w_pert = C_w + C_w/noise_scale_constants * randn(); % Damping factor of motor of flywheels

% Vector to COG
r_h_pert = [0.5*l+l/2/noise_scale_constants*randn(), 0.5*l+l/2/noise_scale_constants*randn(), 0.5*l]+l/2/noise_scale_constants*randn(); % Center of gravity of housing

r_w_1_pert = [0, 0.5*l+l/2/noise_scale_constants*randn(), 0.5*l+l/2/noise_scale_constants*randn()];% Center of gravity of flywheel 1
r_w_2_pert = [0.5*l+l/2/noise_scale_constants*randn(), 0, 0.5*l+l/2/noise_scale_constants*randn()];% Center of gravity of flywheel 2
r_w_3_pert = [0.5*l+l/2/noise_scale_constants*randn(), 0.5*l+l/2/noise_scale_constants*randn(), 0];% Center of gravity of flywheel 3

r_h_t_pert = create_skew_sym_matrix_3(r_h_pert); % Skew symetrix matrix of housing intertia
r_w_1_t_pert = create_skew_sym_matrix_3(r_w_1_pert); % Skew symetrix matrix of flywheel 1
r_w_2_t_pert = create_skew_sym_matrix_3(r_w_2_pert); % Skew symetrix matrix of flywheel 2
r_w_3_t_pert = create_skew_sym_matrix_3(r_w_3_pert); % Skew symetrix matrix of flywheel 3

% Tensors & matrices from system parameters
II_h_pert = R_perturb*diag([I_h_xx_pert,I_h_xx_pert,I_h_xx_pert])*R_perturb'; % Inertia tensor h 

II_w_1_pert = R_perturb*diag([I_w_xx_pert,I_w_yy_pert,I_w_yy_pert])*R_perturb'; % Inertia tensor w1 
II_w_2_pert = R_perturb*diag([I_w_yy_pert,I_w_xx_pert,I_w_yy_pert])*R_perturb'; % Inertia tensor w2 
II_w_3_pert = R_perturb*diag([I_w_yy_pert,I_w_yy_pert,I_w_xx_pert])*R_perturb'; % Inertia tensor w3

II_w_pert = diag([II_w_1_pert(1,1),II_w_2_pert(2,2),II_w_3_pert(3,3)]); % Inertia tensor w
II_pert = II_h_pert + II_w_1_pert + II_w_2_pert + II_w_3_pert - (m_h_pert*r_h_t_pert^2 + m_w_pert*r_w_1_t_pert^2 + m_w_pert*r_w_2_t_pert^2 + m_w_pert*r_w_3_t_pert^2); % Inertia tensor around O
II_hat_pert = II_pert - II_w_pert; % Delta inertia tensor

M_pert = m_h_pert*r_h_t_pert + m_w_pert*r_w_1_t_pert + m_w_pert*r_w_2_t_pert + m_w_pert*r_w_3_t_pert;

% Define that this file ran completely
init_cubli_constants_uncertain_ran = 1;

function m = create_skew_sym_matrix_3(v)
    m = [0,     -v(3),    v(2);
        v(3),   0,        -v(1);
        -v(2),  v(1),     0];
end

function R_perturb = create_R_pertub()
    % Define perturbation magnitude (in radians)
    % 0.01 to 0.05 is a "little"; 0.1 is significant
    eps_mag = 0.04; 

    % 3. Create small random rotation angles for x, y, and z axes
    angles = (2*rand(3,1) - 1) * eps_mag; 
    ax = angles(1); ay = angles(2); az = angles(3);

    % 4. Create basic rotation matrices
    Rx = [1, 0, 0; 0, cos(ax), -sin(ax); 0, sin(ax), cos(ax)];
    Ry = [cos(ay), 0, sin(ay); 0, 1, 0; -sin(ay), 0, cos(ay)];
    Rz = [cos(az), -sin(az), 0; sin(az), cos(az), 0; 0, 0, 1];

    % 5. Combine to form the small perturbation rotation matrix
    R_perturb = Rx * Ry * Rz;
end