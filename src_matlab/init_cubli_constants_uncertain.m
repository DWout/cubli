% 
% FUNCTION: init_cubli_constants_uncertain
%
% DESCRIPTION:
%   Initializes perturbation constants for the Cubli system with uncertainty.
%   This script defines and sets up constant values used for perturbation
%   analysis in uncertain system dynamics modeling.
%
% USAGE:
%   run init_cubli_constants_uncertain.m
%
% OUTPUTS:
%   Various perturbation constants stored in the workspace
%
% NOTES:
%   - Part of the Cubli project
%   - Located in srcom_catlab directory
%   - Related to uncertain system parameters
%
% AUTHOR: [Author Name]
% DATE: [Date]
% VERSION: 1.0

% Perturbation constants
noise_scale_constant = 5; % The higher the value, the lower the noise
eps_mag = 0.05; %defines how much the rotation matrix should rotate
R_perturb = create_R_pertub(eps_mag); % Create random rotation matrix for inertia tensors
com_c = 0.5*l; %center of mass constant

% System parameters with perturbations from other init file
m_h_pert = add_noise(m_h,noise_scale_constant); % Mass of housing [kg]
m_w_pert = add_noise(m_w,noise_scale_constant); % Mass of flywheel [kg]
I_h_xx_pert = add_noise(I_h_xx,noise_scale_constant); % Moment of intertia of housing [kg.m2]
I_w_xx_pert = add_noise(I_w_xx,noise_scale_constant); % Moment of inertia of flywheel - rotational axis [kg.m2]
I_w_yy_pert = add_noise(I_w_yy,noise_scale_constant); % Moment of inertia of flywheel - other axis [kg.m2]

K_m_pert = add_noise(K_m,noise_scale_constant); % Gain factor motor of flywheels
C_w_pert = add_noise(C_w,noise_scale_constant); % Damping factor of motor of flywheels

% Vector to COG
r_h_pert = [add_noise(com_c,noise_scale_constant), add_noise(com_c,noise_scale_constant), add_noise(com_c,noise_scale_constant)]; % Center of gravity of housing

r_w_1_pert = [0, add_noise(com_c,noise_scale_constant), add_noise(com_c,noise_scale_constant)];% Center of gravity of flywheel 1
r_w_2_pert = [add_noise(com_c,noise_scale_constant), 0, add_noise(com_c,noise_scale_constant)];% Center of gravity of flywheel 2
r_w_3_pert = [add_noise(com_c,noise_scale_constant), add_noise(com_c,noise_scale_constant), 0];% Center of gravity of flywheel 3

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

function R_perturb = create_R_pertub(eps_mag)
    % Define perturbation magnitude (in radians)
    % eps_mag can be in range 0.01 to 0.05 is a "little"; 0.1 is significant

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

function noised_con = add_noise (c, noise_scale_constant);
    % This funciton adds noise to a constant, based on the noise level, and a randoml drawn variable with mean 0 and var 1
    noised_con = c+c/noise_scale_constant*randn();
end