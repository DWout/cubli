% Run controller in workspace, designed with file cubli_control_merged.m,
% on a system that is slightly different then the one simulated. Note, the
% workspace must be complete, and not changed, after controller design.

% Check workspace for all parameters

% Make parameters slightly different

% Define real system parameters like sample time

% Create workspace variable that is used in sim_cubli_simu as motor torque pertubation

% Run sim, and write output

% Run system identification

Order         = [3 1 2];               % Model orders [ny nu nx].
Parameters    = [0.5; 0.003; 0.019; ...
                 9.81; 0.25; 0.016];   % Initial parameter vector.
InitialStates = [0; 0.1];              % Initial values of initial states.
nlgr_m    = idnlgrey('nlode_edge', Order, Parameters, InitialStates, 0)


function [x_dot, y] = nlode_edge(~, x, II_hat, II, M, II_w, K_m, C_w, K_canon, T_canon, equi_state_up, balancing_edge, rs)
    % Expand states with zeros for eom for entire system
    x_full = zeros(9,1);
    x_full(rs) = x;
    x = x_full;
    
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

    % The output are all the relevant states (all is measures)
    y=x(rs);

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
    end
end