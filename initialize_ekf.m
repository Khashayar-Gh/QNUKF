function [state, params] = initialize_ekf(dim_x, dim_z, dt, dim_n, q_indices, P_q_indices)
    % Initialize the UKF parameters and state
    % Input:
    %   dim_x - dimension of the state vector
    %   dim_z - dimension of the measurement vector
    %   dt - time step
    %   alpha, beta, kappa - UKF tuning parameters
    %   sigma_mode - mode of sigma point generation
    %   augmentation - boolean flag for augmentation
    %   dim_xa - dimension of the augmented state vector
    %   xa - augmented state vector
    %   q_indices - indices of quaternions in the state vector
    %   P_q_indices - indices of quaternions in the process noise matrix
    % Initialize parameters
    params.dim_x = dim_x;
    params.dim_z = dim_z;
    params.dt = dt;
    params.q_indices = q_indices;
    params.P_q_indices = P_q_indices;

    % Initialize sigma points
    dim_P = dim_x - length(q_indices);
    state.dim_P = dim_P;

    % Initialize state
    state.x = zeros(dim_x, 1);  % State vector
    state.P = eye(dim_P);       % State covariance matrix
    params.Q = eye(dim_n);       % Process noise matrix
    params.R = eye(dim_z);       % Measurement noise matrix
end