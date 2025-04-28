function [state, params] = initialize_ukf(dim_x, dim_z, dt, alpha, beta, kappa, sigma_mode, augmentation, dim_xa, xa, q_indices, P_q_indices)
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
    params.alpha = alpha;
    params.beta = beta;
    params.kappa = kappa;
    params.sigma_mode = sigma_mode;
    params.augmentation = augmentation;
    params.dim_x = dim_x;
    params.dim_xa = dim_xa;
    params.dim_z = dim_z;
    params.dt = dt;
    params.q_indices = q_indices;
    params.P_q_indices = P_q_indices;

    % Initialize sigma points
    if ~augmentation
        dim_P = dim_x - length(q_indices);
        state.sigma_pts = zeros(2 * dim_P + 1, dim_x); % sigma points
        state.sigma_pts_f = zeros(2 * dim_P + 1, dim_x); % transformed sigma points through f(x)
        state.sigma_pts_h = zeros(2 * dim_P + 1, dim_z); % transformed sigma points through h(x)
    else
        dim_P = dim_xa - length(q_indices);
        state.sigma_pts = zeros(2 * dim_P + 1, dim_xa); % sigma points
        state.sigma_pts_f = zeros(2 * dim_P + 1, dim_x); % transformed sigma points through f(x)
        state.sigma_pts_h = zeros(2 * dim_P + 1, dim_z); % transformed sigma points through h(x)
    end
    state.dim_P = dim_P;
    [params.Wc , params.Wm] = calculate_weights(params);

    % Initialize state
    state.x = zeros(dim_x, 1);  % State vector
    state.P = eye(dim_P);       % State covariance matrix
    params.Q = eye(dim_P);       % Process noise matrix
    params.Qa = eye(dim_xa-dim_x);
    params.R = eye(dim_z);       % Measurement noise matrix
    state.xa = xa;              % Augmented state vector
end