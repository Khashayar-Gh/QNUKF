function [state, params] = update(state, params, z , dim_z , P_I)
    % Update step of the UKF
    % Inputs:
    %   state - structure containing the current state of the UKF
    %   params - structure containing UKF parameters
    %   z - measurement vector

    % Compute measurement sigma points
    params.dim_z = dim_z;
    state.sigma_pts_h = zeros(2 * state.dim_P + 1, 3*dim_z);
    for i = 1:size(state.sigma_pts_f, 1)
        state.sigma_pts_h(i, :) = measurement_model(state.sigma_pts_f(i, :), P_I); % Define measurement_model
    end

    % Compute the predicted measurement and its covariance
    [zp, Pz] = compute_z_mean_covariance(state.sigma_pts_h, params); % Define compute_measurement_mean_cov

    % Compute cross-covariance
    Pxz = compute_cross_covariance(state.sigma_pts_f, state.sigma_pts_h, state.x, zp, params); % Define compute_cross_covariance

    % Kalman gain
    K = Pxz / Pz;

    % Update state
    z = reshape(z , [] , 1);
    state.x = xPlusU(state.x , K * (z - zp) , params);
    state.P = state.P - K * Pz * K';
    state.P = symmetrizeCovariance(state.P);


end

function Pxz = compute_cross_covariance(sigma_pts_f, sigma_pts_h, x, zp, params)
    % Compute the cross-covariance matrix
    % Inputs:
    %   sigma_pts_f - Transformed sigma points through the process function
    %   sigma_pts_h - Transformed sigma points through the measurement function
    %   x - Current state estimate
    %   zp - Predicted measurement
    %   params - UKF parameters

    n_sigmas = size(sigma_pts_f, 1);
    dim_x = params.dim_x - length(params.q_indices);
    dim_z = size(sigma_pts_h , 2);
    Wc = params.Wc;

    % Initialize cross-covariance matrix
    Pxz = zeros(dim_x, dim_z);

    % Compute cross-covariance
    for i = 1:n_sigmas
        x_r = xMinusx(sigma_pts_f(i, :)', x, params);
        z_r = sigma_pts_h(i, :)' - zp;
        Pxz = Pxz + Wc(i) * (x_r * z_r');
    end
end

function z_pred = measurement_model(sigma_pt, P_I)
    q = sigma_pt(1:4);
    p = sigma_pt(5:7)';
    R = quat2rotm(q);
    z_pred = reshape(R'*(P_I - p) , [] , 1);
end

function [zp, Pz] = compute_z_mean_covariance(sigma_pts, params)
    % Calculate the mean and covariance of the transformed sigma points
    % Inputs:
    %   sigma_pts - Sigma points transformed by the process or the measurement
    %   params - UKF parameters containing weights Wm and Wc
    %   M - Process or measurement noise matrix (Q or R)

    n_sigmas = size(sigma_pts, 1);
    n = size(sigma_pts, 2);
    Wm = params.Wm;
    Wc = params.Wc;

    % Compute mean of the sigma points
    zp = sigma_pts' * Wm;

    % Initialize covariance matrix
    Pz = zeros(n, n);
    Pz = Pz + params.R; % Add measurement noise covariance

    % Compute covariance of the sigma points
    for k = 1:n_sigmas
        y = sigma_pts(k, :)' - zp;
        Pz = Pz + Wc(k) * (y * y');
    end
    Pz = symmetrizeCovariance(Pz);
end
