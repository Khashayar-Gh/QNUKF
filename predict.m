function [state, params] = predict(state, params  , omega , a , g , e3)
    % Perform UKF prediction step
    % Inputs:
    %   state - structure containing the current state of the UKF
    %   params - structure containing UKF parameters
    %   fx_args - additional arguments needed for the process function
    dT = params.dt;
    state.sigma_pts = update_sigma_pts(state, params);

%     % Update sigma points
%     state.sigma_pts = update_sigma_pts(state, params);

    % Process each sigma point through the process model
    for i = 1:size(state.sigma_pts, 1)
        state.sigma_pts_f(i, :) = dynamics(state.sigma_pts(i, :)',dT , omega , a , g , e3)';
    end

    % Compute the predicted state mean and covariance
    [state.x, state.P] = compute_state_mean_cov(state.sigma_pts_f, params);
end

function next_state = dynamics(cur_state , dT , omega , a , g , e3)
    Rhat = quat2rotm(cur_state(1:4 , :)');
    phat = cur_state(5:7 , :); 
    vhat = cur_state(8:10 , :);    
    bw = cur_state(11:13);
    ba = cur_state(14:16);
    bw_w = cur_state(17:19);
    ba_w = cur_state(20:22);
%     Rhat = Rhat*expm(dT*Skew(omega-bw - bw_w));
    A = [zeros(3) eye(3) zeros(3,1)
        zeros(3) zeros(3) -g*e3+Rhat*(a-ba-ba_w)
        zeros(1 , 7)];
    x = [phat; vhat; 1];
    x = expm(dT*A)*x;
    phat = x(1:3);
    vhat = x(4:6);
%     phat = phat + vhat*dT;
%     vhat = vhat + (-g*e3+Rhat*(a-1*ba-ba_w))*dT;
%     Rhat = Rhat*expm(dT*Skew(omega-bw - bw_w));
%     next_state = [rotm2quat(Rhat)' ;phat ;vhat;bw;ba];
    qhat = cur_state(1:4 , :);
%     qhat = rotm2quat(quat2rotm(qhat'))';
    omega_ = omega-bw - bw_w;
    qhat = expm(dT*1/2*[0 , -omega_';omega_ , -Skew(omega_)])*qhat;
    qhat = qhat/norm(qhat);
    qhat = rotm2quat(quat2rotm(qhat'))';
    next_state = [qhat ;phat ;vhat;bw;ba];
end

function [x_pred, P_pred] = compute_state_mean_cov(sigma_pts_f, params)
    % Compute the predicted state mean and covariance from transformed sigma points
    % Inputs:
    %   sigma_pts_f - Transformed sigma points
    %   params - UKF parameters

    [n_sigmas, n] = size(sigma_pts_f);
    n_p = n - length(params.q_indices);
    
    x_pred = zeros(n, 1);
    P_pred = zeros(n_p, n_p);
    Wm = params.Wm;
    Wc = params.Wc;
    Q = params.Q; % Assuming M is the process noise matrix Q
    
    % Compute mean for quaternion elements
    for i = 1:numel(params.q_indices)
        q_index = params.q_indices{i};
%         x_pred(q_index) = QWA(Wm, sigma_pts_f(:, q_index));
        x_pred(q_index) = wavg_quaternion_markley(sigma_pts_f(:, q_index), Wm);
    end

    % Compute mean for non-quaternion elements
    mask_mean = true(n, 1);
    mask_mean(vertcat(params.q_indices{:})) = false;
%     x_pred(mask_mean) = sigma_pts_f(:, mask_mean) * Wm;
    x_pred(mask_mean) = Wm'*sigma_pts_f(:, mask_mean);


    % Adjust n to exclude quaternion indices
%     n = n - length(vertcat(params.q_indices{:}));
    P_pred = P_pred + Q; % Add process noise covariance

    % Compute covariance
    for k = 1:n_sigmas
        y = xMinusx(sigma_pts_f(k, :)' , x_pred , params);
        P_pred = P_pred + Wc(k) * (y * y');
    end
    P_pred = symmetrizeCovariance(P_pred);
end

function q_avg = QWA(W, qs)
    % Quaternion Weighted Average
    M = zeros(4, 4);
    for i = 1:size(qs, 2)
        M = M + W(i) * (qs(i, :)' * qs(i, :));
    end
    [eigenvectors, eigenvalues] = eig(M);
    [~, max_index] = max(abs(diag(eigenvalues)));
    q_avg = quatnormalize(eigenvectors(:, max_index)')';
    q_avg = rotm2quat(quat2rotm(q_avg));
end

function sigma_pts = update_sigma_pts(state, params)
    % Update the sigma points during the prediction stage
    % Inputs:
    %   state - structure containing the state of the UKF
    %   params - structure containing UKF parameters

    if ~params.augmentation
        dim = params.dim_x - length(params.q_indices);
        x = state.x;
        P = state.P;
    else
        dim = params.dim_xa - length(params.q_indices);
        x = [state.x; state.xa]; % Concatenate state and augmented state
        P = blkdiag(state.P, params.Qa); % Block diagonal for augmented covariance
    end

    if params.sigma_mode == 1
        lambda = params.alpha^2 * (dim + params.kappa) - dim;
    elseif params.sigma_mode == 2
        lambda = 3 - dim;
    else
        error('Invalid sigma_mode');
    end

    % U = chol((dim + lambda) * P, 'lower');
    U = pseudo_sqrt((dim + lambda) * P); % Custom function for pseudo square root
%     disp(U)
    sigma_pts = zeros(2 * dim + 1, length(x));
    sigma_pts(1, :) = x;
    
    for k = 1:dim
        sigma_pts(k+1 , :) = xPlusU(x, U(:, k) , params); % Custom function for adding U to x
%         sigma_pts(dim + k+1, :) = xMinusU(x, U(:, k) , params); % Custom function for subtracting U from x
        sigma_pts(dim + k+1, :) = xPlusU(x, -U(:, k) , params);
    end
end

function sqrt_matrix = pseudo_sqrt(A)
    % Try Cholesky decomposition first
    [R, p] = chol(A , 'lower');
    p = 1;
    if p == 0
        sqrt_matrix = R;
    else
        % Fallback to SVD if Cholesky fails
        [U, S, V] = svd(A);
        sqrt_S = sqrt(S);
        sqrt_matrix = U * sqrt_S * V';
    end
end

% by Tolga Birdal
% Q is an Mx4 matrix of quaternions. weights is an Mx1 vector, a weight for
% each quaternion.
% Qavg is the weightedaverage quaternion
% This function is especially useful for example when clustering poses
% after a matching process. In such cases a form of weighting per rotation
% is available (e.g. number of votes), which can guide the trust towards a
% specific pose. weights might then be interpreted as the vector of votes 
% per pose.
% Markley, F. Landis, Yang Cheng, John Lucas Crassidis, and Yaakov Oshman. 
% "Averaging quaternions." Journal of Guidance, Control, and Dynamics 30, 
% no. 4 (2007): 1193-1197.
function [Qavg]=wavg_quaternion_markley(Q, weights)
% Form the symmetric accumulator matrix
A=zeros(4,4);
M=size(Q,1);
wSum = 0;
for i=1:M
    q = Q(i,:)';
    if(q(1)<0) % handle the antipodal configuration
	    q = -q;
    end
    w_i = weights(i);
    A=w_i.*(q*q')+A; % rank 1 update
    wSum = wSum + w_i;
end
% scale
A=(1.0/wSum)*A;
% Get the eigenvector corresponding to largest eigen value
[Qavg, ~]=eigs(A,1);
Qavg = rotm2quat(quat2rotm(Qavg'))';
end

