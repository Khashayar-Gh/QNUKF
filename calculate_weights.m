function [Wc, Wm] = calculate_weights(params)
    % Calculate the weights for the UKF
    % Inputs:
    %   params - structure containing UKF parameters such as alpha, beta, kappa, etc.

    dim = params.dim_xa - length(params.q_indices);
    alpha = params.alpha;
    beta = params.beta;
    kappa = params.kappa;

%     lambda = alpha^2 * (dim + kappa) - dim;
    lambda = 3-dim;
    Wm = zeros(2 * dim + 1, 1);
    Wc = zeros(2 * dim + 1, 1);

    Wm(1) = lambda / (dim + lambda);
    Wc(1) = Wm(1) + (1 - alpha^2 + beta);

    for i = 2:2*dim+1
        Wm(i) = 1 / (2 * (dim + lambda));
        Wc(i) = Wm(i);
    end
end