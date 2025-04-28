function P_sym = symmetrizeCovariance(P)
    % Symmetrize the covariance matrix P
    % Ensure P is a square matrix
    if size(P, 1) ~= size(P, 2)
        error('Input matrix P must be square.');
    end
    
    % Symmetrize P by averaging it with its transpose
    P_sym = 0.5 * (P + P');
end
