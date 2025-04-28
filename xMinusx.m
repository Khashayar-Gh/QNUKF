function xx = xMinusx(x1, x2, params)
    % Subtracts state vector x2 from state vector x1 with special handling for quaternions
    % x1, x2 - state vectors
    % q_indices - indices of quaternion elements in x
    % P_q_indices - indices corresponding to quaternion elements in the output
    q_indices = params.q_indices;
    P_q_indices = params.P_q_indices;
    n = length(x1) - length(q_indices);
    xx = zeros(n, 1);

    % Create masks for standard and quaternion indices
    mask_x = true(length(x1), 1);
    mask_xx = true(n, 1);

    % Update masks based on quaternion indices
    if ~isempty(q_indices) && ~isempty(P_q_indices)
        mask_x([q_indices{:}]) = false;
        mask_xx([P_q_indices{:}]) = false;
    end

    % Standard subtraction for non-quaternion elements
    xx(mask_xx) = x1(mask_x) - x2(mask_x);

    % Quaternion subtraction for quaternion elements
    for i = 1:length(q_indices)
        xx(P_q_indices{i}) = qMq(x1(q_indices{i}), x2(q_indices{i})); % Define or replace qMq as needed
    end
end

% function q_sub = qMq(q1, q2)
%     % Subtracts a rotation vector r from a quaternion q1
%     % q1 - Quaternion
%     % r - Rotation vector
% 
%     % Placeholder logic - replace with actual implementation
%     % Convert r to a quaternion, invert it, and then add it to q1
% %     q2 = rotvec(r); % Define this conversion function
%     q2_inv = quatinv(q2')'; % Define quaternion inversion function
%     q_sub = quat2axang(qPq(q1, q2_inv)); % Reuse quaternion addition function
% %     q_sub = quat2axang(qPq(q2_inv , q1)); % Reuse quaternion addition function
%     q_sub = q_sub(1:3)*q_sub(4);
% end