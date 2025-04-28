function xx = xMinusU(x, U, params)
    % Subtracts vector U from state vector x with special handling for quaternions
    % x - state vector
    % U - vector to be subtracted
    % q_indices - indices of quaternion elements in x
    % P_q_indices - indices of quaternion-related elements in U
    q_indices = params.q_indices;
    P_q_indices = params.P_q_indices;
    xx = zeros(size(x));

    % Create masks for standard and quaternion indices
    mask_x = true(size(x));
    mask_U = true(size(U));

    % Update masks if indices are not empty
    if ~isempty(q_indices) && ~isempty(P_q_indices)
        mask_x(vertcat(q_indices{:})) = false;
        mask_U(vertcat(P_q_indices{:})) = false;
    end

    
    % Standard subtraction for non-quaternion elements
    xx(mask_x) = x(mask_x) - U(mask_U);

    % Quaternion subtraction for quaternion elements
    for i = 1:length(q_indices)
        xx(q_indices{i}) = qMr(x(q_indices{i}), U(P_q_indices{i})); % Define or replace qMr as needed
    end
end

function q_sub = qMr(q1, r)
    % Subtracts a rotation vector r from a quaternion q1
    % q1 - Quaternion
    % r - Rotation vector

    % Placeholder logic - replace with actual implementation
    % Convert r to a quaternion, invert it, and then add it to q1
    if norm(r)==0
        q2 = [1 , 0, 0 , 0]';
    else
        q2 = axang2quat([r/norm(r) ; norm(r)]')'; % Define this conversion function
    end
    q2_inv = quatinv(q2')'; % Define quaternion inversion function
%     q_sub = qPq(q1, q2_inv); % Reuse quaternion addition function
    q_sub = qPq(q2_inv, q1);
end