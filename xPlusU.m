function xx = xPlusU(x, U, params)
    % Adds vector U to state vector x with special handling for quaternions
    % x - state vector
    % U - vector to be added
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

    % Standard addition for non-quaternion elements
    xx(mask_x) = x(mask_x) + U(mask_U);

    % Quaternion addition for quaternion elements
    for i = 1:length(q_indices)
        xx(q_indices{i}) = qPr(x(q_indices{i}), U(P_q_indices{i})); % Define or replace qPr as needed
    end
end

function q_sum = qPr(q1, r)
    % Adds a quaternion q1 and a rotation vector r
    % q1 - Quaternion
    % r - Rotation vector

    % Placeholder logic - replace with actual implementation
    % Convert r to a quaternion and then add it to q1
    if norm(r)==0
        q2 = [1 , 0, 0 , 0]';
    else
        q2 = axang2quat([r/norm(r) ; norm(r)]')'; % Define this conversion function
    end
%     q_sum = qPq(q1, q2); % Define this addition function
    q_sum = qPq(q2, q1); % Define this addition function
end