function q_sub = qMq(q1, q2)
    % Subtracts a rotation vector r from a quaternion q1
    % q1 - Quaternion
    % r - Rotation vector

    % Placeholder logic - replace with actual implementation
    % Convert r to a quaternion, invert it, and then add it to q1
%     q2 = rotvec(r); % Define this conversion function
    q2_inv = quatinv(q2')'; % Define quaternion inversion function
    q_sub = quat2axang(qPq(q1, q2_inv)); % Reuse quaternion addition function
%     q_sub = quat2axang(qPq(q2_inv , q1)); % Reuse quaternion addition function
    q_sub = q_sub(1:3)*q_sub(4);
end