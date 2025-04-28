function q_sub_q = qMq_q(q1, q2)
    % Subtracts a rotation vector r from a quaternion q1
    % q1 - Quaternion
    % r - Rotation vector

    % Placeholder logic - replace with actual implementation
    % Convert r to a quaternion, invert it, and then add it to q1
%     q2 = rotvec(r); % Define this conversion function
    q2_inv = quatinv(q2')'; % Define quaternion inversion function
    q_sub_q = qPq(q1, q2_inv)'; % Reuse quaternion addition function
%     if q_sub_q(1)<0
%         q_sub_q = q_sub_q*-1;
%     end
%     disp(size(q_sub_q))
    q_sub_q = rotm2quat(quat2rotm(q_sub_q'));
end