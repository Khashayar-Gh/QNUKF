function q_sum = qPq(q1 , q2)
    q_sum = quatnormalize(quatmultiply(q1' , q2'));
%     q_sum = quatnormalize(quatmultiply(q2' , q1'));
    q_sum = rotm2quat(quat2rotm(q_sum));
end