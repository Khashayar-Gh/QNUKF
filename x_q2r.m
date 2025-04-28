function x_r = x_q2r(x_q)
    x_r = zeros(15 , 1);
    axang_ = quat2axang(x_q(1:4)')';
    x_r(1:3) = axang_(1:3)*axang_(4);
    x_r(4:15) = x_q(5:16);
end