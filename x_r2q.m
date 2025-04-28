function x_q = x_r2q(x_r)
    x_q = zeros(16, 1);
    angle = norm(x_r(1:3));
    
    if angle ~= 0
        axis = x_r(1:3) / angle;
        axang_ = [axis; angle];
        x_q(1:4) = axang2quat(axang_')';
    else
        x_q(1:4) = [1; 0; 0; 0]; % Unit quaternion for zero rotation
    end
    
    x_q(5:16) = x_r(4:15);
end