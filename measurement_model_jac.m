syms x [16 , 1]
syms P_I [3 , 1]
z_pred = measurement_model(x , P_I);
gradf = jacobian(z_pred , x)

function z_pred = measurement_model(sigma_pt, P_I)
    q = sigma_pt(1:4);
    p = sigma_pt(5:7)';
    R = quat2rotm(q);
    z_pred = reshape(R'*(P_I - p) , [] , 1);
end