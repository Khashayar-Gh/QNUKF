function [state, params] = OCEKF_update(state, params, z , dim_z , P_I)
    % Update step of the UKF
    % Inputs:
    %   state - structure containing the current state of the UKF
    %   params - structure containing UKF parameters
    %   z - measurement vector

    % Compute measurement sigma points
    zp = measurement_model(state.x, P_I);
    params.dim_z = dim_z;
    P = state.P;
    x = state.x;
    H = cal_H(x , state.last_pred_x , dim_z , P_I);
    Pz = H*P*H' + params.R;
%     Pz = symmetrizeCovariance(Pz);
    % Compute cross-covariance
    Pxz = P*H';

    % Kalman gain
    K = Pxz / Pz;

    % Update state
    z = reshape(z , [] , 1);
    zp = reshape(zp , [] , 1);
    state.x = xPlusU_ekf(state.x , K * (z - zp) , params);
%     state.x = xPlusU(state.x , K * (z - zp) , params);
%     state.P = state.P - K * Pz * K';
    state.P = state.P - K*H*state.P;
    state.P = symmetrizeCovariance(state.P);


end

function H = cal_H(x ,last_pred_x, point_count , P_I)
    Rhat = quat2rotm(x(1:4)');
    phat = x(14:16 , :); 
    Rhat_last = quat2rotm(last_pred_x(1:4)');
    phat_last = last_pred_x(14:16 , :); 
    H = zeros(3*point_count , 15);
    g = -9.81*[0 ; 0 ; 1];
    for i=1:point_count

%         H(3*i-2:3*i , 1:3) = -Rhat'*Skew(P_I(: , i)-phat);
% %         H(3*i-2:3*i , 1:3) = -Rhat'*Skew(-phat);
%         H(3*i-2:3*i , 4:6) = -Rhat';
%         H(3*i-2:3*i , :) = [Skew(Rhat'*(P_I(: , i)-phat)) ,...
%             zeros(3 , 9) , -Rhat'];
%         H(3*i-2:3*i , :) = [-Rhat'*Skew(P_I(: , i)-phat) ,...
%             zeros(3 , 9) , -Rhat'];

        w = zeros(3 , 1);
        u = [Rhat_last'*g ; Skew(P_I(: , i)-phat_last)*g];
%         %Au = w
%         A = w/u;
        A = [Skew(Rhat'*(P_I(: , i)-phat)), -Rhat'];
%         A = [-Rhat'*Skew(P_I(: , i)-phat), -Rhat'];
        A_star = solve_Auw(A , u , w);
        H(3*i-2:3*i , :) = [A_star(: , 1:3) ,...
            zeros(3 , 9) , A_star(: , 4:6)];


    end
end



function z_pred = measurement_model(sigma_pt, P_I)
    q = sigma_pt(1:4);
    p = sigma_pt(14:16 , :);
    R = quat2rotm(q');
%     z_pred = reshape(R'*(P_I - p) , [] , 1);
    z_pred = R'*(P_I - p);
end

