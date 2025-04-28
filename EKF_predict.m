function [state, params] = EKF_predict(state, params  , omega , a , g , e3)
    % Perform UKF prediction step
    % Inputs:
    %   state - structure containing the current state of the UKF
    %   params - structure containing UKF parameters
    %   fx_args - additional arguments needed for the process function
    dT = params.dt;
    [Phi_k , Q_k] = FG(state.x , omega, a , params);
    state.x = dynamics(state.x,dT , omega , a , g , e3);
    state.P = Phi_k*state.P*Phi_k' + Q_k;
    state.P = symmetrizeCovariance(state.P);
    
end

function [Phi_k , Q_k] = FG(cur_state , omega_m, a_m , params)
    % n = [n_w , n_a , n_wb , n_ab]
    % x = [q , p , v , b_w , b_a]
    Rhat = quat2rotm(cur_state(1:4 , :)');
%     phat = cur_state(5:7 , :); 
%     vhat = cur_state(8:10 , :);    
    bw = cur_state(11:13);
    ba = cur_state(14:16);
    omega = omega_m - bw;
    a = a_m - ba;
    zeros3 = zeros(3 , 3);
    F_ = [-Skew(omega)  , zeros3 , zeros3 , -eye(3) , zeros3
        zeros3 , zeros3 , eye(3), zeros3 , zeros3
        -Rhat*Skew(a), zeros3 , zeros3, zeros3 , -Rhat
        zeros3 , zeros3 , zeros3 , zeros3 , zeros3
        zeros3 , zeros3 , zeros3 , zeros3 , zeros3];

    G_ = [-eye(3), zeros3 , zeros3 , zeros3
        zeros3 , zeros3 , zeros3 , zeros3
        zeros3 , -Rhat, zeros3 , zeros3
        zeros3 , zeros3 , eye(3), zeros3
        zeros3 , zeros3 , zeros3, eye(3)];
%       G_ = [-eye(3), zeros3
%         zeros3 , zeros3 
%         zeros3 , -Rhat
%         zeros3 , zeros3 
%         zeros3 , zeros3];
    
    dT = params.dt;
    Phi_k = expm(F_*dT);
%     Q_k = Phi_k*G_*Q*G_'*Phi_k'*dT;
    fun = @(tou) expm(F_*(dT - tou))*G_;
%     G_k = integral(fun,0,dT , 'ArrayValued',true);
%     Q_k = G_k*Q*G_k';
    Q = params.Q;
    Q(7:12 , 7:12) = Q(7:12 , 7:12)/(dT^2);
%     Q = Q*dT^2;
    fun2 = @(tou) fun(tou)*Q*fun(tou)';
    Q_k = integral(fun2,0,dT ,'ArrayValued',true);
%     Q_k(7:12 , 7:12) =  Q_k(7:12 , 7:12) + Q(7:12 , 7:12);


end

function next_state = dynamics(cur_state , dT , omega , a , g , e3)
    Rhat = quat2rotm(cur_state(1:4 , :)');
    phat = cur_state(5:7 , :); 
    vhat = cur_state(8:10 , :);    
    bw = cur_state(11:13);
    ba = cur_state(14:16);
    bw_w = 0;
    ba_w = 0;
%     Rhat = Rhat*expm(dT*Skew(omega-bw - bw_w));
    A = [zeros(3) eye(3) zeros(3,1)
        zeros(3) zeros(3) -g*e3+Rhat*(a-ba-ba_w)
        zeros(1 , 7)];
    x = [phat; vhat; 1];
    x = expm(dT*A)*x;
    phat = x(1:3);
    vhat = x(4:6);
%     phat = phat + vhat*dT;
%     vhat = vhat + (-g*e3+Rhat*(a-1*ba-ba_w))*dT;
%     Rhat = Rhat*expm(dT*Skew(omega-bw - bw_w));
%     next_state = [rotm2quat(Rhat)' ;phat ;vhat;bw;ba];
    qhat = cur_state(1:4 , :);
%     qhat = rotm2quat(quat2rotm(qhat'))';
    omega_ = omega-bw - bw_w;
    qhat = expm(dT*1/2*[0 , -omega_';omega_ , -Skew(omega_)])*qhat;
    qhat = rotm2quat(quat2rotm(qhat'))';
    qhat = qhat/norm(qhat);
    next_state = [qhat ;phat ;vhat;bw;ba];
end