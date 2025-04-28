function [state, params] = initialize_pf(dim_x, dim_z, dt, dim_nx , dim_nw, ...
    m_p ,q_indices, P_q_indices , x0 , P0)
    % Initialize the UKF parameters and state
    % Input:
    %   dim_x - dimension of the state vector
    %   dim_z - dimension of the measurement vector
    %   dt - time step
    %   alpha, beta, kappa - UKF tuning parameters
    %   sigma_mode - mode of sigma point generation
    %   augmentation - boolean flag for augmentation
    %   dim_xa - dimension of the augmented state vector
    %   xa - augmented state vector
    %   q_indices - indices of quaternions in the state vector
    %   P_q_indices - indices of quaternions in the process noise matrix



    % Initialize parameters
    params.dim_x = dim_x;
    params.dim_z = dim_z;
    params.dt = dt;
    params.q_indices = q_indices;
    params.P_q_indices = P_q_indices;
    params.dim_nx = dim_nx;
    params.dim_nw = dim_nw;
    params.m_p = m_p;
    params.x = x0;
    params.P = P0;

    % Initialize sigma points

    dim_P = dim_x - length(q_indices);
    state.dim_P = dim_P;

    % Initialize state
    params.Q = eye(dim_P);       % Process noise matrix
    params.Qa = eye(dim_nx);
    params.R = eye(dim_z);       % Measurement noise matrix
    state.particles = initilize_particles(params.x , params.P , params.m_p);
    params.w = repmat(1/m_p , 1 , m_p);
    params.m_thr = m_p/10;
end

function particles_q = initilize_particles(x , P , m_p)
    x_r = zeros(size(x , 1)-1 , 1);
    axang_ = quat2axang(x(1:4)')';
    x_r(1:3) = axang_(1:3)*axang_(4);
    x_r(4:end) = x(5:end);
    particles_q = zeros(m_p , size(x , 1));
%     particles_r = mvnrnd(x_r',P,m_p); %m_p by (m_x-1)
    nu = 1;
    particles_r = mvtrnd(x_r',P, nu, m_p);
%     axangs_ = [particles_r(: , 1:3)./vecnorm(particles_r(: , 1:3)')' , vecnorm(particles_r(: , 1:3)')'];
%     for i=1:m_p
%         particles_q(i , 1:4) = qPr(x(1:4) , particles_r(i , 1:3)');
%     end
%     particles_q(: , 1:4) = axang2quat(axangs_);
%     particles_q(: , 1:4) = rotm2quat(quat2rotm(particles_q(: , 1:4)));
    particles_q(: , 5:end) = particles_r(: , 4:end);

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

