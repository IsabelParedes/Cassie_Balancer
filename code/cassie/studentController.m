function tau = studentController(t, s, model, params)
    % Modify this code to calculate the joint torques
    % t - time
    % s - 20x1 state of the robot
    % model - struct containing robot properties
    % params - user defined parameters in studentParams.m
    % tau - 10x1 vector of joint torques

    % State vector components ID
    q = s(1 : model.n);
    dq = s(model.n+1 : 2*model.n);
    x0 = getInitialState(model);
    q0 = x0(1:model.n);

    %% Pick control

    control = 4;    % 1-3

    switch control
        case 1
            % [Control #1] zero control
            tau = zeros(10,1);

        case 2
            % [Control #2] High Gain Joint PD control on all actuated joints
            kp = 500;
            kd = 100;
            x0 = getInitialState(model);
            q0 = x0(1:model.n);
            tau = -kp*(q(model.actuated_idx)-q0(model.actuated_idx)) - kd*dq(model.actuated_idx);

        case 3
            % [Control #3] High Gain Joint PD with individual gains
            % Highest score with these gains is 93.0101
            kp = [1e3;      % abduction
                  1e3;
                  500;      % rotation
                  500;
                  800;      % flexion
                  800;
                  1e5;      % knee
                  1e5;
                  8e2;      % toe
                  8e2];

            kd = [100;      % abduction
                  100;
                  100;      % rotation
                  100;
                  100;      % flexion
                  100;
                  400;      % knee
                  400;
                  200;      % toe
                  200];

            x0 = getInitialState(model);
            q0 = x0(1:model.n);
            qErr = q(model.actuated_idx)-q0(model.actuated_idx);
            dqErr = dq(model.actuated_idx);
            tau = -kp.*qErr - kd.*dqErr;
            
        case 4
            % Force control
            wren = wrench_gen(q, dq, q0);
            
        otherwise
            warning('Control not recognized.')
    end
end


%% Generation of Stabilizing Wrench
function [wrench] = wrench_gen(q, dq, q0)
    m = 31; g = -9.81; kp_f = 200 ; kd_f = 20 ;

    %reaction force to correct translational position deviation
    fd_r = -kp_f*(q(1:3)-q0(1:3)) - kd_f*dq(1:3) ;  
    fd_GA = [0, 0, m*g]' + fd_r;  %total force component in the wrench
       
    eul = q(4:6)';  %Euler angle of the body (roll, pitch, yaw in a row vector)
    quat = eul2quat(eul,'XYZ');
    del = quat(1); eps = quat(2:4)';
    kp_t = 100;
    K_r = kp_t*eye(3);  
    %3x3 diagnoal matrix to represent stiffness of torsional spring
    %controlling roll pitch and yaw
    
    t_r = -2*(del*eye(3) + eps./norm(eps))*K_r*eps;
    
    R_wb = eul2rotm(eul,'XYZ');
    kd_t = 50;
    D_r = kd_t*eye(3);  %3x3 symmetric positive matrix to represent damping
    w = dq(4:6);  %small angle approximation  
    
    td_GA = R_wb*(t_r - D_r*(w - zeros(3,1))); 
    %R_wb is transformation matrix from body to world
    %w_d = zeros(3,1)
    
    wrench = [fd_GA; td_GA];
end
