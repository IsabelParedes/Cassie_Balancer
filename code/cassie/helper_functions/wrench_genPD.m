function [wrench] = wrench_genPD(q, dq, q0)
    m = 31; 
    g = -9.81; 
    
    kp_f = 5; 
    kd_f = 1;

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
    
    norEps = norm(eps);
    if norEps == 0 
        norEps = 1; 
    end
    
    t_r = -2*(del*eye(3) + eps./norEps)*K_r*eps;
    
    R_wb = eul2rotm(eul,'XYZ');
    kd_t = 50;
    D_r = kd_t*eye(3);  %3x3 symmetric positive matrix to represent damping
    w = dq(4:6);  %small angle approximation  
    
    td_GA = R_wb*(t_r - D_r*(w - zeros(3,1))); 
    %R_wb is transformation matrix from body to world
    %w_d = zeros(3,1)
    
    wrench = [fd_GA; td_GA];



    


