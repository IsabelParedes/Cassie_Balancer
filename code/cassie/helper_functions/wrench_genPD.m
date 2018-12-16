function [wrench] = wrench_genPD(q, dq, q0)
    m = 31; 
    g = 9.81; 
    
    kp_f = 5; 
    kd_f = 1;
    kp_t = 100;
    kd_t = 50;

    %reaction force to correct translational position deviation
    fd_r = -kp_f*(q(1:3)-q0(1:3)) - kd_f*dq(1:3) ;  
    fd_GA = [0, 0, m*g]' + fd_r;  %total force component in the wrench
    
    w = dq(4:6);  %small angle approximation
    K_r = kp_t*eye(3);
    %3x3 diagnoal matrix to represent stiffness of torsional spring
    %controlling roll pitch and yaw
    D_r = kd_t*eye(3);  %3x3 symmetric positive matrix to represent damping
    td_GA = -K_r*q(4:6)-D_r*w;  %desired orientation and rate of change both are zero
       

    wrench = [fd_GA; td_GA];



    


