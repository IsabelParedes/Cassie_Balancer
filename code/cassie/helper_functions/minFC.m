function fc = minFC(Gc, pseGc, Fga)

    u = 0.75/sqrt(2);   % fric coef
    
    
    x0 = pseGc*Fga;    % initial guess
    
        % x1 x2 x3 x4 x5 x6 x7 x8 x9 x10 x11 x12
    A = [  0  0 -1  0  0  0  0  0  0   0   0   0;   % Unilateral constraints Z
           0  0  0  0  0 -1  0  0  0   0   0   0;
           0  0  0  0  0  0  0  0 -1   0   0   0;
           0  0  0  0  0  0  0  0  0   0   0  -1;
           1  0 -u  0  0  0  0  0  0   0   0   0;   % Friction constraints X
          -1  0 -u  0  0  0  0  0  0   0   0   0;
           0  0  0  1  0 -u  0  0  0   0   0   0;   
           0  0  0 -1  0 -u  0  0  0   0   0   0;
           0  0  0  0  0  0  1  0 -u   0   0   0;   
           0  0  0  0  0  0 -1  0 -u   0   0   0;
           0  0  0  0  0  0  0  0  0   1   0  -u;   
           0  0  0  0  0  0  0  0  0  -1   0  -u;
           0  1 -u  0  0  0  0  0  0   0   0   0;   % Friction constraints Y
           0 -1 -u  0  0  0  0  0  0   0   0   0;
           0  0  0  0  1 -u  0  0  0   0   0   0;   
           0  0  0  0 -1 -u  0  0  0   0   0   0;
           0  0  0  0  0  0  0  1 -u   0   0   0;   
           0  0  0  0  0  0  0 -1 -u   0   0   0;
           0  0  0  0  0  0  0  0  0   0   1  -u;   
           0  0  0  0  0  0  0  0  0   0  -1  -u];
       
    b = zeros(20,1);
    
    % Do we need to optimize?
    
    need = sum(A*x0 <= b);
    
    if need == 20
        fc = x0;
    else
        Aeq = [];
        beq = [];
        lb = [];
        ub = [];
        options = optimoptions(@lsqlin, 'Display', 'off', 'MaxIterations', 100);

        [fc, ~] = lsqlin(Gc, Fga, A, b, Aeq, beq, lb, ub, x0, options);
    end