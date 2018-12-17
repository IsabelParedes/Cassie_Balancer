function fc = minFC2(Gc, pseGc, Fga)

    u = 0.75/sqrt(2);   % fric coef
    
    
    x0 = pseGc*Fga;    % initial guess
    
        % x1 x2 x3 x4 x5 x6 x7 x8 x9 x10 x11 x12
    A = [  1  0 -u  0  0  0  0  0  0   0   0   0;   % Friction constraints X
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
       
    b = zeros(16,1);
    
    
    x = optimvar('x', 12);
    
    % Objectives
    j1 = [eye(3), zeros(3)]*(Fga - Gc*x);
    j1 = j1(1)*j1(1) + j1(2)*j1(2) + j1(3)*j1(3);
    
    j2 = [zeros(3), eye(3)]*(Fga - Gc*x);
    j2 = j2(1)*j2(1) + j2(2)*j2(2) + j2(3)*j2(3);
    
    j3 = x'*x;
    
    a1 = 3;
    a2 = 2;
    a3 = 1;
    
    obj = a1*j1 + a2*j2 + a3*j3;  % in decreasing importance
    
    % Create problem
    prob = optimproblem('Objective', obj);
    
    % Constraints
    prob.Constraints.cons1 = A*x <= b;

    
    % Optimizer
    problem = prob2struct(prob);
    problem.x0 = x0;
       
    [fc, ~] = quadprog(problem);
    
    