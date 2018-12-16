function fc = minFC(Gc, pseGc, Fga)

    mu = 0.75;
    x0 = pseGc*Fga;    % initial guess
    
    x = optimvar('x', 12);
    
    obj = sum((Fga - Gc*x).^2);
    prob = optimproblem('Objective', obj);
    
    % Fz must be positive
    prob.Constraints.unilatZ = [x(3), x(6), x(9), x(12)] >=0;

    % Friction constraints for X
    prob.Constraints.fricX1  = x(1) <= mu*x(3)/sqrt(2);
    prob.Constraints.fricX1n = x(1) >= -mu*x(3)/sqrt(2);
    prob.Constraints.fricX2  = x(4) <= mu*x(6)/sqrt(2);
    prob.Constraints.fricX2n = x(4) >= -mu*x(6)/sqrt(2);
    prob.Constraints.fricX3  = x(7) <= mu*x(9)/sqrt(2);
    prob.Constraints.fricX3n = x(7) >= -mu*x(9)/sqrt(2);
    prob.Constraints.fricX4  = x(10) <= mu*x(12)/sqrt(2);
    prob.Constraints.fricX4n = x(10) >= -mu*x(12)/sqrt(2);
    
    % Frictions constraints for Y
    prob.Constraints.fricY1  = x(2) <= mu*x(3)/sqrt(2);
    prob.Constraints.fricY1n = x(2) >= -mu*x(3)/sqrt(2);
    prob.Constraints.fricY2  = x(5) <= mu*x(6)/sqrt(2);
    prob.Constraints.fricY2n = x(5) >= -mu*x(6)/sqrt(2);
    prob.Constraints.fricY3  = x(8) <= mu*x(9)/sqrt(2);
    prob.Constraints.fricY3n = x(8) >= -mu*x(9)/sqrt(2);
    prob.Constraints.fricY4  = x(11) <= mu*x(12)/sqrt(2);
    prob.Constraints.fricY4n = x(11) >= -mu*x(12)/sqrt(2);
      
    problem = prob2struct(prob);
    problem.x0 = x0;
    
    options= optimoptions(@quadprog, 'Algorith', 'interior-point');
    
    [fc, ~] = quadprog(problem);
    
    