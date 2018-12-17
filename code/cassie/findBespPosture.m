clear; clc;

load('mat\cassie_model.mat') ;

bestP = findPosture(model);


function pos = findPosture(model)


    x0 = getInitialState(model);
    q0 = x0(1:model.n);             % 20x1
    [q1, q2, q3, q4] = computeFootPositions(q0, model);

    lb = -ones(4,1).*pi;
    ub = ones(4,1).*pi;
    
    lb(1) = 0.6;
    ub(1) = q0(3);
    
    ub(4) = 0;

    nvar = 4;   % z, hip pitch, knee, toe
    options = optimoptions('particleswarm', 'Display', 'iter');
    posX = particleswarm(@fun, nvar, lb, ub, options);
    
    % Reconstruct
    pos = q0;
    pos(3) = posX(1);                 % height
    pos(11:12) = [posX(2); posX(2)];  % hip pitch
    pos(13:14) = [posX(3); posX(3)];  % knee joint
    pos(17) = deg2rad(13) - pos(13);  % ankle joint
    pos(18) = deg2rad(13) - pos(14);
    pos(19:20) = [posX(4); posX(4)];  % toe joint
    
    
    function val = fun(x)

        height = x(1);
        
        % Reconstruct
        q = q0;
        q(3) = x(1);                  % height
        q(11:12) = [x(2); x(2)];      % hip pitch
        q(13:14) = [x(3); x(3)];      % knee joint
        q(17) = deg2rad(13) - q(13);  % ankle joint
        q(18) = deg2rad(13) - q(14);
        q(19:20) = [x(4); x(4)];      % toe joint

        [p1, p2, p3, p4] = computeFootPositions(q, model);
        
        val = [height;
               p1 - q1;
               p2 - q2; 
               p3 - q3;
               p4 - q4;
               p1(3);
               p2(3);
               p3(3);
               p4(3)];
        
        val = sum(val.^2);
        
    end

    disp('Helo')

end