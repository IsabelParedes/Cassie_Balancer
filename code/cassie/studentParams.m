function params = studentParams(model)
    % define any parameters here 
    % params - struct
    global oldTau oldTaud oldT
    
    oldTau = zeros(10,1);  % Initial torque value
    oldTaud = zeros(10,1); % Initial torque change per time
    oldT = 0;              % Initial time

    params.tau = oldTau;
    params.taud = oldTaud;
    params.t = oldT;
