function params = studentParams(model)
    % define any parameters here 
    % params - struct
    global control
    
    control = 'PD';
    
    params.control = control;

    
    params.bestPos =  [     0
                            0
                          0.6
                            0
                            0
                            0
                            0
                            0
                            0
                            0
                       1.0197
                       1.0197
                      -2.2525
                      -2.2525
                            0
                            0
                       2.4794
                       2.4794
                      -2.1195
                      -2.1195];
                  
                  
    

