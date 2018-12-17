function f_ext = ExternalForce(t, q,model,dq)
% F_pert 6x1 - roll, pitch, yaw, x,y,z
if t(end) < 0.2
    F_pert =       [0 40 0            0 0 0]';
else
    F_pert =       [0 0 0            0 0 0]';
end    
%Initial COM after a step is taken
% x0=getInitialState(model);
% initcom=compute_COM_pos(model, x0(1:model.NB))' ;
% detect=DetectLargeDisturbance(q(1:3),dq(1:3),initcom);
% disp(detect);
% apply perturbation force on torso
f_ext = cell(1,model.NB);
bXw_curr = bodypos(model, model.idx.torso, q) ;
f_ext{model.idx.torso} = bXw_curr' * F_pert;