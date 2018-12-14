function detect= DetectLargeDisturbance(q,dq,initcom)
%detect:boolean array to check for large disturbances in the
%left,right,forward and backward disturbances respectively
%initcom=[x;y;z];

detect=[false;false;false;false;false];
bound_forward=0.3;
bound_left=0.2;
bound_right=-0.4;
bound_backward=-0.3;
disp(q(2)-initcom(2));

%leftward disturbance
if((q(2)-initcom(2))>bound_left && dq(2)>0)
detect(1)=true;
end
%rightward disturbance
if((q(2)-initcom(2))<bound_right && dq(2)<0)
detect(2)=true;
end
%forward disturbance
if((q(1)-initcom(1))>bound_forward && dq(1)>0)
detect(3)=true;
end
%backward disturbance
if((q(1)-initcom(1))<bound_backward && dq(1)<0)
detect(4)=true;
end
end

