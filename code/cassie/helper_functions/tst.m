load('cassie_model.mat') ;

syms x y z roll pitch yaw lhipab rhipab lhiprot rhiprot lhipflex rhipflex lknee rknee lkneespr rkneespr lankle rankle ltoe rtoe

q = [x y z roll pitch yaw lhipab rhipab lhiprot rhiprot lhipflex rhipflex lknee rknee lkneespr rkneespr lankle rankle ltoe rtoe]';

X_foot1 = bodypos(model, model.idx.foot1, q) ;
X_foot2 = bodypos(model, model.idx.foot2, q) ;

leftFront = X_to_r(xlt(model.p1)*X_foot1);
leftRear = X_to_r(xlt(model.p2)*X_foot1);
rightFront = X_to_r(xlt(model.p1)*X_foot2);
rightRear = X_to_r(xlt(model.p2)*X_foot2);

function X = bodypos(model, b, q, X)
    if(nargin < 4)
        X = eye(6) ;
    end
    while b > 0
        XJ = jcalc(model.jtype{b}, q(b)) ;
        X = X * XJ * model.Xtree{b} ;
        b = model.parent(b) ;
    end
end

