function [leftFront, leftRear, rightFront, rightRear] = computeFootPositions(q, model)

X_foot1 = bodypos(model, model.idx.foot1, q) ;
X_foot2 = bodypos(model, model.idx.foot2, q) ;

leftFront = X_to_r(xlt(model.p1)*X_foot1);
leftRear = X_to_r(xlt(model.p2)*X_foot1);
rightFront = X_to_r(xlt(model.p1)*X_foot2);
rightRear = X_to_r(xlt(model.p2)*X_foot2);