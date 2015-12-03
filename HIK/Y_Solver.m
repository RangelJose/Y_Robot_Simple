function dq = Y_Solver(J,Dx,P,G,JDmp,n)
%Initial Conditions
Pn = eye(n);
Dt = zeros(n,1);
%

%% Convergence loop
%
for i=1:P
Dx_h = Dx{i}-(G*J{i}*Dt);
J{i} = J{i}*Pn;
[Jtd,Jt] = JacPsInv(J{i},JDmp);
Dt = Dt+(Jtd*Dx_h);
Pn = Pn-(Jt*J{i});
end

dq=Dt;
end