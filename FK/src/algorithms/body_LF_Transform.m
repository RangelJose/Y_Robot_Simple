function [T,TCoM,PosCM] = body_LF_Transform (nb, bodyLFinPF, qAxes, q, parent,CoM,m,T_Mass)
%Each element in T{i} is the transformation matrix from the inertial frame to
%the beginning of body i after the variation of q(i)

T = cell(nb,1);
TCoM = cell(nb,1);
PosCM = 0;
T{1} = bodyLFinPF{1}*HT(qAxes(1), q(1));
TCoM{1} = T{1}*CoM{1};

for i = 2:nb
    T{i} = T{parent(i)}*bodyLFinPF{i}*HT(qAxes(i), q(i));
    TCoM{i} = T{i}*CoM{i};
    PosCM = PosCM +  TCoM{i}(1:3,4)*m{i};
end
PosCM=PosCM/T_Mass;

end