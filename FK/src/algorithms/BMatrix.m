function B = BMatrix(axis,E1,E2,E3,Damping)
EtaB=cell(3,1);
for i = 1:3
    switch axis(i)
        case 1
            EtaB{i} = [1;0;0];
        case 2
            EtaB{i} = [0;1;0];
        case 3
            EtaB{i} = [0;0;1];
    end
end
Rot{1} = R(axis(1),E1);
Rot{2} = Rot{1}*R(axis(2),E2);
Rot{3} = Rot{2}*R(axis(3),E3);

B=[Rot{1}*EtaB{1} Rot{2}*EtaB{2} Rot{3}*EtaB{3}];
B = B+(eye(3)*Damping);
end
