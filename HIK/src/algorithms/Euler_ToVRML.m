function out=Euler_ToVRML(convention,q1,q2,q3)
    X=[1;0;0];
    Y=[0;0;-1];
    Z=[0;1;0];
    
switch convention
    case 1 %XYZ
        out=[X;q1;Y;q2;Z;q3];
    case 2 %XZY
        out=[X;q1;Z;q2;Y;q3];
    case 3 %XYX
        out=[X;q1;Y;q2;X;q3];
    case 4 %XZX
        out=[X;q1;Z;q2;X;q3];
    case 5 %YXZ
        out=[Y;q1;X;q2;Z;q3];
    case 6 %YZX
        out=[Y;q1;Z;q2;X;q3];
    case 7 %YXY
        out=[Y;q1;X;q2;Y;q3];
    case 8 %YZY
        out=[Y;q1;Z;q2;Y;q3];
    case 9 %ZXY
        out=[Z;q1;X;q2;Y;q3];
    case 10 %ZYX
        out=[Z;q1;Y;q2;X;q3];
    case 11 %ZXZ
        out=[Z;q1;X;q2;Z;q3];
    case 12 %ZYZ
        out=[Z;q1;Y;q2;Z;q3];
end
end
