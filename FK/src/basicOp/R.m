function Rot = R(axis, angle)
switch axis
    case 1
        Rot = Rx(angle);
    case 2
        Rot = Ry(angle);
    case 3
        Rot = Rz(angle);
end
end