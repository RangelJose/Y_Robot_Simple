function R = Rot(axis, angle)
switch axis
    case 1
        R = RotX(angle);
    case 2
        R = RotY(angle);
    case 3
        R = RotZ(angle);
end
end