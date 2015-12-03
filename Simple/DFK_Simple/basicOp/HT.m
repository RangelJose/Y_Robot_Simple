function T = HT(axis, q)
switch axis
    case 1
        T = RotX(q);
    case 2
        T = RotY(q);
    case 3
        T = RotZ(q);
    case 4
        T = Tras([q, 0, 0]);
    case 5
        T = Tras([0, q, 0]);
    case 6
        T = Tras([0, 0, q]);
end
end