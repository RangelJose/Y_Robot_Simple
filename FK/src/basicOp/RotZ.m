function T = RotZ(angle)
C = cos(angle);
S = sin(angle);
T = [C -S  0  0; 
     S  C  0  0;
     0  0  1  0;
     0  0  0  1];
end