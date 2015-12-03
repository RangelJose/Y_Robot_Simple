function R = Rz(angle)
C = cos(angle);
S = sin(angle);
R = [C -S  0; 
     S  C  0;
     0  0  1];
end