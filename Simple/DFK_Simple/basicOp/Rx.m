function R = Rx (angle)
C = cos(angle);
S = sin(angle);
R = [1  0  0;
     0  C -S; 
     0  S  C];
end