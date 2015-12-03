function T = RotY (angle)
C = cos(angle);
S = sin(angle);
T = [ C  0  S  0; 
      0  1  0  0;
     -S  0  C  0;
      0  0  0  1];
end