function R = Ry (angle)
C = cos(angle);
S = sin(angle);
R = [ C  0  S; 
      0  1  0;
     -S  0  C];
end