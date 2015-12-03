function [E1,E2,E3] = Conventions(Index,Matrix)
%   Conventions
%       Case 1  = XYZ
%       Case 2  = XZY
%       Case 3  = XYX
%       Case 4  = XZX
%       Case 5  = YXZ
%       Case 6  = YZX
%       Case 7  = YXY
%       Case 8  = YZY
%       Case 9  = ZXY
%       Case 10 = ZYX
%       Case 11 = ZXZ
%       Case 12 = ZYZ
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
switch Index
    case 1 %XYZ
       E1 = atan2(-Matrix(2,3),Matrix(3,3));
       E2 = atan2(Matrix(1,3),sqrt(Matrix(3,3)^2+Matrix(2,3)^2));
       E3 = atan2(-Matrix(1,2),Matrix(1,1));
    case 2 %XZY
       E1 = atan2(Matrix(3,2),Matrix(2,2));
       E2 = atan2(-Matrix(1,2),sqrt(Matrix(3,2)^2+Matrix(2,2)^2));
       E3 = atan2(-Matrix(1,3),Matrix(1,1));
    case 3 %XYX
       E1 = atan2(Matrix(2,1),-Matrix(3,1));
       E2 = atan2(sqrt(Matrix(2,1)^2+Matrix(3,1)^2),Matrix(1,1));
       E3 = atan2(Matrix(1,2),Matrix(1,3));
    case 4 %XZX
       E1 = atan2(Matrix(3,1),Matrix(2,1));
       E2 = atan2(sqrt(Matrix(3,1)^2+Matrix(2,1)^2),Matrix(1,1));
       E3 = atan2(Matrix(1,3),-Matrix(1,2));
    case 5 %YXZ
       E1 = atan2(Matrix(1,3),Matrix(3,3));
       E2 = atan2(Matrix(2,3),sqrt(Matrix(1,3)^2+Matrix(3,3)^2));
       E3 = atan2(Matrix(2,1),Matrix(2,2));
    case 6 %YZX
       E1 = atan2(-Matrix(3,1),Matrix(1,1));
       E2 = atan2(Matrix(2,1),sqrt(Matrix(2,3)^2+Matrix(2,2)^2));
       E3 = atan2(-Matrix(2,3),Matrix(2,2));
    case 7 %YXY
       E1 = atan2(-Matrix(3,1),Matrix(1,1));
       E2 = atan2(Matrix(2,1),sqrt(Matrix(2,3)^2+Matrix(2,2)^2));
       E3 = atan2(-Matrix(2,3),Matrix(2,2));
    case 8 %YZY
       E1 = atan2(Matrix(3,2),-Matrix(1,2));
       E2 = atan2(sqrt(Matrix(3,2)^2+Matrix(1,2)^2),Matrix(2,2)); 
       E3 = atan2(-Matrix(2,3),Matrix(2,1));
    case 9 %ZXY
       E1 = atan2(-Matrix(1,2),Matrix(2,2));
       E2 = atan2(Matrix(3,2),sqrt(Matrix(2,2)^2+Matrix(1,2)^2)); 
       E3 = atan2(-Matrix(3,1),Matrix(3,3));
    case 10 %ZYX
       E1 = atan2(Matrix(2,1),Matrix(1,1));
       E2 = atan2(-Matrix(3,1),sqrt(Matrix(2,1)^2+Matrix(1,1)^2)); 
       E3 = atan2(Matrix(3,2),Matrix(3,3));
    case 11 %ZXZ
       E1 = atan2(Matrix(1,3),-Matrix(2,3));
       E2 = atan2(sqrt(Matrix(2,3)^2+Matrix(1,3)^2),Matrix(3,3)); 
       E3 = atan2(Matrix(3,1),Matrix(3,2));
    case 12 %ZYZ
       E1 = atan2(Matrix(2,3),Matrix(1,3));
       E2 = atan2(sqrt(Matrix(1,3)^2 + Matrix(2,3)^2),Matrix(3,3));
       E3 = atan2(Matrix(3,2),-Matrix(3,1)); %%%CHECAR TODO OTRA VEZ POR SI LAS DUDAS
end
end
