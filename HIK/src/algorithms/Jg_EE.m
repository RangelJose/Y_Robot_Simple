function [Jv,Jw] = Jg_EE(EEPos,T,LJoint,parent,axis)
%% Geometric Jacobian matrix
% Inputs:
%    EEPos = End effector position
%    T  = Homogeneus transformations of every link
%    LJoint = Parent link of the End-effector
%    parent = parent of every link
%    axis = axis where the joint rotate
%    EE = End-Effector
% Outputs:
%    Jv = Linear velocity Jacobian
%    Jw = Angular velocity Jacobian
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Setting up Constants
i = LJoint;         % i for the counter of while cycle
Jv = zeros(3,5);   % Size of the Jv and Jw 
Jw = zeros(3,5);
Jv(:,1) = [1;0;0];  % Setting up the first line of Jv and Jw which can't enter in the cycle because the parent is 0
Jw(:,1) = [0;0;0];


while i>1                   % Begining the while cycle
    switch axis(i)          % Obtaining the actual Eta Hat
        case 1
            Eta = [1;0;0];
            case 2
            Eta = [0;1;0];
            case 3
            Eta = [0;0;1];
            case 4
            Eta = [1;0;0];
            case 5
            Eta = [0;1;0];
            case 6
            Eta = [0;0;1];
    end
    
    EtaRow = T{parent(i)}(1:3,1:3)*Eta; % Obtaining the Eta Row
    O_f = EEPos - T{i}(1:3,4); %Acquisition of On-O's
    if axis(i)<=3 % Condition to Acquire the Jv and Jw
        Jv(:,i) = cross(EtaRow,O_f);
        Jw(:,i) = EtaRow;
    else
        Jv(:,i) = EtaRow;
        Jw(:,i) = [0;0;0];
    end
    i = parent(i);  % Giving another value to i
end
end