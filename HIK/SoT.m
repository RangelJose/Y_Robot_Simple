function [P,EE,Targets,G,DmpB,DmpJ] = SoT()
%% This file is the control of the stack of tasks
% Outputs = 
%       P = Number of priorities to accomplish
%       EE = End-Effector that will raise the Target
%       Targets = Cell with the targets
%       G = Gain of the system
%       DmpB = Damping of the Forward Differential Kinematics
%       DmpJ = Damping of the Pseudo-Inverse Jacobian
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Number of priorities
P = 2;
%%%%%%%%%%%%%%%%%%%%%%

%% Priority conditions
if P<3
    n=3;
elseif P>=3
    n=P;
end
%%%%%%%%%%%%%%%%%%%%%%%

%% Dampings and Gain
G = 1; %Gain of the system
DmpB = 0.000001; %Damping for the B Matrix
DmpJ = 0.000001;  %Damping for the Pseudo-Inverse Jacobian
%%%%%%%%%%%%%%%%%%%%%%%%

%% End-Effector to raise the Target
EE = cell(n,3); %%It contains 3 columns which are The end effector, the number of the end effector and the parent link of the end-effector respectively
% Write the end effector to taise the target

        %END EFFECTOR            %Restrictions   %Structure
        % 'RA' for Right Hand          6          [X;Y;Teta]
        % 'LA' for Left Hand           6                "
        % 'CM' fot Center of Mass      2          [X]
        
EE{1} = 'CM';
EE{3} = 'LH';
EE{2} = 'RH';

for i = 1:n
    switch EE{i}
        case 'RH'
            EE{i,2} = 1;
            EE{i,3} = 4;
        case 'LH'
            EE{i,2} = 2;
            EE{i,3} = 5;
        case 'CM'
            EE{i,2} = 3;
            EE{i,3} = 5;
    end
end %Assigning values to the end effector
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Targets to accomplish
Targets{1} = 25;
Targets{2} = [2.5;2;1];
Targets{3} = [-2.8284;6.8284;0];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end