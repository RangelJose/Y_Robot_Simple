function Y_Model()
%% Y Model
%In this section is where all the parameters of the robot are controlled
% NB is the number of DOF of the robot
% link is the number of the link of every joint
% axes is the axis where every joint moves
% 1= Rotation in X
% 2= Rotation in Y
% 3= Rotation in Z
% 4= Traslation in X
% 5= Traslation in Y
% 6= Traslation in Z
% parent is the parent of every joint, it means the transformation before
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
global srcLoaded Ymodel
if isempty(srcLoaded)
    addpath(genpath('../../src'));
    display('--> Folder src and subfolders added to the path')
    srcLoaded = true;
end

Ymodel.NB = 5;
Ymodel.NB_EE = 2;
Ymodel.link =   [ 1 2 3 4 5 ];%1to6 referential frame to Body frame, 7to9 Right Arm, 10to12 Left Arm, 13to18 Right Leg, 19to24 Left Leg
Ymodel.parent = [ 0 1 2 3 3 ];
Ymodel.axes =   [ 4 6 2 2 2 ];
Ymodel.EE_parent = [ 4 5 ];

%% Joint limits
Ymodel.jntVelLim = [ inf -inf; inf -inf; inf -inf; inf -inf; inf -inf; inf -inf;inf -inf; inf -inf; inf -inf; inf -inf; inf -inf; inf -inf;inf -inf; inf -inf; inf -inf; inf -inf; inf -inf; inf -inf;inf -inf; inf -inf; inf -inf; inf -inf; inf -inf; inf -inf];
Ymodel.jntLim = [ inf -inf; inf -inf; inf -inf; inf -inf; inf -inf; inf -inf;inf -inf; inf -inf; inf -inf; inf -inf; inf -inf; inf -inf;inf -inf; inf -inf; inf -inf; inf -inf; inf -inf; inf -inf;inf -inf; inf -inf; inf -inf; inf -inf; inf -inf; inf -inf];

%% Position of the referential frame of each body in the parent coordinates
%"Variable" distances between joints
Ymodel.LFinPF{1} = Tras([0,0,0]);
Ymodel.LFinPF{2} = Tras([0,0,0]);
Ymodel.LFinPF{3} = Tras([0,0,0]);
Ymodel.LFinPF{4} = Tras([0,0,2])*RotY(deg2rad(-45));
Ymodel.LFinPF{5} = Tras([0,0,2])*RotY(deg2rad(45));

%% Position of the referential frame of the end-effector in the body local frame
Ymodel.EEinLF{1} = Tras([0,0,2])*RotY(deg2rad(45));%End-effector Right hand
Ymodel.EEinLF{2} = Tras([0,0,2])*RotY(deg2rad(-45));%End-effector Left hand

%% Position of the wrist wrt the end-efector frame
%Ymodel.eePw = -Ymodel.EEinLF{1}(1:3,1:3).'*(Ymodel.EEinLF{1}(1:3,4) + [0; 0; l]);
%It hasn't been designed due to we are not using it, it has no changes from
%the original.
%% dynamic parameters
Ymodel.m{1}=0;
Ymodel.CoM{1} = Tras([0; 0; 0]);
Ymodel.I{1} = [0 0 0; 0 0 0; 0 0 0];

Ymodel.m{2}=0;
Ymodel.CoM{2} = Tras([0; 0; 0]);
Ymodel.I{2} = [0 0 0; 0 0 0; 0 0 0];

Ymodel.m{3}=1;
Ymodel.CoM{3} = Tras([0; 0; 1]);
Ymodel.I{3} = [0 0 0; 0 0 0; 0 0 0];

Ymodel.m{4}=1;
Ymodel.CoM{4} = Tras([0; 0; 1]);
Ymodel.I{4} = [0 0 0; 0 0 0; 0 0 0];

Ymodel.m{5}=1;
Ymodel.CoM{5} = Tras([0; 0; 1]);
Ymodel.I{5} = [0 0 0; 0 0 0; 0 0 0];

Ymodel.T_Mass=0;
for i=1:Ymodel.NB
Ymodel.T_Mass = Ymodel.T_Mass+ Ymodel.m{i};
end

end
