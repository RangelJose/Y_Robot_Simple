function dq = HIK(q)
%% Y Robot Hierachical Inverse Kinematics
% Inputs:
%    q     = Robot configuration, i.e. each element of q is a joint position
%
% Outputs:
%    dq = Articular Velocities
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

global srcLoaded Ymodel FK
if isempty(Ymodel)
    if isempty(srcLoaded)
        addpath(genpath('src'));
        display('--> Folder src and subfolders added to the path')
        srcLoaded = true;
    end
    Y_Model();
    display('--> Y model loaded')
end

%Forward Kinematics
[T, T_ee,MCoM,PosCM] = Y_T (q);
[E_RH]=Euler_Angles(6,T_ee{1}(1:3,1:3));
[E_LH]=Euler_Angles(6,T_ee{2}(1:3,1:3));

FK.PosCoM = PosCM;
% Stack of tasks
[P,EE,Targets,G,DmpB,DmpJ] = SoT();

%Poses for the tasks
Pose{1} = [T_ee{1}(1,4);T_ee{1}(3,4);E_RH];
Pose{2} = [T_ee{2}(1,4);T_ee{2}(3,4);E_LH];
Pose{3} = PosCM(1);

%Computing Delta X (error)
for i=1:P
    Dx{i} = Targets{i} - Pose{EE{i,2}};
end
%Geometrical Jacobians
[Jv1,Jw1]=Jg_EE(T_ee{1}(1:3,4),T,4,Ymodel.parent,Ymodel.axes);
[Jv2,Jw2]=Jg_EE(T_ee{2}(1:3,4),T,5,Ymodel.parent,Ymodel.axes);

%Analytical Jacobians
Ja{1}=Ja_EE(Jv1,Jw1,DmpB,[2;3;1],[E_RH,0,0]);
Ja{1} = [Ja{1}(1,:);Ja{1}(3,:);Ja{1}(4,:)];
Ja{2}=Ja_EE(Jv2,Jw2,DmpB,[2;3;1],[E_LH,0,0]);
Ja{2} = [Ja{2}(1,:);Ja{2}(3,:);Ja{2}(4,:)];
Ja{3} = Jac_CoM(MCoM,T,Ymodel.NB,Ymodel.parent,Ymodel.axes,Ymodel.m,Ymodel.T_Mass);

dq = Y_Solver(Ja,Dx,P,G,DmpJ,Ymodel.NB);
end