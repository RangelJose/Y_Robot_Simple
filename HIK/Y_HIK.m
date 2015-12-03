function dq = Y_HIK(q)
%% Bioloid Hierachical Inverse Kinematics
% Inputs:
%    q     = Robot configuration, i.e. each element of q is a joint position
%    Index = Index for the Euler's Angles
%
% Outputs:
%    dq = Articular Velocities
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Charge of the global variables, model, and Euler convention
global srcLoaded Ymodel EulerConvention
if isempty(Ymodel)
    if isempty(srcLoaded)
        addpath(genpath('src'));
        display('--> Folder src and subfolders added to the path')
        srcLoaded = true;
    end
    Y_Model();
    Euler_Convention(6);
    display('-->Y model loaded')
end %End If isempty BM

% Variables, model, and Euler convention charged



%% Computing forward  kinematics
[T,T_ee,MCoM,PosCM] = Y_T (q);
%T_ee{1}
%T_ee{2}
% Forward kinematics computed

%% Poses
%EulerConvention.IndexSaved
[E1] = Euler_Angles(EulerConvention.IndexSaved,T_ee{1}(1:3,1:3)); % Euler angles being obtained according to the indicated convention
Pose{1} = [T_ee{1}(1,4);T_ee{1}(3,4);E1];                                    % Building pose with position and orientation %Right hand pose
[E1] = Euler_Angles(EulerConvention.IndexSaved,T_ee{2}(1:3,1:3)); 
Pose{2} = [T_ee{2}(1,4);T_ee{2}(3,4);E1];                                     % Left hand pose
Pose{3} = PosCM(1);
%


%% Call of the Stack of task
[P,EE,Targets,G,BDmp,JDmp] = SoT();
%

%% Computing Tasks (Dx)
Dx = cell(P,1);
for i=1:P 
    Dx{i} = Targets{i} - Pose{EE{i,2}};
end%Cycle for Dx

%% Computing Jacobians
J = Y_Ja(T_ee,T,EE,2,MCoM,PosCM,BDmp); %Calling the function of the Jacobians
J{3} = Jac_CoM(MCoM,T,Ymodel.NB,Ymodel.parent,Ymodel.axes,Ymodel.m,Ymodel.T_Mass);
%Jacobians

%% Solver
dq = Y_Solver(J,Dx,P,G,JDmp,Ymodel.NB); %Calling the function of the solver
%
end