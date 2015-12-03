function  out = ToSimulink_Y_FK_ST (q)
%% Forward kinemtics by means Successive Transformations
% Inputs: 
%    q = Robot configuration, i.e. each element of q is a joint position
%
% Outputs: 
%    T_ee(1:3,1:3) = End-effector rotation matrix 
%    T_ee(1:3,4) = End-effector position
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
global srcLoaded Ymodel
if isempty(Ymodel)
    if isempty(srcLoaded)
        addpath(genpath('src'));
        display('--> Folder src and subfolders added to the path')
        srcLoaded = true;
    end
    Y_Model();
    display('--> Y model loaded')
end
Y_Model();
[~, T_ee,~,PosCM] = Y_T (q);
[E2_RH]=Euler_Angles(6,T_ee{1}(1:3,1:3));
[E2_LH]=Euler_Angles(6,T_ee{2}(1:3,1:3));
out = [T_ee{1}(1,4);T_ee{1}(3,4);E2_RH;T_ee{2}(1,4);T_ee{2}(3,4);E2_LH;0;0;0;PosCM];

end
