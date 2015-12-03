 function out = ToSimulink_Y_Signals2VRML (q,EEpose_LH,EEpose_RH,PosCM)
%% Signals to VRML
% This file controls the signals to send to VRML(Virtual Model)
% Inputs:
%     q=Robot configuration, i.e. each element of q is a joint position
%     EEpose_RH= Right Hand pose Xi of the Right Hand
%     EEpose_LH= Left Hand pose Xi of the Left Hand
%     EEpose_RF= Right Foot pose Xi of the Right Foot
%     EEpose_LF= Left Foot pose Xi of the Left Foot
% 
% Outputs:
%     VRML signals= This send the Virtual Model signals to control the position of the frame of each end-effector    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Z=[0;0;-1];
 qTras=[q(1);q(2);0];
 q3= [Z;q(3)];
 q4= [Z;q(4)];
 q5= [Z;q(5)];


EEpose_RH_Tras=[EEpose_RH(1);EEpose_RH(2);0];%Right Hand pose, Position and Orientation
EEpose_RH_Y=[Z;EEpose_RH(3)];

EEpose_LH_Tras=[EEpose_LH(1);EEpose_LH(2);0];%Left Hand pose, Position and Orientation
EEpose_LH_Y=[Z;EEpose_LH(3)];

PosCM_Tras=[PosCM(1);0;0];

out=[qTras;q3;q4;q5;EEpose_RH_Tras;EEpose_RH_Y;EEpose_LH_Tras;EEpose_LH_Y;PosCM_Tras];
end