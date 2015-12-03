 function out = ToSimulink_Y_Signals2VRML (q)
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
global FK
EEpose_RH_Tras=zeros(3,1);%Right Hand pose, Position and Orientation
EEpose_RH_Y=zeros(4,1);
EEpose_LH_Tras=zeros(3,1);
EEpose_LH_Y=zeros(4,1);
ObjCM_Tras=zeros(3,1); %3

Y=[0;0;-1];
 qTras=[q(1);q(2);0];
 q3= [Y;q(3)];
 q4= [Y;q(4)];
 q5= [Y;q(5)]; %%15
 
[P,EE,Targets] = SoT();
 PosCM = FK.PosCoM;
for i = 1:P
switch EE{i,1}
    case 'RH'
EEpose_RH_Tras=[Targets{i}(1);Targets{i}(2);0];%Right Hand pose, Position and Orientation
EEpose_RH_Y=[Y;Targets{i}(3)]; %7
    case 'LH'
EEpose_LH_Tras=[Targets{i}(1);Targets{i}(2);0];%Left Hand pose, Position and Orientation
EEpose_LH_Y=[Y;Targets{i}(3)]; %7
    case 'CM'
ObjCM_Tras=[Targets{i}(1);0;0]; %3
end
end
PosCM_Tras=[PosCM(1);0;0];

out=[qTras;q3;q4;q5;EEpose_RH_Tras;EEpose_RH_Y;EEpose_LH_Tras;EEpose_LH_Y;ObjCM_Tras;PosCM_Tras];
end