function dX = DFK(q,dq)
%% Constants
m1 = 0.7;
m2 = 0.7;
m3 = 0.7;
M = 2.1;
DmpB=0.00001;


%% Forward Kinematics
% FK to the beginning of link i after the rotation of joint i
Link1_pose = Tras([q(1);0;q(2)])* RotY(q(3));
Link2_pose = Link1_pose * Tras([0;0;2])*RotY(-pi/4+q(4));
Link3_pose = Link1_pose * Tras([0;0;2])*RotY(pi/4+q(5));  


% FK to each end-effector
EEright_pose = Link2_pose*Tras([0;0;2])*RotY(deg2rad(45));
EEleft_pose = Link3_pose*Tras([0;0;2])*RotY(deg2rad(-45));
EEright_Euler_Angles = [atan2(-EEright_pose(3,1),EEright_pose(1,1)); 0 ; 0];
EEleft_Euler_Angles = [atan2(-EEleft_pose(3,1),EEleft_pose(1,1));0;0];

% FK to each CoM
Link1_CoM_pose = Link1_pose * Tras([0;0;1]);
Link2_CoM_pose = Link2_pose * Tras([0;0;1]);
Link3_CoM_pose = Link3_pose * Tras([0;0;1]);

% Position of the CoM of each link
Link1_CoM_position = Link1_CoM_pose(1:3,4);
Link2_CoM_position = Link2_CoM_pose(1:3,4);
Link3_CoM_position = Link3_CoM_pose(1:3,4);

% Position of the robot CoM
Robot_CoM_position = (Link1_CoM_position*m1 + Link2_CoM_position*m2 + Link3_CoM_position*m3 )/ (M);

% Jacobians of each end effector
B_right = [[Ry(EEright_Euler_Angles(1))*[0;1;0]] [Ry(EEright_Euler_Angles(1))*Rz(0)*[0;0;1]] [Ry(EEright_Euler_Angles(1))*Rz(0)*Rx(0)*[1;0;0]]];
B_left = [[Ry(EEleft_Euler_Angles(1))*[0;1;0]] [Ry(EEleft_Euler_Angles(1))*Rz(0)*[0;0;1]] [Ry(EEleft_Euler_Angles(1))*Rz(0)*Rx(0)*[1;0;0]]];
EEright_Jv = [[1;0;0] [0;0;1] cross([0; 1; 0],(EEright_pose(1:3,4)-Link1_pose(1:3,4))) cross((Link1_pose(1:3,1:3)*[0; 1; 0]),(EEright_pose(1:3,4)-Link2_pose(1:3,4))) zeros(3,1) ];
EEright_Ja = (B_right+(eye(3)*DmpB))\[[0;0;0] [0;0;0] ([0; 1; 0]) (Link1_pose(1:3,1:3)*[0; 1; 0]) zeros(3,1) ];
EEleft_Jv = [[1;0;0] [0;0;1] cross([0; 1; 0],(EEleft_pose(1:3,4)-Link1_pose(1:3,4))) zeros(3,1) cross((Link1_pose(1:3,1:3)*[0; 1; 0]),(EEleft_pose(1:3,4)-Link3_pose(1:3,4))) ];
EEleft_Ja = (B_left+(eye(3)*DmpB))\[[0;0;0] [0;0;0] (Link1_pose(1:3,1:3)*[0; 1; 0]) zeros(3,1) (Link3_pose(1:3,1:3)*[0; 1; 0]) ];
% EEright_Ja = (eye(3)*dmpB)\EEright_Jw;
% EEleft_Ja = (eye(3)*dmpB)\EEleft_Jw;


EEright_J= [EEright_Jv(1,:);EEright_Jv(3,:);EEright_Ja(1,:)];
EEleft_J= [EEleft_Jv(1,:);EEleft_Jv(3,:);EEleft_Ja(1,:)];
% Jacobians of each CoM 
Link1_CoM_Jv = [ [1; 0; 0] [0; 0; 1] cross([0; 1; 0],(Link1_CoM_pose(1:3,4)-Link1_pose(1:3,4))) zeros(3,1) zeros(3,1) ];
Link2_CoM_Jv = [ [1; 0; 0] [0; 0; 1] cross([0; 1; 0],(Link2_CoM_pose(1:3,4)-Link1_pose(1:3,4))) cross([0; 1; 0],(Link2_CoM_pose(1:3,4)-Link2_pose(1:3,4))) zeros(3,1) ];
Link3_CoM_Jv = [ [1; 0; 0] [0; 0; 1] cross([0; 1; 0],(Link3_CoM_pose(1:3,4)-Link1_pose(1:3,4))) zeros(3,1) cross([0; 1; 0],(Link3_CoM_pose(1:3,4)-Link3_pose(1:3,4))) ];

% Robot CoM Jacobian
Robot_CoM_Jv = (Link1_CoM_Jv*m1 + Link2_CoM_Jv*m2 + Link3_CoM_Jv*m3)/M;

% Robot CoM linear velocity
Robot_CoM_v = Robot_CoM_Jv*dq;

% Robot end effector velocity
Robot_EEright_v = EEright_J * dq;
Robot_EEleft_v = EEleft_J * dq;

EE_RH_nu = [0; 0; 0];
EE_LH_nu = [0; 0; 0];
dX=[Robot_EEright_v; Robot_EEleft_v; Robot_CoM_v; Robot_CoM_position];
end