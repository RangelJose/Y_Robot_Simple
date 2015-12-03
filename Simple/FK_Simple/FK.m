function dq = FK(q)
%% Forward Kinematics
   
for i=1 %Transformations for every joint
    T{1} = Tras([0;0;0]);
    Tq{1} = Tras([q(1);0;0]);
    T{2} = Tras([0;0;0]);
    Tq{2} = Tras([0;0;q(2)]);
    T{3} = Tras([0;0;0]);
    Tq{3} = RotY(q(3));
    T{4} = Tras([0;0;2])*RotY(deg2rad(-45));
    Tq{4} = RotY(q(4));
    T{5} = Tras([0;0;2])*RotY(deg2rad(45));
    Tq{5} = RotY(q(5));
    %Transformations to end-effector
    Tee{1} = Tras([0;0;2])*RotY(deg2rad(45));
    Tee{2} = Tras([0;0;2])*RotY(deg2rad(-45));
    %Transformations for every CoM
    TCoM{1} = Tras([0;0;0]);
    TCoM{2} = Tras([0;0;0]);
    TCoM{3} = Tras([0;0;1]);
    TCoM{4} = Tras([0;0;1]);
    TCoM{5} = Tras([0;0;1]);
    %Transformation to every CoM
    MCoM{1} = T{1}*Tq{1}*TCoM{1};
    MCoM{2} = T{1}*Tq{1}*T{2}*Tq{2}*TCoM{2};
    MCoM{3} = T{1}*Tq{1}*T{2}*Tq{2}*T{3}*Tq{3}*TCoM{3};
    MCoM{4} = T{1}*Tq{1}*T{2}*Tq{2}*T{3}*Tq{3}*T{4}*Tq{4}*TCoM{4};
    MCoM{5} = T{1}*Tq{1}*T{2}*Tq{2}*T{3}*Tq{3}*T{5}*Tq{5}*TCoM{5};
    %Masses of every link
    m{1}=0;
    m{2}=0;
    m{3}=1;
    m{4}=1;
    m{5}=1;
    %Average
    CoM{1} = MCoM{1}(1:3,4)*m{1};
    CoM{2} = MCoM{2}(1:3,4)*m{2};
    CoM{3} = MCoM{3}(1:3,4)*m{3};
    CoM{4} = MCoM{4}(1:3,4)*m{4};
    CoM{5} = MCoM{5}(1:3,4)*m{5};
    PosCM = (CoM{1}+CoM{2}+CoM{3}+CoM{4}+CoM{5})/(m{1}+m{2}+m{3}+m{4}+m{5});
    %End effectors
    T_ee{1} = T{1}*Tq{1}*T{2}*Tq{2}*T{3}*Tq{3}*T{4}*Tq{4}*Tee{1}; %Right robot end effector
    T_ee{2} = T{1}*Tq{1}*T{2}*Tq{2}*T{3}*Tq{3}*T{5}*Tq{5}*Tee{2}; %Left robot end effector
    %Convenion YZX
    EulerRH = atan2(-T_ee{1}(3,1),T_ee{1}(1,1));
    EulerLH = atan2(-T_ee{2}(3,1),T_ee{2}(1,1));
    %Poses
    EE_RH = [T_ee{1}(1,4);T_ee{1}(3,4);EulerRH]
    EE_LH = [T_ee{2}(1,4);T_ee{2}(3,4);EulerLH]
end

dq=[EE_RH;EE_LH;0;0;0;PosCM];
end