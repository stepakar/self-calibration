%% Karla Stepanova, Aug 2017
%based on Matej Hoffmann (changes 2017) and Alessandro Roncone (2013) function FwdKin.m 
%(iCub forward kinematics computation)
%
function [LA_3D, RA_3D, Leye_2D, Reye_2D, RFFrameLA,RFFrameRA,RD] = compute_points_3Dand2D(JV_torso,JV_LA,JV_RA,JV_eyes, Robot_pars, Flags,VIS_PAR)
    %function computes position of EEF of left and right arms and projection of 
    %these EEFs in 2D on left and right eye for a robot which parameters
    %are setted in file Robot_pars (DH parameters)
    %
    %INPUTS:
    %   JV_LA - measured joint values of left arm 
    %       e.g. JV_LA  = [-44.053443 23.359441 49.646422 74.846729 -0.558269 0.006326 -3.871375];   
    %   JV_RA - measured joint values of right arm
    %       e.g. JV_RA  = [-44.508147 23.429129 49.529941 75.007906 0.119645 0.019927 -3.904707];
    %   JV_eyes - measured joint values of eyes and neck 
    %       [joints_neck_deg joint_eyeTilt_deg joint_LeyePan_deg joint_ReyePan_deg]
    %       e.g. JV_eyes = [0.0 0.0 0.0 -60.0 0.0 0.0]
    %   Robot_pars - structure with robot parameters as setted in robot
    %       cofiguration file (e.g. Robot_pars = robots_config('realRobot'))
    %   Flags - structure with flags for individual body_parts 0n/off (e.g.
    %       Flags.TORSO_CHAIN_ON
    %OUTPUTS:
    %   LA_3D - [x y z]' - (3xn vector): 3D position of left arm end
    %   effector (each column is for 1 pose)
    %   RA_3D - [x y z]' - (3xn vector): 3D position of right arm end
    %   effector (each column is for 1 pose)
    %	Leye_2D - [xLA yLA xRA yRA ]' - (4xn matrix): 2D projection of left and right arm
    %       EEF on left eye plane (each colum is for 1 pose)
    %   Reye_2D - [xLA yLA xRA yRA]' - (4xn matrix): 2D projection of left and right arm
    %       EEF on right eye plane (each column is for 1 pose)
    M_PI = pi;
    CTRL_DEG2RAD = pi/180;
    %should be changed afterwards, but for now it is ok
    DH = ParsToInitGuess(Robot_pars,1,1,1,1,0);%InitGuess = ParsToInitGuess(Pars,RA,LA,REye,LEye,only_offsets);    
    %determining position of parameters in vector x for individual body parts
    RA_start = 2;
    LA_start = 10;
    head_start = 18;
    REye_start = 22;
    LEye_start = 24;

    RA_3D = [];
    LA_3D = [];
    Leye_2D = [];
    Reye_2D = [];
    
    DHl = length(DH);
    a = DH(1:DHl/4).*1000;%converting to mm
    d    = DH((DHl/4+1):(DHl/2)).*1000;%converting to mm
    alph = DH((DHl/2+1):(3*DHl/4));
    offs = DH((3*DHl/4 +1):DHl);

    if ~ Flags.TORSO_CHAIN_ON
        error('Torso needs to be enabled in this version');
    else
        for pp = 1: size(JV_torso,1)
            %torso chain evaluation
            %real robot torso chain HO
            H0      = [0 -1  0  0; 
                      0  0 -1  0;
                      1  0  0  0;
                      0  0  0  1];
            ri = 1;
            H0(1:3,4)  = H0(1:3,4).*1000; % converting translational part to mm
            RFFrame = {};
            RFFrame{ri} = H0; 
            for j = 1:2
                RTMat{ri+1} = evalDHMatrix(a(j), d(j), alph(j), JV_torso(pp,j)* (CTRL_DEG2RAD) + offs(j));%G = evalDHMatrix(a, d, alph, thet)
                RFFrame{ri+1} = RFFrame{ri} * RTMat{ri+1}; ri = ri + 1;
            end
            if Flags.LEFT_ARM_CHAIN_ON
                %left arm chain evaluation
                RFFrameLA = {};
                RFFrameLA{1} = RFFrame{3};
                ri = 1;
                JV_LA2 = [JV_torso(:,3) JV_LA]* (CTRL_DEG2RAD);%joint angles theta for LA                
                for j = 1:8
                    RTMat{ri+1} = evalDHMatrix(a(LA_start + j), d(LA_start + j), alph(LA_start + j), JV_LA2(pp,j)+ offs(LA_start + j));%G = evalDHMatrix(a, d, alph, thet)
                    RFFrameLA{ri+1} = RFFrameLA{ri} * RTMat{ri+1}; ri = ri + 1;
                end
            end
            %right arm chain evaluation
            if Flags.RIGHT_ARM_CHAIN_ON
                RFFrameRA = {};
                RFFrameRA{1} = RFFrame{3};
                ri = 1;
                JV_RA2 = [JV_torso(:,3) JV_RA]* (CTRL_DEG2RAD);%joint angles theta for LA
                for j = 1:8
                    RTMat{ri+1} = evalDHMatrix(a(RA_start + j), d(RA_start + j), alph(RA_start + j), JV_RA2(pp,j)+ offs(RA_start + j));%G = evalDHMatrix(a, d, alph, thet)
                    RFFrameRA{ri+1} = RFFrameRA{ri} * RTMat{ri+1}; ri = ri + 1;
                end
            end
            %left eye chain evaluation
            if (Flags.LEFT_EYE_CHAIN_ON || Flags.RIGHT_EYE_CHAIN_ON)
                RFFrameHead = {};
                RFFrameHead{1} = RFFrame{2};
                ri = 1;
                %head
                JV_head = [JV_torso(:,3) JV_eyes(:,1:3)]* (CTRL_DEG2RAD);%joint angles theta for head
                for j = 1:4
                    RTMat{ri+1} = evalDHMatrix(a(head_start + j), d(head_start + j), alph(head_start + j), JV_head(pp,j) + offs(head_start + j));%G = evalDHMatrix(a, d, alph, thet)
                    RFFrameHead{ri+1} = RFFrameHead{ri} * RTMat{ri+1}; ri = ri + 1;
                end
            end
            if Flags.LEFT_EYE_CHAIN_ON
                RFFrameLEye = {};
                RFFrameLEye{1} = RFFrameHead{5};
                ri = 1;
                %left eye
                JV_LEye = [JV_eyes(:,[4 5])]* (CTRL_DEG2RAD);%joint angles theta for head
                for j = 1:2
                    RTMat{ri+1} = evalDHMatrix(a(LEye_start + j), d(LEye_start + j), alph(LEye_start + j), JV_LEye(pp,j) + offs(LEye_start + j));%G = evalDHMatrix(a, d, alph, thet)
                    RFFrameLEye{ri+1} = RFFrameLEye{ri} * RTMat{ri+1}; ri = ri + 1;
                end
            end
            if Flags.RIGHT_EYE_CHAIN_ON
                RFFrameREye = {};
                RFFrameREye{1} = RFFrameHead{5};
                ri = 1;
                %right eye
                JV_REye = [JV_eyes(:,[4 6])]* (CTRL_DEG2RAD);%joint angles theta for head
                for j = 1:2
                    RTMat{ri+1} = evalDHMatrix(a(REye_start + j), d(REye_start + j), alph(REye_start + j), JV_REye(pp,j) + offs(REye_start + j));%G = evalDHMatrix(a, d, alph, thet)
                    RFFrameREye{ri+1} = RFFrameREye{ri} * RTMat{ri+1}; ri = ri + 1;
                end
            end    
            if Flags.LEFT_ARM_CHAIN_ON
                LA_3Dv = (RFFrameLA{end})*[0 0 0 1]';%position of end effector of left arm
                LA_3D = [LA_3D LA_3Dv];
            end
            if Flags.RIGHT_ARM_CHAIN_ON
                RA_3Dv = (RFFrameRA{end})*[0 0 0 1]';%position of end effector of left arm 
                RA_3D = [RA_3D RA_3Dv];
            end
            if Flags.LEFT_EYE_CHAIN_ON
                Leye_3DLA =(inv(RFFrameLEye{end}))*LA_3Dv;%left eye frame left arm
                Leye_2DLA = -VIS_PAR.f/Leye_3DLA(3)*[Leye_3DLA(1) Leye_3DLA(2)];%2D projection of LA to left eye
                Leye_3DRA =(inv(RFFrameLEye{end}))*RA_3Dv;%left eye frame right arm
                Leye_2DRA = -VIS_PAR.f/Leye_3DRA(3)*[Leye_3DRA(1) Leye_3DRA(2)];%2D projection of RA to left eye
                Leye_2Dv = [Leye_2DLA Leye_2DRA]';
                Leye_2D = [Leye_2D Leye_2Dv];
            end
            if Flags.RIGHT_EYE_CHAIN_ON
                RD =(inv(RFFrameREye{end}));
                Reye_3DLA = (inv(RFFrameREye{end}))*LA_3Dv;%right eye frame left arm
                Reye_2DLA = -VIS_PAR.f/Reye_3DLA(3)*[Reye_3DLA(1) Reye_3DLA(2)];%2D projection of LA to right eye
                Reye_3DRA = (inv(RFFrameREye{end}))*RA_3D;%right eye frame right arm
                Reye_2DRA = -VIS_PAR.f/Reye_3DRA(3)*[Reye_3DRA(1) Reye_3DRA(2)];%2D projection of RA to right eye
                Reye_2Dv = [Reye_2DLA Reye_2DRA]';
                Reye_2D = [Reye_2D Reye_2Dv];
            end
        end
end
% function K = create_intrinsic_matrix(VIS_PAR)
%     K = [VIS_PAR.f 0 300;0 VIS_PAR.f -300; 0 0 1]
% end