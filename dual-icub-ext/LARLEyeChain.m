%% Karla Stepanova, Aug 2017
%based on Matej Hoffmann (changes 2017) and Alessandro Roncone (2013) function FwdKin.m 
%(iCub forward kinematics computation)

function F = LARLEyeChain(DH)
global Flags; global VIS_PAR
%function for computing distance between position of the point on 2D
%eye plane measured and estimated by actual model of estimated robot
%minimizes error for both and left eye
%DH - nD vector of input parameters in a structure as defined in ParsToInitGuess.m 
%       vector is containing DH parameters in a following order:
%       [a(1)...a(n) d(1)...d(n) alpha(1)...alpha(n) offset(1)...offset(n)]
%       order of body parts for each estimated paremeter is: 
%       [torso right_arm left_arm head right_eye left_eye]
%F - output function to be optimized in least squares manner
%TODO should be changed to global variables - in optim_script.m
    load('./data_files/joint_data.mat');
    MD = load('./data_files/measured_data.mat');
    %load('Setting.mat')
    M_PI = pi;
    CTRL_DEG2RAD = pi/180;
    if Flags.ONLY_OFFSETS
        Real_robot_pars = robots_config('realRobot');        
        DH_full = ParsToInitGuess(Real_robot_pars,Flags.RIGHT_ARM_CHAIN_ON,Flags.LEFT_ARM_CHAIN_ON,...
                Flags.RIGHT_EYE_CHAIN_ON,Flags.LEFT_EYE_CHAIN_ON,0);
        DHfl = length(DH_full);
        a = DH_full(1:DHfl/4).*1000;%converts to mm
        d    = DH_full((DHfl/4+1):(DHfl/2)).*1000;%converts to mm
        alph = DH_full((DHfl/2+1):(3*DHfl/4));
        offs = DH;
    else        
        DHl = length(DH);
        a = DH(1:DHl/4).*1000;%converts to mm
        d    = DH((DHl/4+1):(DHl/2)).*1000;%converts to mm
        alph = DH((DHl/2+1):(3*DHl/4));
        offs = DH((3*DHl/4 +1):DHl);
    end
 %   K = create_intrinsic_matrix(VIS_PAR);
    
    %determining position of parameters in vector x for individual body parts
    if Flags.RIGHT_ARM_CHAIN_ON
        LA_start = 10;
        head_start = 18;
        if Flags.RIGHT_EYE_CHAIN_ON            
            LEye_start = 24;
            REye_start = 22;
        else
            LEye_start = 22;
            REye_start = 20;
        end
    else
        LA_start = 2;
        head_start = 10;
        if Flags.RIGHT_EYE_CHAIN_ON
            LEye_start = 16;
            REye_start = 14;
        else
            LEye_start = 14;
            REye_start = 12;
        end
    end;
    if ~(Flags.LEFT_ARM_CHAIN_ON && Flags.LEFT_EYE_CHAIN_ON)
        error('Both left arm and left eye chain has to be enabled to evaluate LALECHain')
    elseif ~Flags.TORSO_CHAIN_ON
        error('Torso needs to be enabled in this version,');
    else
        est_p2DRE = [];
        est_p2DLE = [];
        for pp = 1 : size(JV_torso,1)
            %torso chain evaluation
            %real robot torso chain HO
            H0      = [0 -1  0  0; 
                      0  0 -1  0;
                      1  0  0  0;
                      0  0  0  1];
            RFFrame = {};
            ri = 1;
            H0(1:3,4)  = H0(1:3,4).*1000; % converting translational part to mm            
            RFFrame{ri} = H0;
            for j = 1:2
                RTMat{ri+1} = evalDHMatrix(a(j), d(j), alph(j), JV_torso(pp,j)* (CTRL_DEG2RAD) + offs(j));%G = evalDHMatrix(a, d, alph, thet)
                RFFrame{ri+1} = RFFrame{ri} * RTMat{ri+1}; ri = ri + 1;
            end
            %left arm chain evaluation
            RFFRameLA = {};
            RFFrameLA{1} = RFFrame{end};
            ri = 1;
            JV_LA2 = [JV_torso(:,3) JV_LA]* (CTRL_DEG2RAD);%joint angles theta for LA
            for j = 1:8
                RTMat{ri+1} = evalDHMatrix(a(LA_start + j), d(LA_start + j), alph(LA_start + j), JV_LA2(pp,j)+ offs(LA_start + j));%G = evalDHMatrix(a, d, alph, thet)
                RFFrameLA{ri+1} = RFFrameLA{ri} * RTMat{ri+1}; ri = ri + 1;
            end
            %left eye chain evaluation
            RFFRameHead = {};
            RFFrameHead{1} = RFFrame{2};
            ri = 1;
            %head
            JV_head = [JV_torso(:,3) JV_eyes(:,1:3)]* (CTRL_DEG2RAD);%joint angles theta for head
            for j = 1:4
                RTMat{ri+1} = evalDHMatrix(a(head_start + j), d(head_start + j), alph(head_start + j), JV_head(pp,j) + offs(head_start + j));%G = evalDHMatrix(a, d, alph, thet)
                RFFrameHead{ri+1} = RFFrameHead{ri} * RTMat{ri+1}; ri = ri + 1;
            end
            %left eye
            RFFRameLE = {};            
            RFFrameLE{1} = RFFrameHead{end};
            ri = 1;
            JV_LEye = [JV_eyes(:,[4 5])]* (CTRL_DEG2RAD);%joint angles theta for head
            for j = 1:2
                RTMat{ri+1} = evalDHMatrix(a(LEye_start + j), d(LEye_start + j), alph(LEye_start + j), JV_LEye(pp,j) + offs(LEye_start + j));%G = evalDHMatrix(a, d, alph, thet)
                RFFrameLE{ri+1} = RFFrameLE{ri} * RTMat{ri+1}; ri = ri + 1;
            end
            %right eye
            RFFRameRE = {};            
            RFFrameRE{1} = RFFrameHead{end};
            ri = 1;
            JV_REye = [JV_eyes(:,[4 6])]* (CTRL_DEG2RAD);%joint angles theta for head
            for j = 1:2
                RTMat{ri+1} = evalDHMatrix(a(REye_start + j), d(REye_start + j), alph(REye_start + j), JV_REye(pp,j) + offs(REye_start + j));%G = evalDHMatrix(a, d, alph, thet)
                RFFrameRE{ri+1} = RFFrameRE{ri} * RTMat{ri+1}; ri = ri + 1;
            end

            est_pLA = (RFFrameLA{end})*[0 0 0 1]';%position of end effector of left arm 
            est_p3D =(inv(RFFrameLE{end}))*est_pLA;%left eye frame
            est_p2DvLE = -VIS_PAR.f/est_p3D(3)*[est_p3D(1) est_p3D(2)];%2D projection to left eye
            est_3DRE = (inv(RFFrameRE{end}))*est_pLA;%right eye frame left arm
            est_p2DvRE = -VIS_PAR.f/est_3DRE(3)*[est_3DRE(1) est_3DRE(2)];%2D projection of LA to right eye
            est_p2DLE = [est_p2DLE est_p2DvLE'];%projection of EEF LA to LE
            est_p2DRE = [est_p2DRE est_p2DvRE'];%projection of EEF LA to RE
        end
    end
        F = (est_p2DLE - MD.Leye_2D(1:2,:))+(est_p2DRE - MD.Reye_2D(1:2,:));
        disp(F)
end