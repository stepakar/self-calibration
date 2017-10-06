%% Karla Stepanova, Aug 2017
% extended version of ../dual-icub/iCubCalib.m script by Matej Hoffmann
% which was based on ../icub-matlab (which was in turn based on 
% https://github.com/alecive/kinematics-visualization-matlab by Alessandro Roncone)
% Making a version with two models - one "real" and one current model estimate (subject to calibration)
% Extensions: added visualisation of 2D projection of arms EEFs on 2D eye
% plane (VIS_PROJ_ARMS_EEF_ON)
clc; %clear;

TORSO_CHAIN_ON = true; % this chain is mandatory - from the original iCub root frame to 3rd (last) torso joint
% the last torso joint is the start of all other chains (arms and head)
% USER CHOICE 
LEFT_ARM_CHAIN_ON = true;
RIGHT_ARM_CHAIN_ON = true;
HEAD_CHAIN_ON = true;
LEFT_EYE_CHAIN_ON = true; % requires HEAD_CHAIN_ON == true;
RIGHT_EYE_CHAIN_ON = true; % requires HEAD_CHAIN_ON == true;

VIS_PROJ_ARMS_EEF_ON = true;
INIT_SETTINGS_DH = true;%false loads computed parameters from a file estimated_parameters.mat
LOAD_DH_PAR = false;%if true, loads computed parameters from a file estimated_parameters.mat
ONLY_OFFSETS = false; %if true, loads computed offsets from a file estimated_parameters_offs.mat
ESTIMATED_ROBOT_ON = true;
SHOW_OPTIMIZATION_ERROR = true;

addpath('./utils/');

M_PI = pi;
CTRL_DEG2RAD = pi/180;

LINK_COLOR_REAL = [0.8 0.5 0.5]; % redish
LINK_COLOR_ESTIMATE = [0.5 0.5 0.8]; % blueish

%parameters for visualisation of 3D-2D projection
VIS_PAR.f = 257;%focal length
VIS_PAR.VAng = 60;%field of view
VIS_PAR.RetS = VIS_PAR.VAng*pi()*VIS_PAR.f/360;%f*sqrt(3)/3;size of the retina

%% Joint values (as read from the encoders)
%  These will be common to both models: real and estimate
   joints_torso_deg = C(selected_poses,4:6)%[0.0   0.0   0.0]; % pitch, roll, yaw order (iKin, CAD, DH) - unlike iCub motor interfaces
   
   %canonical arm posture 
   %joints_Larm_deg  = [-30.0  30.0   0.0  45.0   0.0   0.0   0.0];
   %joints_Rarm_deg  = [-30.0  30.0   0.0  45.0   0.0   0.0   0.0];
   %self-touch arm configuration
   joints_Larm_deg  = C(selected_poses,7:13)%[-23.6988   24.5989   42.5855   66.4666    0.0391         0   -3.5190];
   joints_Rarm_deg  = [-45.0474   23.0000   80.0000   68.1435  -88.5362  -20.6167   12.4597];
 
   joints_neck_deg = [0.0 0.0 0.0];
   joint_eyeTilt_deg = -60.0;
   joint_LeyePan_deg = 0.0;
   joint_ReyePan_deg = 0.0;
   
   if ONLY_OFFSETS
       load('./data_files/resulting_parameters_offs.mat');       
       lR = length(R);
       rarm_start = 2;
       larm_start = 10;
       head_start = 18;
       reye_start = 22;
       leye_start = 24;           
   elseif LOAD_DH_PAR
       load('./data_files/resulting_parameters.mat');       
       lR = length(R);
       rarm_start = 2;
       larm_start = 10;
       head_start = 18;
       reye_start = 22;
       leye_start = 24;
   end
%% INIT AND PLOT BODY PARTS   
   
% Each body part has the same structure:
%     name = the name of the body_part;
%     H0   = is the roto-translation matrix in the origin of the chain (if the body part is attached
%            to another one, typically the last reference frame of the previous body part goes here)
%     DH   = it's the parameter matrix. Each row has 4 DH parameters (a, d, alpha, offset), thus each
%            row completely describes a link. The more rows are added, the more links are attached.
%     Th   = it's the joint values vector (as read from the encoders)
%  Please note that everything is in SI units (i.e. meters and radians), 
%  unless the variables have the _deg suffix (the eventual conversions will be handled by
%  the algorithm itself). However, the FwdKin.m works internally with
%  mm and so the chains and transforms returned have translations in mm.

% The plotting is done by the FwdKin function, which also outputs the chain
% of the corresponding body part. This is needed as input to subsequent
% chains (e.g. torso is needed for arm and head chains).

figure('Position', [1436 30 1300 750]);
    axes  ('Position', [0 0 1 1]); hold on; grid on;
    xlabel('x (mm)'); ylabel('y (mm)'),zlabel('z (mm)');
   
  
    
%% ROOT - plot the original iCub root reference frame
    root = eye(4);
    DrawRefFrame(root,1,40,'hat','ROOT');
  
%% TORSO
if TORSO_CHAIN_ON 
   % root to 3rd torso joint
   % real robot    
    realRobot.tor.name = 'root-to-torso';
    realRobot.tor.H0      = [0 -1  0  0;
                      0  0 -1  0;
                      1  0  0  0;
                      0  0  0  1];
    realRobot.tor.DH = [     0.032,      0.0,  M_PI/2.0,                 0.0; % link from 1st to 2nd torso joint
                              0.0,  -0.0055,  M_PI/2.0,           -M_PI/2.0;]; % link from 2nd to 3rd torso joint 
    realRobot.tor.Th = joints_torso_deg(1:2); % only first two joints go here 
    realRobot.tor.Th = realRobot.tor.Th * (CTRL_DEG2RAD);
    realRobot.tor.LinkColor = LINK_COLOR_REAL;
    realRobot.chain.rootToTorso = FwdKin(realRobot.tor,'noFrames'); % we don't draw this chain common to all
    
    if ESTIMATED_ROBOT_ON   % for this chain, they can be identical
        estimatedRobot.tor = realRobot.tor;
        if LOAD_DH_PAR
            estimatedRobot.tor.DH = [R(1), R(lR/4 + 1), R(lR/2 + 1), R(3*lR/4+1);
            R(2), R(lR/4 + 2), R(lR/2 + 2), R(3*lR/4+2);] 
        elseif ONLY_OFFSETS
            estimatedRobot.tor.DH = realRobot.tor.DH;
            estimatedRobot.tor.DH(:,4) = [R(1:2)'];
        end
        estimatedRobot.chain.rootToTorso = realRobot.chain.rootToTorso;
    end        
end

%% LEFT_ARM
% taking arm v2 kinematics version from http://wiki.icub.org/wiki/ICubFowardKinematics_left
if LEFT_ARM_CHAIN_ON
   realRobot.larm.name = 'left-arm';
   if TORSO_CHAIN_ON
      realRobot.larm.H0 = realRobot.chain.rootToTorso.RFFrame{end};
      realRobot.larm.H0(1:3,4)  = realRobot.larm.H0(1:3,4)./1000; % converting the translational part from mm back to m 
   else
      error('Torso needs to be enabled in this version,');
   end
   realRobot.larm.DH = [        0.0233647,  -0.1433, -M_PI/2.0,  105.0*CTRL_DEG2RAD;  % newly added to arm chain - from 3rd torso to shoulder
                           0.0,  0.10774, -M_PI/2.0,            M_PI/2.0;
                           0.0,      0.0,  M_PI/2.0,           -M_PI/2.0;
                         0.015,  0.15228, -M_PI/2.0,   75.0*CTRL_DEG2RAD;
                        -0.015,      0.0,  M_PI/2.0,                 0.0;
                          0.0,   0.1413,  M_PI/2.0,           -M_PI/2.0;    %V1:  0.0, 0.1373, M_PI/2.0, -M_PI/2.0;
                           0.0,      0.0,  M_PI/2.0,            M_PI/2.0;
                        0.0625,   -0.02598,     0.0,                 0.0 ]; %V1: 0.0625, -0.016,  0.0, 0.0 
   realRobot.larm.Th = [joints_torso_deg(3) joints_Larm_deg];
   realRobot.larm.Th = realRobot.larm.Th * (CTRL_DEG2RAD);
   realRobot.larm.LinkColor = LINK_COLOR_REAL;
   realRobot.chain.left_arm = FwdKin(realRobot.larm);

    if ESTIMATED_ROBOT_ON   
        estimatedRobot.larm = realRobot.larm;
        if TORSO_CHAIN_ON
            estimatedRobot.larm.H0 = estimatedRobot.chain.rootToTorso.RFFrame{end};
            estimatedRobot.larm.H0(1:3,4)  = estimatedRobot.larm.H0(1:3,4)./1000; % converting the translational part from mm back to m 
        else
            error('Torso needs to be enabled in this version,');
        end
        if INIT_SETTINGS_DH
        estimatedRobot.larm.DH = [0.03,  -0.16, -M_PI/2.0,  125.0*CTRL_DEG2RAD;  % newly added to arm chain - from 3rd torso to shoulder
                                   0.0,  0.15, -M_PI/2.0,    M_PI/2.0;
                                   0.0,      0.0,  M_PI/2.0,    -M_PI/2.0;
                                   0.018,  0.22, -M_PI/2.0,   75.0*CTRL_DEG2RAD;
                                   -0.012,      0.0,  M_PI/2.0,        0.0;
                                    0.0,   0.12,  M_PI/2.0,           -M_PI/2.0;    
                                    0.0,      0.0,  M_PI/2.0,            M_PI/2.0;
                                    0.07,   -0.03,     0.0,                 0.0 ];  
        elseif ONLY_OFFSETS
            estimatedRobot.larm.DH = realRobot.larm.DH;
            estimatedRobot.larm.DH(:,4) = [R((1 + larm_start):(8 + larm_start))'];
        else
        estimatedRobot.larm.DH = [R((1 + larm_start):(8 + larm_start))',...
        R((lR/4 + larm_start + 1):(lR/4 + larm_start + 8))',...
        R((lR/2 + larm_start + 1):(lR/2 + larm_start + 8))',... 
        R((3*lR/4 + larm_start + 1):(3*lR/4 + larm_start + 8))']    
        end 
        estimatedRobot.larm.LinkColor = LINK_COLOR_ESTIMATE;
        estimatedRobot.chain.left_arm = FwdKin(estimatedRobot.larm);        
    end    
end
    
%% RIGHT_ARM
% taking v2 kinematics version from http://wiki.icub.org/wiki/ICubFowardKinematics_right
if RIGHT_ARM_CHAIN_ON
     realRobot.rarm.name = 'right-arm';
   if TORSO_CHAIN_ON
      realRobot.rarm.H0 = realRobot.chain.rootToTorso.RFFrame{end};
      realRobot.rarm.H0(1:3,4)  = realRobot.rarm.H0(1:3,4)./1000; % converting the translational part from mm back to m 
   else
      error('Torso needs to be enabled in this version,');
   end
   realRobot.rarm.DH =  [-0.0233647,  -0.1433,  M_PI/2.0, -105.0*CTRL_DEG2RAD  % newly added to arm chain - from 3rd torso to shoulder
                         0.0, -0.10774,  M_PI/2.0,           -M_PI/2.0;
                          0.0,      0.0, -M_PI/2.0,           -M_PI/2.0;
                       -0.015, -0.15228, -M_PI/2.0, -105.0*CTRL_DEG2RAD;
                        0.015,      0.0,  M_PI/2.0,                 0.0;
                          0.0,  -0.1413,  M_PI/2.0,           -M_PI/2.0; %V1:  0.0,  -0.1373,  M_PI/2.0,  -M_PI/2.0;
                          0.0,      0.0,  M_PI/2.0,            M_PI/2.0;
                       0.0625,    0.02598,   0.0,                M_PI ]; %V1:  0.0625,  0.016,  0.0, M_PI
   realRobot.rarm.Th = [joints_torso_deg(3) joints_Rarm_deg];
   realRobot.rarm.Th = realRobot.rarm.Th * (CTRL_DEG2RAD);
   realRobot.rarm.LinkColor = LINK_COLOR_REAL;
   realRobot.chain.right_arm = FwdKin(realRobot.rarm);

   if ESTIMATED_ROBOT_ON   
        estimatedRobot.rarm = realRobot.rarm;
        if TORSO_CHAIN_ON
            estimatedRobot.rarm.H0 = estimatedRobot.chain.rootToTorso.RFFrame{end};
            estimatedRobot.rarm.H0(1:3,4)  = estimatedRobot.rarm.H0(1:3,4)./1000; % converting the translational part from mm back to m 
        else
            error('Torso needs to be enabled in this version,');
        end
        if INIT_SETTINGS_DH
        estimatedRobot.rarm.DH =  [-0.03,  -0.1433,  M_PI/2.0, -85.0*CTRL_DEG2RAD  % newly added to arm chain - from 3rd torso to shoulder
                         0.0, -0.10774,  M_PI/2.0,           -M_PI/2.0;
                          0.0,      0.0, -M_PI/2.0,           -M_PI/2.0;
                       -0.015, -0.15228, -M_PI/2.0, -105.0*CTRL_DEG2RAD;
                        0.015,      0.0,  M_PI/2.0,                 0.0;
                          0.0,  -0.1413,  M_PI/2.0,           -M_PI/2.0; 
                          0.0,      0.0,  M_PI/2.0,            M_PI/2.0;
                       0.0625,    0.02598,   0.0,                M_PI ]; 
        elseif ONLY_OFFSETS
            estimatedRobot.rarm.DH = realRobot.rarm.DH;
            estimatedRobot.rarm.DH(:,4) = [R((1 + rarm_start):(8 + rarm_start))'];
        else
            estimatedRobot.rarm.DH = [R((1 + rarm_start):(8 + rarm_start))',...
            R((lR/4 + rarm_start + 1):(lR/4 + rarm_start + 8))',...
            R((lR/2 + rarm_start + 1):(lR/2 + rarm_start + 8))',... 
            R((3*lR/4 + rarm_start + 1):(3*lR/4 + rarm_start + 8))']    
        end            
        estimatedRobot.rarm.LinkColor = LINK_COLOR_ESTIMATE;
        estimatedRobot.chain.right_arm = FwdKin(estimatedRobot.rarm);
   end     
end

%% HEAD 
% from head V2 kinematics http://wiki.icub.org/wiki/ICubHeadV2Kinematics
% up to the tilt unit between the eyes
if HEAD_CHAIN_ON
        realRobot.head.name = 'torso-to-head';

        if TORSO_CHAIN_ON == 1
            realRobot.head.H0 = realRobot.chain.rootToTorso.RFFrame{end}; % the first frame - frame 0 - before link 0
            realRobot.head.H0(1:3,4)  = realRobot.head.H0(1:3,4)./1000; % converting the translational part from mm back to m 
        else
           error('Torso needs to be enabled in this version.');
        end
        realRobot.head.DH = [0.0,     -0.2233,    -M_PI/2.0,  -90.0*CTRL_DEG2RAD;  % from 3rd torso to first neck
                             0.0095,      0,      M_PI/2.0,    90*CTRL_DEG2RAD;
                               0,           0,      -M_PI/2.0,  -90.0*CTRL_DEG2RAD;
                            -0.059,   0.08205,    -M_PI/2.0, 90*CTRL_DEG2RAD] ;
 
        realRobot.head.Th = [joints_torso_deg(3) joints_neck_deg];
        realRobot.head.Th = realRobot.head.Th * (CTRL_DEG2RAD);
        realRobot.head.LinkColor = LINK_COLOR_REAL;

        realRobot.chain.head = FwdKin(realRobot.head);

        if ESTIMATED_ROBOT_ON   
            estimatedRobot.head = realRobot.head;
            if TORSO_CHAIN_ON
                estimatedRobot.head.H0 = estimatedRobot.chain.rootToTorso.RFFrame{end};
                estimatedRobot.head.H0(1:3,4)  = estimatedRobot.head.H0(1:3,4)./1000; % converting the translational part from mm back to m 
            else
                error('Torso needs to be enabled in this version,');
            end
            if INIT_SETTINGS_DH
            estimatedRobot.head.DH =  [-0.05,     -0.25,    -M_PI/2.0,  -90.0*CTRL_DEG2RAD;  % from 3rd torso to first neck
                                     0.0095,      0,      M_PI/2.0,    90*CTRL_DEG2RAD;
                                     0,           0,      -M_PI/2.0,  -90.0*CTRL_DEG2RAD;
                                      -0.059,   0.08205,    -M_PI/2.0, 90*CTRL_DEG2RAD] ;
            elseif ONLY_OFFSETS
                estimatedRobot.head.DH = realRobot.head.DH;
                estimatedRobot.head.DH(:,4) = [R((1 + head_start):(4 + head_start))'];
            else
                estimatedRobot.head.DH = [R((1 + head_start):(4 + head_start))',...
                    R((lR/4 + head_start + 1):(lR/4 + head_start + 4))',...
                    R((lR/2 + head_start + 1):(lR/2 + head_start + 4))',...
                    R((3*lR/4 + head_start + 1):(3*lR/4 + head_start + 4))']
            end
            estimatedRobot.head.LinkColor = LINK_COLOR_ESTIMATE;
            estimatedRobot.chain.head = FwdKin(estimatedRobot.head);
        end     

end
   
    
%% LEFT EYE
% from head V2 kinematics http://wiki.icub.org/wiki/ICubHeadV2Kinematics
if LEFT_EYE_CHAIN_ON
       if ~ (TORSO_CHAIN_ON && HEAD_CHAIN_ON)
          error('Both torso and head chains need to be on.');
       else
           realRobot.leye.name = 'tilt-to-left-eye';
           realRobot.leye.H0 = realRobot.chain.head.RFFrame{end}; % the first frame - frame 0 - before link 0
           realRobot.leye.H0(1:3,4)  = realRobot.leye.H0(1:3,4)./1000; % converting the translational part from mm back to m 
           realRobot.leye.DH = [0.0,     -0.034,    -M_PI/2.0,   0.0;  
                                0.0,      0.0,      M_PI/2.0,   -90*CTRL_DEG2RAD];
           realRobot.leye.Th = [joint_eyeTilt_deg joint_LeyePan_deg];
           realRobot.leye.Th = realRobot.leye.Th * (CTRL_DEG2RAD);
           realRobot.leye.LinkColor = LINK_COLOR_REAL;

           realRobot.chain.left_eye = FwdKin(realRobot.leye);
      
           if ESTIMATED_ROBOT_ON   
              estimatedRobot.leye = realRobot.leye;
              estimatedRobot.leye.H0 = estimatedRobot.chain.head.RFFrame{end}; % the first frame - frame 0 - before link 0
              estimatedRobot.leye.H0(1:3,4)  = estimatedRobot.leye.H0(1:3,4)./1000; % converting the translational part from mm back to m 
              if INIT_SETTINGS_DH
              estimatedRobot.leye.DH = [0.0,     -0.034,    -M_PI/2.0,   0.0;  
                                    0.0,      0.0,      M_PI/2.0,   -90*CTRL_DEG2RAD];
              elseif ONLY_OFFSETS
                estimatedRobot.leye.DH = realRobot.leye.DH
                estimatedRobot.leye.DH(:,4) = [R((1 + leye_start):(2 + leye_start))']
              else
                  estimatedRobot.leye.DH = [R((1 + leye_start):(2 + leye_start))',...
                    R((lR/4 + leye_start + 1):(lR/4 + leye_start + 2))',...
                    R((lR/2 + leye_start + 1):(lR/2 + leye_start + 2))',...
                    R((3*lR/4 + leye_start + 1):(3*lR/4 + leye_start + 2))']                  
              end
              estimatedRobot.leye.LinkColor = LINK_COLOR_ESTIMATE;
              estimatedRobot.chain.left_eye = FwdKin(estimatedRobot.leye);
           end     
       end   
end 
    
%% RIGHT EYE
if RIGHT_EYE_CHAIN_ON
       if ~ (TORSO_CHAIN_ON && HEAD_CHAIN_ON)
          error('Both torso and head chains need to be on');
       else
           realRobot.reye.name = 'tilt-to-right-eye';
           realRobot.reye.H0 = realRobot.chain.head.RFFrame{end}; % the first frame - frame 0 - before link 0
           realRobot.reye.H0(1:3,4)  = realRobot.reye.H0(1:3,4)./1000; % converting the translational part from mm back to m 
           realRobot.reye.DH = [0.0,     0.034,    -M_PI/2.0,   0.0;  
                      0.0,      0.0,      M_PI/2.0,   -90*CTRL_DEG2RAD];
           realRobot.reye.Th = [joint_eyeTilt_deg joint_ReyePan_deg];
           realRobot.reye.Th = realRobot.reye.Th * (CTRL_DEG2RAD);
           realRobot.reye.LinkColor = LINK_COLOR_REAL;

           realRobot.chain.right_eye = FwdKin(realRobot.reye);
           
           if ESTIMATED_ROBOT_ON   
              estimatedRobot.reye = realRobot.reye;
              estimatedRobot.reye.H0 = estimatedRobot.chain.head.RFFrame{end}; % the first frame - frame 0 - before link 0
              estimatedRobot.reye.H0(1:3,4)  = estimatedRobot.reye.H0(1:3,4)./1000; % converting the translational part from mm back to m 
              if INIT_SETTINGS_DH
              estimatedRobot.reye.DH = [0.0,     -0.034,    -M_PI/2.0,   0.0;  
                                    0.0,      0.0,      M_PI/2.0,   -90*CTRL_DEG2RAD];
              elseif ONLY_OFFSETS
                estimatedRobot.reye.DH = realRobot.reye.DH;
                estimatedRobot.reye.DH(:,4) = [R((1 + reye_start):(2 + reye_start))'];
              else
                  estimatedRobot.reye.DH = [R((1 + reye_start):(2 + reye_start))',...
                    R((lR/4 + reye_start + 1):(lR/4 + reye_start + 2))',...
                    R((lR/2 + reye_start + 1):(lR/2 + reye_start + 2))',...
                    R((3*lR/4 + reye_start + 1):(3*lR/4 + reye_start + 2))']                  
              end
              estimatedRobot.reye.LinkColor = LINK_COLOR_ESTIMATE;
              estimatedRobot.chain.right_eye = FwdKin(estimatedRobot.reye);
           end 
       end           
end
view(3);

%VISUALISATION of arms eef projected to 2D eye frame
if VIS_PROJ_ARMS_EEF_ON
    if ~ (TORSO_CHAIN_ON && HEAD_CHAIN_ON)
          error('Both torso and head chains need to be on');        
    else
        if ~ (RIGHT_ARM_CHAIN_ON || LEFT_ARM_CHAIN_ON)
            warning('nothing to visualise, turn on some arm chains')
        elseif ~ (RIGHT_EYE_CHAIN_ON || LEFT_EYE_CHAIN_ON)
            warning('at least one eye has to be turn on to see some projection')        
        end
        li = 1;
        sizeP = 20;%size of point in scatter plot
        colors = [1 0 0;0 1 0];%colors for points in scatter plot
        figure('Name','Projection to 2D')            
        view(2)
                hr1 = subplot(2,1,1);
                    axis([-VIS_PAR.RetS,VIS_PAR.RetS,-VIS_PAR.RetS,VIS_PAR.RetS]);hold on;
%                    legend('right arm - s','left arm - s')
                    title('Right eye');
                hr2 = subplot(2,1,2);
                    axis([-VIS_PAR.RetS,VIS_PAR.RetS,-VIS_PAR.RetS,VIS_PAR.RetS]);hold on;
                    title('Left eye');       
        if RIGHT_ARM_CHAIN_ON
            %right arm end effector projection to 2D
            X_RA = (realRobot.chain.right_arm.RFFrame{end})*[0 0 0 1]';
            [DL, DR] = compute_point(X_RA,VIS_PAR,realRobot);
            X_RA2D = [DL' DR'];
    %plot results
            set(hr1, 'NextPlot','add');
            scatter(X_RA2D(1,2), X_RA2D(2,2),sizeP,'red','o','MarkerFacecolor','red','DisplayName','right arm - s','parent',hr1);hold on;                            
            set(hr2, 'NextPlot','add');
            scatter(X_RA2D(1,1), X_RA2D(2,1),sizeP,'red','o','MarkerFacecolor','red','DisplayName','right arm - s','parent',hr2);hold on;
            legendInfo{li} = 'right arm - real';
            li = li +1;
        end
        if  LEFT_ARM_CHAIN_ON            
            %left arm end effector projection to 2D
            X_LA = (realRobot.chain.left_arm.RFFrame{end})*[0 0 0 1]';            
            [DL, DR] = compute_point(X_LA,VIS_PAR,realRobot);            
            X_LA2D = [DL' DR']; 
            set(hr1, 'NextPlot','add');            
            scatter(X_LA2D(1,2), X_LA2D(2,2),sizeP,'green','o','MarkerFacecolor','g','DisplayName','left arm - est','parent',hr1);hold on;
            set(hr2, 'NextPlot','add');
            scatter(X_LA2D(1,1), X_LA2D(2,1),sizeP,'g','o','MarkerFacecolor','g','DisplayName','left arm - est','parent',hr2);hold on;                
            legendInfo{li} = 'left arm - real';
            li = li +1;
        end
        if ESTIMATED_ROBOT_ON   
            if  RIGHT_ARM_CHAIN_ON                        
                %right arm end effector projection to 2D
                X_RA = estimatedRobot.chain.right_arm.RFFrame{end}*[0 0 0 1]';
                [DL, DR] = compute_point(X_RA,VIS_PAR,realRobot);
                Xe_RA2D = [DL' DR'];                            
                set(hr1, 'NextPlot','add');
                scatter(Xe_RA2D(1,2), Xe_RA2D(2,2),sizeP,[0.7 0 0],'h','MarkerFacecolor',[0.7 0 0],'DisplayName','right arm - est','parent',hr1);hold on;                
                line([X_RA2D(1,2) Xe_RA2D(1,2)],[X_RA2D(2,2) Xe_RA2D(2,2)],'parent',hr1);hold on;
                set(hr2, 'NextPlot','add');
                scatter(Xe_RA2D(1,1), Xe_RA2D(2,1),sizeP,[0.7 0 0],'h','MarkerFacecolor',[0.7 0 0],'DisplayName','right arm - est','parent',hr2);hold on;
                line([X_RA2D(1,1) Xe_RA2D(1,1)],[X_RA2D(2,1) Xe_RA2D(2,1)],'parent',hr2);hold on;
                legendInfo{li} = 'right arm - est';
                li = li +1;
            end
            if LEFT_ARM_CHAIN_ON
                %left arm end effector projection to 2D
                X_LA = estimatedRobot.chain.left_arm.RFFrame{end}*[0 0 0 1]' ;           
                [DL, DR] = compute_point(X_LA,VIS_PAR,realRobot);
                Xe_LA2D = [DL' DR'];
                set(hr1, 'NextPlot','add');            
                line([X_LA2D(1,2) Xe_LA2D(1,2)],[X_LA2D(2,2) Xe_LA2D(2,2)],'parent',hr1);hold on;
                scatter(Xe_LA2D(1,2), Xe_LA2D(2,2),sizeP,[0 0.7 0],'h','MarkerFacecolor',[0 0.7 0],'DisplayName','left arm - est','parent',hr1);hold on;
                set(hr2, 'NextPlot','add');
                line([X_LA2D(1,1) Xe_LA2D(1,1)],[X_LA2D(2,1) Xe_LA2D(2,1)],'parent',hr2);hold on;
                scatter(Xe_LA2D(1,1), Xe_LA2D(2,1),sizeP,[0 0.7 0],'h','MarkerFacecolor',[0 0.7 0],'DisplayName','left arm - est','parent',hr2);hold on;                   
                legendInfo{li} = 'left arm - est';
                li = li +1;
            end
            %show actual legend for a plot
            legend(hr1,legendInfo);
            legend(hr2,legendInfo);       
        end
    end
end
if SHOW_OPTIMIZATION_ERROR
    if ~ ESTIMATED_ROBOT_ON
        error('Cannot compute error when estimated robot is not on');
    else
        estimation_error = optim_error(realRobot, estimatedRobot,VIS_PAR,TORSO_CHAIN_ON, LEFT_ARM_CHAIN_ON,...
        RIGHT_ARM_CHAIN_ON,HEAD_CHAIN_ON,LEFT_EYE_CHAIN_ON,RIGHT_EYE_CHAIN_ON);
    end
end

axis equal;
    