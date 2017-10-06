%INITIAL GUESS OF PARAMETERS
    %PARAMETERS TO BE ESTIMATED:
    %     DH parameters   = it's the parameter matrix. Each row has 4 DH parameters (a, d, alpha, offset), thus each
    %            row completely describes a link. The more rows are added, the more links are attached.
%SET JOINT ANGLES:
%     Th   = it's the joint values vector (as read from the encoders)
%FIXED PARAMETERS:
%        H0   = is the roto-translation matrix in the origin of the chain (if the body part is attached
%            to another one, typically the last reference frame of the previous body part goes here)
%TODOS:
% Pars, InitGuess and Joint values should be setted as global variables and
% not saved and loaded
addpath('./utils/');

global Flags
Flags.RIGHT_ARM_CHAIN_ON = true;
Flags.LEFT_ARM_CHAIN_ON = true;
Flags.HEAD_CHAIN_ON = true;
Flags.LEFT_EYE_CHAIN_ON = true; % requires HEAD_CHAIN_ON == true;
Flags.RIGHT_EYE_CHAIN_ON = true; % requires HEAD_CHAIN_ON == true;
Flags.TORSO_CHAIN_ON = true;
Flags.EEF_ERROR = false;
Flags.OPTIMIZATION = true;
Flags.ONLY_OFFSETS = true;%if set to true, estimating only offsets, if set to false, estimating all DH parameters
%parameters for visualisation of 3D-2D projection
global VIS_PAR
VIS_PAR.f = 257;%focal length
VIS_PAR.VAng = 60;%field of view
VIS_PAR.RetS = VIS_PAR.VAng*pi()*VIS_PAR.f/360;%f*sqrt(3)/3;size of the retina
%save('Setting.mat','VIS_PAR','RIGHT_EYE_CHAIN_ON','LEFT_EYE_CHAIN_ON','HEAD_CHAIN_ON','RIGHT_ARM_CHAIN_ON','LEFT_ARM_CHAIN_ON')
C = load('C:\MATLAB\code-selfcalibration\dual-icub-ext\dataset\selfTouchConfigs.log');
NmbPoints = [10 20 30 40 50 100];

for inp = [4]                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            
    nmb_points = NmbPoints(inp);
    %indices of poses from dataset to be selected
    selected_poses = randi(size(C,1),nmb_points,1);
    %measured poses
    JV_torso = [C(selected_poses,4:6)];
    % JV_torso = [0.0   0.0   0.0;
    %     0.0   0.0   0.0;
    %     0.0   0.0   0.0;
    %     0.0   0.0   0.0;
    %     ]; % pitch, roll, yaw order (iKin, CAD, DH) - unlike iCub motor interfaces
    JV_LA = [C(selected_poses,7:13)];
    % JV_LA  = [-23.6988   24.5989   42.5855   66.4666    0.0391         0   -3.5190;...
    %     -33.427007 40.184665 33.138665 85.384457 -2.690376 -2.512371 -10.875232;...
    %     -66.224776 23.934643 75.642779 78.652858 38.444721 -0.156411 -5.298458;...
    %     -73.867376 30.957454 61.733904 83.172081 62.349254 -0.784177 -5.224954
    %     ];
    JV_RA = [C(selected_poses,17:23)];
    % JV_RA  = [-45.0474   23.0000   80.0000   68.1435  -88.5362  -20.6167   12.4597;...
    %     -64.918945 24.004996 76.207497 78.526694 39.162228 -0.185452 -5.440456;...
    %     -57.637105 59.292179 67.055622 90.017286 37.432644 -2.073013 -11.37774;...
    %     -76.906334 29.765079 63.080942 76.522777 38.988757 -3.204964 -4.359413
    %     ];
    %converting from common pan and vergence to pan and tilt for individual
    %eyes when loading data
    JV_eyes = [C(selected_poses,24:27) C(selected_poses,28)+C(selected_poses,29)/2 C(selected_poses,28)-C(selected_poses,29)/2];
    % JV_eyes = [0.0 0.0 0.0 -60.0 0.0 0.0;
    %     0.0 0.0 0.0 -60.0 0.0 0.0;
    %     0.0 0.0 0.0 -60.0 0.0 0.0;
    %     0.0 0.0 0.0 -60.0 0.0 0.0
    %     ];
    save('./data_files/joint_data.mat','JV_torso','JV_LA','JV_RA','JV_eyes')

    %robots parameters
    estimated_robot_Init_pars = robots_config('estimatedRobotInit');%robots_config('estimatedRobotInit');
    Real_robot_pars = robots_config('realRobot');

    %measured value for a given pose set (JV variables)
    [LA_3D, RA_3D, Leye_2D, Reye_2D,RFFrameLA, RFFrameRA,Reye_3DLA] = compute_points_3Dand2D(JV_torso,JV_LA,JV_RA,JV_eyes, Real_robot_pars,Flags,VIS_PAR);
    save('./data_files/measured_data.mat','LA_3D','RA_3D','Leye_2D','Reye_2D')
    
    if Flags.OPTIMIZATION
        %init guess for optimization
        xRR = ParsToInitGuess(Real_robot_pars,Flags.RIGHT_ARM_CHAIN_ON,Flags.LEFT_ARM_CHAIN_ON,...
            Flags.RIGHT_EYE_CHAIN_ON,Flags.LEFT_EYE_CHAIN_ON,Flags.ONLY_OFFSETS);%InitGuess = ParsToInitGuess(Pars,RA,LA,REye,LEye);
        x0 = ParsToInitGuess(estimated_robot_Init_pars,Flags.RIGHT_ARM_CHAIN_ON,Flags.LEFT_ARM_CHAIN_ON,...
            Flags.RIGHT_EYE_CHAIN_ON,Flags.LEFT_EYE_CHAIN_ON,Flags.ONLY_OFFSETS);%InitGuess = ParsToInitGuess(Pars,RA,LA,REye,LEye);

        %perform optimization
        options.Algorithm = 'levenberg-marquardt';%'trust-region-reflective'
        options.MaxIter = 20000;
        options.MaxFunEvals = 200000;
        options.StepTolerance = 1e-12;
       % [R,RESNORM,RESIDUAL,EXITFLAG,OUTPUT,LAMBDA,JACOBIAN] = lsqnonlin(@LALEyeChain,x0,[],[],options);
       lb = xRR-abs(xRR)*0.1;
       ub = xRR+abs(xRR)*0.1;
        %[R,RESNORM,RESIDUAL,EXITFLAG,OUTPUT,LAMBDA,JACOBIAN] = lsqnonlin(@LARLEyeChain,x0,lb,ub,options);
        [R,RESNORM,RESIDUAL,EXITFLAG,OUTPUT,LAMBDA,JACOBIAN] = lsqnonlin(@LARLEyeChain,x0,[],[],options);
        %visualise estimated robot
        %iCubCalib_ext_fce(DH, Flags)
        %observability
        observability = compute_observability(JACOBIAN,size(JV_LA,1));
        disp(observability)
        save('./data_files/resulting_parameters_offs.mat','R')
        Results.estimation_error(inp) = iCubCalib_ext_fce();
        Results.R{inp} = R;
        Results.ErrorLA(inp) = estimation_error.errorLA;
    end
    %computing and visualising errors of end effector for right and left
    %arm (real positions are compared to the aimed point)
    if Flags.EEF_ERROR
        figure(3)
            subplot(2,1,1)
            Results.error.LA.x = C(selected_poses,1)*1000-LA_3D(1,1:nmb_points)';
            Results.error.LA.y = C(selected_poses,2)*1000-LA_3D(2,1:nmb_points)';
            Results.error.LA.z = C(selected_poses,3)*1000-LA_3D(3,1:nmb_points)';
            hist([Results.error.LA.x Results.error.LA.y Results.error.LA.z]);
            legend('error_x','error_y','error_z','Location','northwest');
            xlabel('Error [mm]')
            title('Errors on LA')
        subplot(2,1,2)         
            Results.error.RA.x = C(selected_poses,1)*1000-RA_3D(1,1:nmb_points)';
            Results.error.RA.y = C(selected_poses,2)*1000-RA_3D(2,1:nmb_points)';
            Results.error.RA.z = C(selected_poses,3)*1000-RA_3D(3,1:nmb_points)';
            hist([Results.error.RA.x Results.error.RA.y Results.error.RA.z]);
            legend('error_x','error_y','error_z','Location','northwest');
            xlabel('Error [mm]')
            title('Errors on RA')
        Results.error.LA.mse = sqrt(sum(((C(1:nmb_points,1:3)*1000-LA_3D(1:3,1:nmb_points)').^2),2));
        Results.error.PA.mse = sqrt(sum(((C(1:nmb_points,1:3)*1000-RA_3D(1:3,1:nmb_points)').^2),2));
    end
end