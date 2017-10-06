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
%parameters for visualisation of 3D-2D projection
global VIS_PAR
VIS_PAR.f = 257;%focal length
VIS_PAR.VAng = 60;%field of view
VIS_PAR.RetS = VIS_PAR.VAng*pi()*VIS_PAR.f/360;%f*sqrt(3)/3;size of the retina
%save('Setting.mat','VIS_PAR','RIGHT_EYE_CHAIN_ON','LEFT_EYE_CHAIN_ON','HEAD_CHAIN_ON','RIGHT_ARM_CHAIN_ON','LEFT_ARM_CHAIN_ON')

%Changed by Nikita
%measured poses
JV_torso = [];
JV_LA = [];
JV_RA = [];
JV_eyes = [];
startOfReading = 1;
finishOfReading = 10;
for i = startOfReading : 1 : finishOfReading
    [C1, C2, C3, C4, C5, C6, C7, C8, C9, C10, C11, C12, C13, C14, C15, C16, C17, C18, C19, C20, C21, C22, C23, C24, C25, C26, C27, C28, C29] = getVector(i);
    JV_torso = [JV_torso ; C4 C5 C6];
    JV_LA = [JV_LA ; C7 C8 C9 C10 C11 C12 C13];
    JV_RA = [JV_RA ; C17 C18 C19 C20 C21 C22 C23];
    JV_eyes = [JV_eyes ; C24 C25 C26 C27 C28 C29];
end

%JV_torso = [0.0   0.0   0.0;
%    0.0   0.0   0.0;
%    0.0   0.0   0.0;
%    0.0   0.0   0.0;
%    ]; % pitch, roll, yaw order (iKin, CAD, DH) - unlike iCub motor interfaces
%JV_LA  = [-44.053443 23.359441 49.646422 74.846729 -0.558269 0.006326 -3.871375;...
%    -33.427007 40.184665 33.138665 85.384457 -2.690376 -2.512371 -10.875232;...
%    -66.224776 23.934643 75.642779 78.652858 38.444721 -0.156411 -5.298458;...
%    -73.867376 30.957454 61.733904 83.172081 62.349254 -0.784177 -5.224954
%    ];
%JV_RA  = [-44.508147 23.429129 49.529941 75.007906 0.119645 0.019927 -3.904707;...
%    -64.918945 24.004996 76.207497 78.526694 39.162228 -0.185452 -5.440456;...
%    -57.637105 59.292179 67.055622 90.017286 37.432644 -2.073013 -11.37774;...
%    -76.906334 29.765079 63.080942 76.522777 38.988757 -3.204964 -4.359413
%    ];
%JV_eyes = [0.0 0.0 0.0 -60.0 0.0 0.0;
%    0.0 0.0 0.0 -60.0 0.0 0.0;
%    0.0 0.0 0.0 -60.0 0.0 0.0;
%    0.0 0.0 0.0 -60.0 0.0 0.0
%    ];
save('./data_files/joint_data.mat','JV_torso','JV_LA','JV_RA','JV_eyes')

%robots parameters
estimated_robot_Init_pars = robots_config('estimatedRobotInit');
Real_robot_pars = robots_config('realRobot');

%init guess for optimization
x0 = ParsToInitGuess(estimated_robot_Init_pars,Flags.RIGHT_ARM_CHAIN_ON,Flags.LEFT_ARM_CHAIN_ON,...
    Flags.RIGHT_EYE_CHAIN_ON,Flags.LEFT_EYE_CHAIN_ON);%InitGuess = ParsToInitGuess(Pars,RA,LA,REye,LEye);

%measured value for a given pose set (JV variables)
[LA_3D, RA_3D, Leye_2D, Reye_2D] = compute_points_3Dand2D(JV_torso,JV_LA,JV_RA,JV_eyes, Real_robot_pars,Flags,VIS_PAR);
save('./data_files/measured_data.mat','LA_3D','RA_3D','Leye_2D','Reye_2D')

%perform optimization
options.Algorithm = 'levenberg-marquardt';
[R,RESNORM,RESIDUAL,EXITFLAG,OUTPUT,LAMBDA,JACOBIAN] = lsqnonlin(@LALEyeChain,x0,[],[],options);
observability = compute_observability(JACOBIAN,size(JV_LA,1));
disp(observability)
save('./data_files/resulting_parameters.mat','R')

%function for reading from dataset and storing to vector
function [a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29] = getVector(n)
fileID = fopen('selfTouchConfigs.log','r');
formatSpec = '%f';
A = fscanf(fileID,formatSpec);
fclose(fileID);
a1 = A((n - 1)*29 + 1); a2 = A((n - 1)*29 + 2); a3 = A((n - 1)*29 + 3);
a4 = A((n - 1)*29 + 4); a5 = A((n - 1)*29 + 5); a6 = A((n - 1)*29 + 6);
a7 = A((n - 1)*29 + 7); a8 = A((n - 1)*29 + 8); a9 = A((n - 1)*29 + 9);
a10 = A((n - 1)*29 + 10); a11 = A((n - 1)*29 + 11); a12 = A((n - 1)*29 + 12);
a13 = A((n - 1)*29 + 13); a14 = A((n - 1)*29 + 14); a15 = A((n - 1)*29 + 15);
a16 = A((n - 1)*29 + 16); a17 = A((n - 1)*29 + 17); a18 = A((n - 1)*29 + 18);
a19 = A((n - 1)*29 + 19); a20 = A((n - 1)*29 + 20); a21 = A((n - 1)*29 + 21);
a22 = A((n - 1)*29 + 22); a23 = A((n - 1)*29 + 23); a24 = A((n - 1)*29 + 24);
a25 = A((n - 1)*29 + 25); a26 = A((n - 1)*29 + 26); a27 = A((n - 1)*29 + 27);
a28 = A((n - 1)*29 + 28); a29 = A((n - 1)*29 + 29);
end