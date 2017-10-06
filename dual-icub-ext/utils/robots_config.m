%% Karla Stepanova, Aug 2017
function Robot_pars = robots_config(robot)
    %configuration file with robot parameters
    %creates structure Robot_pars with DH parameters for a given robot
    %INPUT:
    %   robot - string with a robot type
    %OUTPUT:
    %   Robot_pars - structure with robot parameters
    M_PI = pi;
    CTRL_DEG2RAD = pi/180;
    switch robot
        case 'realRobot'
            Robot_pars.tor.H0      = [0 -1  0  0;
                      0  0 -1  0;
                      1  0  0  0;
                      0  0  0  1];
            Robot_pars.tor.DH = [     0.032,      0.0,  M_PI/2.0, 0.0; % link from 1st to 2nd torso joint
                              0.0,  -0.0055,  M_PI/2.0,           -M_PI/2.0;]; % link from 2nd to 3rd torso joint 
            Robot_pars.larm.DH = [        0.0233647,  -0.1433, -M_PI/2.0,  105.0*CTRL_DEG2RAD;  % newly added to arm chain - from 3rd torso to shoulder
                           0.0,  0.10774, -M_PI/2.0,            M_PI/2.0;
                           0.0,      0.0,  M_PI/2.0,           -M_PI/2.0;
                         0.015,  0.15228, -M_PI/2.0,   75.0*CTRL_DEG2RAD;
                        -0.015,      0.0,  M_PI/2.0,                 0.0;
                          0.0,   0.1413,  M_PI/2.0,           -M_PI/2.0;    %V1:  0.0, 0.1373, M_PI/2.0, -M_PI/2.0;
                           0.0,      0.0,  M_PI/2.0,            M_PI/2.0;
                        0.0625,   -0.02598,     0.0,                 0.0 ]; %V1: 0.0625, -0.016,  0.0, 0.0 
            Robot_pars.rarm.DH =  [-0.0233647,  -0.1433,  M_PI/2.0, -105.0*CTRL_DEG2RAD  % newly added to arm chain - from 3rd torso to shoulder
                         0.0, -0.10774,  M_PI/2.0,           -M_PI/2.0;
                          0.0,      0.0, -M_PI/2.0,           -M_PI/2.0;
                       -0.015, -0.15228, -M_PI/2.0, -105.0*CTRL_DEG2RAD;
                        0.015,      0.0,  M_PI/2.0,                 0.0;
                          0.0,  -0.1413,  M_PI/2.0,           -M_PI/2.0; %V1:  0.0,  -0.1373,  M_PI/2.0,  -M_PI/2.0;
                          0.0,      0.0,  M_PI/2.0,            M_PI/2.0;
                       0.0625,    0.02598,   0.0,                M_PI ]; %V1:  0.0625,  0.016,  0.0, M_PI
            Robot_pars.head.DH = [0.0,     -0.2233,    -M_PI/2.0,  -90.0*CTRL_DEG2RAD;  % from 3rd torso to first neck
                             0.0095,      0,      M_PI/2.0,    90*CTRL_DEG2RAD;
                               0,           0,      -M_PI/2.0,  -90.0*CTRL_DEG2RAD;
                            -0.059,   0.08205,    -M_PI/2.0, 90*CTRL_DEG2RAD] ;
            Robot_pars.leye.DH = [0.0,     -0.034,    -M_PI/2.0,   0.0;  
                                0.0,      0.0,      M_PI/2.0,   -90*CTRL_DEG2RAD];
            Robot_pars.reye.DH = [0.0,     0.034,    -M_PI/2.0,   0.0;  
                      0.0,      0.0,      M_PI/2.0,   -90*CTRL_DEG2RAD];
        case 'estimatedRobotInit'
            Robot_pars.tor.H0      = [0 -1  0  0;
                      0  0 -1  0;
                      1  0  0  0;
                      0  0  0  1];
            Robot_pars.tor.DH = [     0.032,      0.0,  M_PI/2.0, 0.0; % link from 1st to 2nd torso joint
                              0.0,  -0.0055,  M_PI/2.0,           -M_PI/2.0;]; % link from 2nd to 3rd torso joint 
            Robot_pars.larm.DH = [0.03,  -0.16, -M_PI/2.0,  106*CTRL_DEG2RAD;  % newly added to arm chain - from 3rd torso to shoulder
                                   0.0,  0.15, -M_PI/2.0,    M_PI/2.0;
                                   0.0,      0.0,  M_PI/2.0,    -M_PI/2.0;
                                   0.018,  0.22, -M_PI/2.0,   75.0*CTRL_DEG2RAD;
                                   -0.012,      0.0,  M_PI/2.0,        0.0;
                                    0.0,   0.12,  M_PI/2.0,           -M_PI/2.0;    
                                    0.0,      0.0,  M_PI/2.0,            M_PI/2.0;
                                    0.07,   -0.03,     0.0,                 0.0 ];
            Robot_pars.rarm.DH =  [-0.03,  -0.1433,  M_PI/2.0, -105.0*CTRL_DEG2RAD  % newly added to arm chain - from 3rd torso to shoulder
                         0.0, -0.10774,  M_PI/2.0,           -M_PI/2.0;
                          0.0,      0.0, -M_PI/2.0,           -M_PI/2.0;
                       -0.015, -0.15228, -M_PI/2.0, -105.0*CTRL_DEG2RAD;
                        0.015,      0.0,  M_PI/2.0,                 0.0;
                          0.0,  -0.1413,  M_PI/2.0,           -M_PI/2.0; 
                          0.0,      0.0,  M_PI/2.0,            M_PI/2.0;
                       0.0625,    0.02598,   0.0,                M_PI ];
            Robot_pars.head.DH = [-0.05,     -0.25,    -M_PI/2.0,  -90.0*CTRL_DEG2RAD;  % from 3rd torso to first neck
                                     0.0095,      0,      M_PI/2.0,    90*CTRL_DEG2RAD;
                                     0,           0,      -M_PI/2.0,  -90.0*CTRL_DEG2RAD;
                                      -0.059,   0.08205,    -M_PI/2.0, 90*CTRL_DEG2RAD] ;
            Robot_pars.leye.DH = [0.0,     -0.034,    -M_PI/2.0,   0.0;  
                                    0.0,      0.0,      M_PI/2.0,   -90*CTRL_DEG2RAD];
            Robot_pars.reye.DH = [0.0,     -0.034,    -M_PI/2.0,   0.0;  
                                    0.0,      0.0,      M_PI/2.0,   -90*CTRL_DEG2RAD];           
    end
end