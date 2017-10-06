%   This script visualises pairs of points from model and real robot in 3D
%   and 2D and transforms them to 2D.
%       T_0n:  rigid transformation from the 0th reference frame of the DH
%       notation to the last reference frame of the DH notation (right eye)
%       Tp_0n: rigid transformation from the 0th reference frame of the DH
%       notation to the last reference frame of the DH notation (left eye)
%       T_Ro0: transformation from the root to 0th ref.frame of the DH not.
% iCub Root XYZ: -0.28 0 0.15 
% Right arm -44.508147 23.429129 49.529941 75.007906 0.119645 0.019927 -3.904707
% Left arm: -44.053443 23.359441 49.646422 74.846729 -0.558269 0.006326 -3.871375
% iCub Root XYZ: (-0.28 -0.1 0.15)
% Left arm state: -33.427007 40.184665 33.138665 85.384457 -2.690376 -2.512371 -10.875232
% Right arm state: -64.918945 24.004996 76.207497 78.526694 39.162228 -0.185452 -5.440456
% iCub Root XYZ: (-0.28 0.1 0.15)
% Left arm state: -66.224776 23.934643 75.642779 78.652858 38.444721 -0.156411 -5.298458
% Right arm state: -57.637105 59.292179 67.055622 90.017286 37.432644 -2.073013 -11.37774
% iCub Root XYZ: (-0.28 0.0 0.25)
% Left arm state: -73.867376 30.957454 61.733904 83.172081 62.349254 -0.784177 -5.224954
% Right arm state: -76.906334 29.765079 63.080942 76.522777 38.988757 -3.204964 -4.359413
close all;
pars.f = 257
pars.VAng = 60
pars.RetS = pars.VAng*pi()*pars.f/360;%f*sqrt(3)/3;size of the retina
%INPUT DATA FROM MATEJ
Points = 1000*[-0.28 0 0.15 0.001;-0.28 -0.1 0.15 0.001;-0.28 -0.1 0.15 0.001;-0.28 0.0 0.25 0.001]
LeftArm =[-44.053443 23.359441 49.646422 74.846729 -0.558269 0.006326 -3.871375;...
    -33.427007 40.184665 33.138665 85.384457 -2.690376 -2.512371 -10.875232;...
    -66.224776 23.934643 75.642779 78.652858 38.444721 -0.156411 -5.298458;...
    -73.867376 30.957454 61.733904 83.172081 62.349254 -0.784177 -5.224954];
RightArm = [-44.508147 23.429129 49.529941 75.007906 0.119645 0.019927 -3.904707;...
    -64.918945 24.004996 76.207497 78.526694 39.162228 -0.185452 -5.440456;...
    -57.637105 59.292179 67.055622 90.017286 37.432644 -2.073013 -11.37774;...
    -76.906334 29.765079 63.080942 76.522777 38.988757 -3.204964 -4.359413];
addpath(genpath('../ICubFwdKinNew'))
wtheta = [0, 0, 0]*pi/180;
htheta = [0, 0, 0, -30, 0, 0]*pi/180;
rltheta = [0, 0, 0, 0, 0, 0]*pi/180;
lltheta = [0, 0, 0, 0, 0, 0]*pi/180;
for k = 2:2%size(RightArm,1)
    %simulated Robot visualisation and transformation matrices computed for
    %a given joint configuration
    [self.T_Ro0, self.T_0n, self.Tp_0n, self.T_RA, self.T_LA, self.J, self.Jp] = ...
        WholeBodyFwdKin_TouchF(wtheta,htheta,RightArm(k,:)*pi/180,LeftArm(k,:)*pi/180,rltheta,lltheta);
    hold on;
    %observed point of touch
    X = Points(k,:);
    %point where RA end effector actually is
    X_RA = (self.T_Ro0*self.T_RA)*[0 0 0 1]';
    %point where LA end effector actually is
    X_LA = (self.T_Ro0*self.T_LA)*[0 0 0 1]';
    %Visualise all points
    scatter3(X(1),X(2),X(3),'r');hold on;
    scatter3(X_RA(1),X_RA(2),X_RA(3),'b');hold on;
    scatter3(X_LA(1),X_LA(2),X_LA(3),'g');
    %compute 2D configuration of observed point
    self = compute_point(X,pars,self)
    X_2D{k} = [self.DL' self.DR']
    %compute 2D configuration of point where RA end effector actually is
    self = compute_point(X_RA',pars,self)    
    X_RA2D{k} = [self.DL' self.DR']
    %compute 2D configuration of point where RA end effector actually is
    self = compute_point(X_LA',pars,self)        
    X_LA2D{k} = [self.DL' self.DR']
    colors = [1 0 0;0 1 0;0 0 1];%colors for points in scatter plot
    sizeP = 20;%size of point in scatter plot
    figure(3)
    subplot(2,1,1)
        scatter([X_RA2D{k}(1,1) X_2D{k}(1,1) X_LA2D{k}(1,1)],[X_RA2D{k}(2,1) X_2D{k}(2,1)  X_LA2D{k}(2,1)],sizeP,colors,'o');hold on;
        line([X_RA2D{k}(1,1) X_2D{k}(1,1) X_LA2D{k}(1,1)],[X_RA2D{k}(2,1) X_2D{k}(2,1)  X_LA2D{k}(2,1)]);hold on;    
        axis([-pars.RetS,pars.RetS,-pars.RetS,pars.RetS])
        title('left eye')
    subplot(2,1,2)
        scatter([X_RA2D{k}(1,2) X_2D{k}(1,2) X_LA2D{k}(1,2)],[X_RA2D{k}(2,2) X_2D{k}(2,2)  X_LA2D{k}(2,2)],sizeP,colors,'o');hold on;
        line([X_RA2D{k}(1,2) X_2D{k}(1,2) X_LA2D{k}(1,2)],[X_RA2D{k}(2,2) X_2D{k}(2,2)  X_LA2D{k}(2,2)]);hold on;
        axis([-pars.RetS,pars.RetS,-pars.RetS,pars.RetS])
        title('right eye')
end