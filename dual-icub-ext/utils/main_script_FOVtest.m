%% Karla Stepanova, June 2017

%this script goes through predefined points in 3D space and checks whether
%they are within the field of view of left, right and both eyes of iCub
%
%       T_0n:  rigid transformation from the 0th reference frame of the DH
%       notation to the last reference frame of the DH notation (right eye)
%       Tp_0n: rigid transformation from the 0th reference frame of the DH
%       notation to the last reference frame of the DH notation (left eye)
%       T_Ro0: transformation from the root to 0th ref.frame of the DH not.
wtheta = [0, 0, 0]*pi/180;
htheta = [0, 0, 0, 0, -30, 30]*pi/180;
k = 1;
l = 1;
r = 1;
f = 257;%focal length - for real robot obtained from calibration from calibration toolbox, e.g. 1800mm
VAng = 60;%angle of view
RetS = VAng*pi()*f/360;%f*sqrt(3)/3;size of the retina
for x = -1000:50:0
   for y = -1000:50:1000
       for z = -1000:50:1000
 % x = -30;y = 230;z = -80 
  disp([num2str(l),' ',num2str(r),' ',num2str(k),' ','x: ',num2str(x),', y: ',num2str(y),', z: ',num2str(z)])
            xeye = 0;yeye =0;
            point3 = [x y z];
            %3D point in homogenous coordinates
            P_w =[point3 1];
            %P_w = [  -320.6575  320.8000  112.6049    1.0000];
            %inversion because we need the transformation from eye to 0th
            %frame (multiplied by TR0 so we get transformation from eye to
            %root)
            EyeL=(inv(realRobot.chain.left_eye.RFFrame{end}))*P_w';
            EyeR=(inv(realRobot.chain.left_eye.RFFrame{end}))*P_w';
            %2D coordinates of a projected point
            DL = -f/EyeL(3)*[EyeL(1) EyeL(2)];%left eye
            DR = -f/EyeR(3)*[EyeR(1) EyeR(2)];%right eye
            %check whether the 3D point is within field of view of left eye
            if and(and(DL(1)<RetS,DL(1)>-RetS),and(DL(2)<RetS,DL(2)>-RetS))
                disp('left eye ok');
                xeye = 1;
                eye.coordLeye(l,1)=x;
                eye.coordLeye(l,2) = y;
                eye.coordLeye(l,3) = z;
                eye.coordLeye(l,4)=EyeL(1);
                eye.coordLeye(l,5) = EyeL(2);
                eye.coordLeye(l,6) = EyeL(3);
                l = l + 1;
            end
            %check whether the 3D point is within field of view of right eye
            if and(and(DR(1)<RetS,DR(1)>-RetS),and(DR(2)<RetS,DR(2)>-RetS))
                disp('right eye ok');   
                yeye = 1;
                eye.coordReye(r,1)=x;
                eye.coordReye(r,2) = y;
                eye.coordReye(r,3) = z;
                eye.coordReye(r,4)=EyeR(1);
                eye.coordReye(r,5) = EyeR(2);
                eye.coordReye(r,6) = EyeR(3);
                r = r + 1;
            end
            %check whether the 3D point is within field of view of both
            %eyes
            if and(xeye,yeye)
                disp('both eyes ok')
                eye.coord(k,1) = x;
                eye.coord(k,2) = y;
                eye.coord(k,3) = z;
                eye.coord(k,4) = EyeL(1);
                eye.coord(k,5) = EyeL(2);
                eye.coord(k,6) = EyeL(3);
                eye.coord(k,7) = EyeR(1);
                eye.coord(k,8) = EyeR(2);
                eye.coord(k,9) = EyeR(3);
                k = k + 1;
            end
    end
    end
end
%save('eye.mat','eye','f','VAng','RetS')
field_of_view_vis()
%field_of_view_vis.m - visualize data