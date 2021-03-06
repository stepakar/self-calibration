% Edited by Alessandro Roncone
% Genova Oct 2013
% changes Matej Hoffmann, July 2017
function [chain] = FwdKin(body_part,varargin)
% This function computes and displays the forward kinematics of a body part
% INPUT
%   body_part (struct) - the body part under consideration. Each body part has the same structure:
%     name = the name of the body_part;
%     H0   = is the roto-translation matrix in the origin of the chain (if the body part is attached
%            to another one, typically the last reference frame of the previous body part goes here)
%     DH   = it's the parameter matrix. Each row has 4 DH parameters (a, d, alpha, offset), thus each
%            row completely describes a link. The more rows are added, the more links are attached.
%     Th   = it's the joint values vector (as read from the encoders)
%  This function is expecting input (translations in the H0 and lengths and angles in DH) in SI units (meters and radians).
%  However, the lengths are converted to mm here and for the drawing of reference frames.
%
%   varargin - "noFrames" = do not draw any frames
%
% OUTPUT
%   chain (struct) - the resulted chain with everything inside it. It's divided by body parts.

    %% MISC STUFF
        ljnt  = 7;               % joint pic length
        rjnt  = 2;               % joint pic radius
        linkratio = 1/15;        % link dimension ratio

        LinkColor  = body_part.LinkColor;
        linkTransparency = 0.2;

        JntColor   = [.7 .7 .7];  % RGB color of the joints
        
        refFrameSize = 20;

    %% PARAMETERS

        H0  = body_part.H0();
        H0(1:3,4)  = H0(1:3,4).*1000; % converting translational part to mm

        DH  = body_part.DH;
        DH(:,1:2)  = DH(:,1:2).*1000; % converting lengths (a and d) to mm

        theta = body_part.Th';

        %%% TOTAL D-H;
        a    = DH(:,1);
        d    = DH(:,2);
        alph = DH(:,3);
        offs = DH(:,4);

        %%% THETAS!
        thet = theta + offs;
    
        %%% CHAIN
        RTMat    = cell(1,length(a)+1);
        RFFrame  = cell(1,length(a)+1);
        cyl      = cell(1,length(a)+1);

        RFFrame{1} = H0;
        RTMat{1}   = H0;

        for i = 1:length(a)
            % i-th link roto-translation
            RTMat{i+1} = evalDHMatrix ( a(i), d(i), alph(i), thet(i));
            % from root to i-th link
            RFFrame{i+1} = RFFrame{i} * RTMat{i+1};
        end

        % Draw the stuff (joints, ref frames, links)
        for i = 1:length(RFFrame)
            DrawCylinder(ljnt, rjnt, RFFrame{i} * [1 0 0 0; 0 1 0 0; 0 0 1 -ljnt/2; 0 0 0 1], JntColor, 100, 0.8);
        end
        
        if ~((nargin >=2) && strcmp(varargin{1},'noFrames') ) 
            DrawRefFrame(RFFrame{1},1,refFrameSize,'hat',body_part.name);
            if length(RFFrame) >=2
                for i = 2:length(RFFrame)
                    DrawRefFrame(RFFrame{i},i,refFrameSize,'noh',body_part.name);
                end
            end  
        end
        for i = 1:length(RFFrame)-1
            cyl{i} = DrawCylinderFromTo(RFFrame{i}(1:3,4),RFFrame{i+1}(1:3,4), LinkColor, 100, linkTransparency, linkratio);
        end

        view(3);

    %%%%%%%%%%%%%%
    % FINAL STUFF - CREATE THE OUTPUT DATA FILE (I.E. CHAIN)
    %%%%%%%%%%%%%%
        chain.name = body_part.name;
        chain.H0   = body_part.H0;
        chain.DH   = body_part.DH;
        chain.Th   = body_part.Th;

        chain.a    = a;
        chain.d    = d;
        chain.alph = alph;
        chain.offs = offs;

        chain.RTMat    = RTMat;
        chain.RFFrame = RFFrame;
end