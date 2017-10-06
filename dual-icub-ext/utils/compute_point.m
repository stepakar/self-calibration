function [DL DR] = compute_point(X,VIS_PAR,robotModel)    
    %INPUT
    %   X - point to be projected
    %   VIS_PAR - parameters needed for projection: f - focal length
    %   robotModel - robotModel containing roto-translation matrices
    %inversion because we need the transformation from eye to 0th
    %frame (multiplied by TR0 so we get transformation from eye to
    %root)
    EyeL=(inv(robotModel.chain.left_eye.RFFrame{end}))*X;
    EyeR=(inv(robotModel.chain.right_eye.RFFrame{end}))*X;
    %2D coordinates of a projected point
    DL = -VIS_PAR.f/EyeL(3)*[EyeL(1) EyeL(2)];%left eye
    DR = -VIS_PAR.f/EyeR(3)*[EyeR(1) EyeR(2)];%right eye        
end