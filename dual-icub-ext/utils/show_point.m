function show_point(X,pars,self)
    WholeBodyFwdKin_Touch;hold on;
    %inversion because we need the transformation from eye to 0th
    %frame (multiplied by TR0 so we get transformation from eye to
    %root)
    EyeL=inv(T_Ro0*T_0n)*X';
    EyeR=inv(T_Ro0*Tp_0n)*X';
    %2D coordinates of a projected point
    DL = -pars.f/EyeL(3)*[EyeL(1) EyeL(2)];%left eye
    DR = -pars.f/EyeR(3)*[EyeR(1) EyeR(2)];%right eye    
    scatter3(X(1),X(2),X(3),'r');hold on;
    scatter3(X(1),X(2),X(3),'b')
    figure(3)
    subplot(2,1,1)
    scatter(DL(1),DL(2))
    axis([-pars.RetS,pars.RetS,-pars.RetS,pars.RetS])
    subplot(2,1,2)
    scatter(DR(1),DR(2))
    axis([-pars.RetS,pars.RetS,-pars.RetS,pars.RetS])
end