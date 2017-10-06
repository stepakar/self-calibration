function errorO = optim_error(realRobot, estimatedRobot,VIS_PAR,TORSO_CHAIN_ON,LEFT_ARM_CHAIN_ON,RIGHT_ARM_CHAIN_ON,HEAD_CHAIN_ON,LEFT_EYE_CHAIN_ON,RIGHT_EYE_CHAIN_ON)
    if ~ (TORSO_CHAIN_ON && HEAD_CHAIN_ON)
          error('Both torso and head chains need to be on');        
    else
        if RIGHT_ARM_CHAIN_ON
            %right arm end effector projection to 2D
            X_RA = (realRobot.chain.right_arm.RFFrame{end})*[0 0 0 1]';
            [DL DR] = compute_point(X_RA,VIS_PAR,realRobot);
            X_RA2D = [DL' DR'];
            %right arm end effector projection to 2D
            X_RA = estimatedRobot.chain.right_arm.RFFrame{end}*[0 0 0 1]';
            [DL DR] = compute_point(X_RA,VIS_PAR,realRobot);
            Xe_RA2D = [DL' DR'];
            errorO.errorRA = norm(Xe_RA2D - X_RA2D);
            disp(['Error RA: ', num2str(errorO.errorRA)])
        end
        if  LEFT_ARM_CHAIN_ON            
            %left arm end effector projection to 2D
            X_LA = (realRobot.chain.left_arm.RFFrame{end})*[0 0 0 1]'  ;          
            [DL DR] = compute_point(X_LA,VIS_PAR,realRobot);            
            X_LA2D = [DL' DR']; 
            %left arm end effector projection to 2D
            X_LA = estimatedRobot.chain.left_arm.RFFrame{end}*[0 0 0 1]' ;           
            [DL DR] = compute_point(X_LA,VIS_PAR,realRobot);
            Xe_LA2D = [DL' DR'];
            errorO.errorLA = norm(Xe_LA2D - X_LA2D) ;           
            disp(['Error LA: ', num2str(errorO.errorLA)])
        end       
    end

