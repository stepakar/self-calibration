%% Karla Stepanova, Aug 2017

function InitGuess = ParsToInitGuess(Pars,RA,LA,REye,LEye,only_offsets)
%converts robot parameters saved in a structure defined as in configuration
%file (e.g. Robot_pars = robots_config('realRobot')) to nD vector of init
%guess for DH optimization
%INPUTS
%   Pars - robot parameters saved in a structure defined as in configuration
%       file (e.g. Robot_pars = robots_config('realRobot'))
%   LA - 1/0 whether left arm chain should be included into final InitGuess vector
%   RA - 1/0 whether right arm chain should be included into final InitGuess vector
%   LEye - 1/0 whether left eye chain should be included into final InitGuess vector
%   REye - 1/0 whether right eye chain should be included into final InitGuess vector
%   only_offsets - 1/0 whether put into init guess only offsets (1) or all DH
%   parameters
%OUTPUTS
%   InitGuess - nD vector containing DH parameters in a following order:
%       [a(1)...a(n) d(1)...d(n) alpha(1)...alpha(n) offset(1)...offset(n)]
%       order of body parts for each estimated paremeter is: 
%       [torso right_arm left_arm head right_eye left_eye]
if only_offsets
    if ~(LEye||REye) %no eyes
        if ~(LA||RA)%no hands
            InitGuess = [Pars.tor.DH(:,4)'];
        elseif ((RA)&&(~LA))%no eyes, right arm 
            InitGuess = [Pars.tor.DH(:,4)' Pars.rarm.DH(:,4)']
        elseif ((LA)&&(~RA))%no eyes, left arm
            InitGuess = [Pars.tor.DH(:,4)' Pars.larm.DH(:,4)'];
        elseif ((RA)&&(LA))%no eyes, both arms
            InitGuess = [Pars.tor.DH(:,4)' Pars.rarm.DH(:,4)' Pars.larm.DH(:,4)'];        
        end
    else %some eye is here
        if ~(LA||RA)%some eye, no arms
            if REye%right eye, no arms
                InitGuess = [Pars.tor.DH(:,4)' Pars.head.DH(:,4)' Pars.reye.DH(:,4)'];
                if LEye%both eyes, no arms
                    InitGuess = [Pars.tor.DH(:,4)' Pars.head.DH(:,4)' Pars.reye.DH(:,4)' Pars.leye.DH(:,4)'];
                end
            elseif LEye%left eye, no arms
                InitGuess = [Pars.tor.DH(:,4)' Pars.head.DH(:,4)' Pars.leye.DH(:,4)'];
            end
        elseif ((RA)&&(~LA))%some eye, right arm
            if REye%right eye, right arm
                InitGuess = [Pars.tor.DH(:,4)' Pars.rarm.DH(:,4)' Pars.head.DH(:,4)' Pars.reye.DH(:,4)'];
                if LEye%both eyes, right arms
                    InitGuess = [Pars.tor.DH(:,4)' Pars.rarm.DH(:,4)' Pars.head.DH(:,4)' Pars.reye.DH(:,4)' Pars.leye.DH(:,4)'];
                end
            elseif LEye%left eye, right arm
                InitGuess = [Pars.tor.DH(:,4)' Pars.rarm.DH(:,4)' Pars.head.DH(:,4)' Pars.leye.DH(:,4)'];
            end
        elseif ((LA)&&(~RA))%some eye, left arm
            if REye%right eye, left arm
                InitGuess = [Pars.tor.DH(:,4)' Pars.larm.DH(:,4)' Pars.head.DH(:,4)' Pars.reye.DH(:,4)'];
                if LEye%both eyes, left arm
                    InitGuess = [Pars.tor.DH(:,4)' Pars.larm.DH(:,4)' Pars.head.DH(:,4)' Pars.reye.DH(:,4)' Pars.leye.DH(:,4)'];
                end
            elseif LEye%left eye, left arm
                InitGuess = [Pars.tor.DH(:,4)' Pars.larm.DH(:,4)' Pars.head.DH(:,4)' Pars.leye.DH(:,4)'];
            end
            elseif ((RA)&&(LA))%some eye, both arms
                if REye%right eye, both arms
                    InitGuess = [Pars.tor.DH(:,4)' Pars.rarm.DH(:,4)' Pars.larm.DH(:,4)' Pars.head.DH(:,4)' Pars.reye.DH(:,4)'];
                    if LEye%both eyes, both arms
                        InitGuess = [Pars.tor.DH(:,4)' Pars.rarm.DH(:,4)' Pars.larm.DH(:,4)' Pars.head.DH(:,4)' Pars.reye.DH(:,4)' Pars.leye.DH(:,4)'];
                    end
                elseif LEye%left eye, both arms
                    InitGuess = [Pars.tor.DH(:,4)' Pars.rarm.DH(:,4)' Pars.larm.DH(:,4)' Pars.head.DH(:,4)' Pars.leye.DH(:,4)'];
            end
        end    
    end
else
    if ~(LEye||REye) %no eyes
        if ~(LA||RA)%no hands
            InitGuess = [Pars.tor.DH(:,1)' Pars.tor.DH(:,2)' Pars.tor.DH(:,3)' Pars.tor.DH(:,4)'];
        elseif ((RA)&&(~LA))%no eyes, right arm 
            InitGuess = [Pars.tor.DH(:,1)' Pars.rarm.DH(:,1)' ...
                    Pars.tor.DH(:,2)' Pars.rarm.DH(:,2)' ...
                    Pars.tor.DH(:,3)' Pars.rarm.DH(:,3)' ...
                    Pars.tor.DH(:,4)' Pars.rarm.DH(:,4)']
        elseif ((LA)&&(~RA))%no eyes, left arm
            InitGuess = [Pars.tor.DH(:,1)' Pars.larm.DH(:,1)' ...
                    Pars.tor.DH(:,2)' Pars.larm.DH(:,2)' ...
                    Pars.tor.DH(:,3)' Pars.larm.DH(:,3)' ...
                    Pars.tor.DH(:,4)' Pars.larm.DH(:,4)'];
        elseif ((RA)&&(LA))%no eyes, both arms
            InitGuess = [Pars.tor.DH(:,1)' Pars.rarm.DH(:,1)' Pars.larm.DH(:,1)' ...
                    Pars.tor.DH(:,2)' Pars.rarm.DH(:,2)' Pars.larm.DH(:,2)' ...
                    Pars.tor.DH(:,3)' Pars.rarm.DH(:,3)' Pars.larm.DH(:,3)' ...
                    Pars.tor.DH(:,4)' Pars.rarm.DH(:,4)' Pars.larm.DH(:,4)'];        
        end
    else %some eye is here
        if ~(LA||RA)%some eye, no arms
            if REye%right eye, no arms
                InitGuess = [Pars.tor.DH(:,1)' Pars.head.DH(:,1)' Pars.reye.DH(:,1)'...
                    Pars.tor.DH(:,2)' Pars.head.DH(:,2)' Pars.reye.DH(:,2)'...            
                    Pars.tor.DH(:,3)' Pars.head.DH(:,3)' Pars.reye.DH(:,3)'...
                    Pars.tor.DH(:,4)' Pars.head.DH(:,4)' Pars.reye.DH(:,4)'];
                if LEye%both eyes, no arms
                    InitGuess = [Pars.tor.DH(:,1)' Pars.head.DH(:,1)' Pars.reye.DH(:,1)' Pars.leye.DH(:,1)'...
                    Pars.tor.DH(:,2)' Pars.head.DH(:,2)' Pars.reye.DH(:,2)' Pars.leye.DH(:,2)'...            
                    Pars.tor.DH(:,3)' Pars.head.DH(:,3)' Pars.reye.DH(:,3)' Pars.leye.DH(:,3)'...
                    Pars.tor.DH(:,4)' Pars.head.DH(:,4)' Pars.reye.DH(:,4)' Pars.leye.DH(:,4)'];
                end
            elseif LEye%left eye, no arms
                InitGuess = [Pars.tor.DH(:,1)' Pars.head.DH(:,1)' Pars.leye.DH(:,1)'...
                    Pars.tor.DH(:,2)' Pars.head.DH(:,2)' Pars.leye.DH(:,2)'...            
                    Pars.tor.DH(:,3)' Pars.head.DH(:,3)' Pars.leye.DH(:,3)'...
                    Pars.tor.DH(:,4)' Pars.head.DH(:,4)' Pars.leye.DH(:,4)'];
            end
        elseif ((RA)&&(~LA))%some eye, right arm
            if REye%right eye, right arm
                InitGuess = [Pars.tor.DH(:,1)' Pars.rarm.DH(:,1)' Pars.head.DH(:,1)' Pars.reye.DH(:,1)'...
                    Pars.tor.DH(:,2)' Pars.rarm.DH(:,2)' Pars.head.DH(:,2)' Pars.reye.DH(:,2)'...            
                    Pars.tor.DH(:,3)' Pars.rarm.DH(:,3)' Pars.head.DH(:,3)' Pars.reye.DH(:,3)'...
                    Pars.tor.DH(:,4)' Pars.rarm.DH(:,4)' Pars.head.DH(:,4)' Pars.reye.DH(:,4)'];
                if LEye%both eyes, right arms
                    InitGuess = [Pars.tor.DH(:,1)' Pars.rarm.DH(:,1)' Pars.head.DH(:,1)' Pars.reye.DH(:,1)' Pars.leye.DH(:,1)'...
                    Pars.tor.DH(:,2)' Pars.rarm.DH(:,2)' Pars.head.DH(:,2)' Pars.reye.DH(:,2)' Pars.leye.DH(:,2)'...            
                    Pars.tor.DH(:,3)' Pars.rarm.DH(:,3)' Pars.head.DH(:,3)' Pars.reye.DH(:,3)' Pars.leye.DH(:,3)'...
                    Pars.tor.DH(:,4)' Pars.rarm.DH(:,4)' Pars.head.DH(:,4)' Pars.reye.DH(:,4)' Pars.leye.DH(:,4)'];
                end
            elseif LEye%left eye, right arm
                InitGuess = [Pars.tor.DH(:,1)' Pars.rarm.DH(:,1)' Pars.head.DH(:,1)' Pars.leye.DH(:,1)'...
                    Pars.tor.DH(:,2)' Pars.rarm.DH(:,2)' Pars.head.DH(:,2)' Pars.leye.DH(:,2)'...            
                    Pars.tor.DH(:,3)' Pars.rarm.DH(:,3)' Pars.head.DH(:,3)' Pars.leye.DH(:,3)'...
                    Pars.tor.DH(:,4)' Pars.rarm.DH(:,4)' Pars.head.DH(:,4)' Pars.leye.DH(:,4)'];
            end
        elseif ((LA)&&(~RA))%some eye, left arm
            if REye%right eye, left arm
                InitGuess = [Pars.tor.DH(:,1)' Pars.larm.DH(:,1)' Pars.head.DH(:,1)' Pars.reye.DH(:,1)'...
                    Pars.tor.DH(:,2)' Pars.larm.DH(:,2)' Pars.head.DH(:,2)' Pars.reye.DH(:,2)'...            
                    Pars.tor.DH(:,3)' Pars.larm.DH(:,3)' Pars.head.DH(:,3)' Pars.reye.DH(:,3)'...
                    Pars.tor.DH(:,4)' Pars.larm.DH(:,4)' Pars.head.DH(:,4)' Pars.reye.DH(:,4)'];
                if LEye%both eyes, left arm
                    InitGuess = [Pars.tor.DH(:,1)' Pars.larm.DH(:,1)' Pars.head.DH(:,1)' Pars.reye.DH(:,1)' Pars.leye.DH(:,1)'...
                    Pars.tor.DH(:,2)' Pars.larm.DH(:,2)' Pars.head.DH(:,2)' Pars.reye.DH(:,2)' Pars.leye.DH(:,2)'...            
                    Pars.tor.DH(:,3)' Pars.larm.DH(:,3)' Pars.head.DH(:,3)' Pars.reye.DH(:,3)' Pars.leye.DH(:,3)'...
                    Pars.tor.DH(:,4)' Pars.larm.DH(:,4)' Pars.head.DH(:,4)' Pars.reye.DH(:,4)' Pars.leye.DH(:,4)'];
                end
            elseif LEye%left eye, left arm
                InitGuess = [Pars.tor.DH(:,1)' Pars.larm.DH(:,1)' Pars.head.DH(:,1)' Pars.leye.DH(:,1)'...
                    Pars.tor.DH(:,2)' Pars.larm.DH(:,2)' Pars.head.DH(:,2)' Pars.leye.DH(:,2)'...            
                    Pars.tor.DH(:,3)' Pars.larm.DH(:,3)' Pars.head.DH(:,3)' Pars.leye.DH(:,3)'...
                    Pars.tor.DH(:,4)' Pars.larm.DH(:,4)' Pars.head.DH(:,4)' Pars.leye.DH(:,4)'];
            end
            elseif ((RA)&&(LA))%some eye, both arms
                if REye%right eye, both arms
                    InitGuess = [Pars.tor.DH(:,1)' Pars.rarm.DH(:,1)' Pars.larm.DH(:,1)' Pars.head.DH(:,1)' Pars.reye.DH(:,1)'...
                        Pars.tor.DH(:,2)' Pars.rarm.DH(:,2)' Pars.larm.DH(:,2)' Pars.head.DH(:,2)' Pars.reye.DH(:,2)'...            
                        Pars.tor.DH(:,3)' Pars.rarm.DH(:,3)' Pars.larm.DH(:,3)' Pars.head.DH(:,3)' Pars.reye.DH(:,3)'...
                        Pars.tor.DH(:,4)' Pars.rarm.DH(:,4)' Pars.larm.DH(:,4)' Pars.head.DH(:,4)' Pars.reye.DH(:,4)'];
                    if LEye%both eyes, both arms
                        InitGuess = [Pars.tor.DH(:,1)' Pars.rarm.DH(:,1)' Pars.larm.DH(:,1)' Pars.head.DH(:,1)' Pars.reye.DH(:,1)' Pars.leye.DH(:,1)'...
                            Pars.tor.DH(:,2)' Pars.rarm.DH(:,2)' Pars.larm.DH(:,2)' Pars.head.DH(:,2)' Pars.reye.DH(:,2)' Pars.leye.DH(:,2)'...            
                            Pars.tor.DH(:,3)' Pars.rarm.DH(:,3)' Pars.larm.DH(:,3)' Pars.head.DH(:,3)' Pars.reye.DH(:,3)' Pars.leye.DH(:,3)'...
                            Pars.tor.DH(:,4)' Pars.rarm.DH(:,4)' Pars.larm.DH(:,4)' Pars.head.DH(:,4)' Pars.reye.DH(:,4)' Pars.leye.DH(:,4)'];
                    end
                elseif LEye%left eye, both arms
                    InitGuess = [Pars.tor.DH(:,1)' Pars.rarm.DH(:,1)' Pars.larm.DH(:,1)' Pars.head.DH(:,1)' Pars.leye.DH(:,1)'...
                        Pars.tor.DH(:,2)' Pars.rarm.DH(:,2)' Pars.larm.DH(:,2)' Pars.head.DH(:,2)' Pars.leye.DH(:,2)'...            
                        Pars.tor.DH(:,3)' Pars.rarm.DH(:,3)' Pars.larm.DH(:,3)' Pars.head.DH(:,3)' Pars.leye.DH(:,3)'...
                        Pars.tor.DH(:,4)' Pars.rarm.DH(:,4)' Pars.larm.DH(:,4)' Pars.head.DH(:,4)' Pars.leye.DH(:,4)'];
            end
        end    
    end
end