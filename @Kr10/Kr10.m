classdef Kr10 < RobotBaseClass

    properties(Access = public)              
        plyFileNameStem = 'Kr10';
    end
    
    methods
%% Define robot Function 
        function self = Kr10(baseTr)

			self.CreateModel();
            if nargin < 1			
				baseTr = eye(4);
            end
            self.model.base = self.model.base.T * baseTr  ;
            OriginalQ = [ 0         0   -0.8727         0    0.7272         0];

            % self.model.plot(zeros(1, 6));            
            self.PlotAndColourRobot(); 
            self.model.teach(OriginalQ);

        end

%% Create the robot model
        
        
        function CreateModel(self)   

            link(1) = Link('d',0.4,'a',0,'alpha',pi/2,'qlim',deg2rad([-180 180]), 'offset',0);
            link(2) = Link('d',0,'a',0.550,'alpha',0,'qlim',deg2rad([-90 90]), 'offset',pi/2);
            link(3) = Link('d',0,'a',-0.025,'alpha',-pi/2,'qlim', deg2rad([-50 150]), 'offset',pi);
            link(4) = Link('d',0.515,'a',0,'alpha',pi/2,'qlim', deg2rad([-180 180]), 'offset', pi);
            link(5) = Link('d',0,'a',0,'alpha',-pi/2,'qlim',deg2rad([-100 100]),'offset', 0);
            link(6) = Link('d',0.09,'a',0,'alpha',0,'qlim',deg2rad([-180 180]),'offset', 0);


            self.model = SerialLink(link,'name',self.name);
            OriginalQ = [ 0         0   -0.8727         0    0.7272         0];
            self.homeQ = OriginalQ;
        end

     
    end
end