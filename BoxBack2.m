function BoxBack2 (robot,vert,item,n,vertItem,item2)

OriginalQ = [0         0   -0.8727         0    0.7272         0];

QFunc2 = [1.5708   -0.4581    0.0727         0   -1.7453         0];

QItem2 = [1.5708   -0.4398    0.2793         0   -1.7453         0];

QMain2 = [1.5708    0.4581   -0.5091         0   -1.6639         0];

QMain = [-1.5708    0.4581   -0.5091         0   -1.6639         0];

QDest = [-1.5708   -0.6545    0.6545   -0.0000   -1.5661   0];

BoxesL = [0.8 -0.023 0.93;        0 0.77 0.93;        -0.8 -0.023 0.93];
LastLoc= transl(BoxesL(2,1),BoxesL(2,2),BoxesL(2,3));

t = 5;                                                                      % Total time (s)
deltaT = 0.010;                                                             % Control frequency
steps = t/deltaT;                                                           % No. of steps for simulation
delta = -pi/(2*steps);                                                      % Small angle change
epsilon = 0.1;                                                              % Threshold value for manipulability/Damped Least Squares
W = diag([1 1 1 0.1 0.1 0.1]);

m = zeros(steps,1);                                                         % Array for Measure of Manipulability
qMatrix = zeros(steps,6);                                                   % Array for joint anglesR
qdot = zeros(steps,6);                                                      % Array for joint velocities
theta = zeros(3,steps);                                                     % Array for roll-pitch-yaw angles
x = zeros(3,steps);                                                         % Array for x-y-z trajectory
positionError = zeros(3,steps);                                             % For plotting trajectory error
angleError = zeros(3,steps);

start_point = [0, -1.041, 0.6013]
end_point = [0, -0.8502, 0.9513]

switch n
    case 1
        s = lspb(0,1,steps);                                                        % Trapezoidal trajectory scalar
        for i = 1:steps
            x(1,i) = start_point(1) + s(i) * (end_point(1) - start_point(1));
            x(2,i) = start_point(2) + s(i) * (end_point(2) - start_point(2));
            x(3,i) = start_point(3) + s(i) * (end_point(3) - start_point(3));                                                           % Points in z
            theta(1,i) = 0;                                                         % Roll angle
            theta(2,i) = 0;                                                         % Pitch angle
            theta(3,i) = 0;
        end

        T = [rpy2r(theta(1,1),theta(2,1),theta(3,1)) x(:,1);zeros(1,3) 1];          % Create transformation of first point and angle
        q0 = QDest;                                                            % Initial guess for joint angles
        qMatrix(1,:) = robot.model.ikcon(T,q0);

        for i = 1:steps-1
            T = robot.model.fkine(qMatrix(i,:)).T;                                  % Get forward transformation at current joint state
            deltaX = x(:,i+1) - T(1:3,4);                                         	% Get position error from next waypoint
            Rd = rpy2r(theta(1,i+1),theta(2,i+1),theta(3,i+1));                     % Get next RPY angles, convert to rotation matrix
            Ra = T(1:3,1:3);                                                        % Current end-effector rotation matrix
            Rdot = (1/deltaT)*(Rd - Ra);                                            % Calculate rotation matrix error
            S = Rdot*Ra';                                                           % Skew symmetric!
            linear_velocity = (1/deltaT)*deltaX;
            angular_velocity = [S(3,2);S(1,3);S(2,1)];                              % Check the structure of Skew Symmetric matrix!!
            deltaTheta = tr2rpy(Rd*Ra');                                            % Convert rotation matrix to RPY angles
            xdot = W*[linear_velocity;angular_velocity];                          	% Calculate end-effector velocity to reach next waypoint.
            J = robot.model.jacob0(qMatrix(i,:));                                   % Get Jacobian at current joint state
            m(i) = sqrt(det(J*J'));
            if m(i) < epsilon                                                       % If manipulability is less than given threshold
                lambda = (1 - m(i)/epsilon)*5E-2;
            else
                lambda = 0;
            end
            invJ = inv(J'*J + lambda *eye(6))*J';                                   % DLS Inverse
            qdot(i,:) = (invJ*xdot)';                                               % Solve the RMRC equation (you may need to transpose the         vector)
            for j = 1:6                                                             % Loop through joints 1 to 6
                if qMatrix(i,j) + deltaT*qdot(i,j) < robot.model.qlim(j,1)          % If next joint angle is lower than joint limit...
                    qdot(i,j) = 0;                                                  % Stop the motor
                elseif qMatrix(i,j) + deltaT*qdot(i,j) > robot.model.qlim(j,2)      % If next joint angle is greater than joint limit ...
                    qdot(i,j) = 0;                                                  % Stop the motor
                end
            end
            qMatrix(i+1,:) = qMatrix(i,:) + deltaT*qdot(i,:);                       % Update next joint state based on joint velocities
            positionError(:,i) = x(:,i+1) - T(1:3,4);                               % For plotting
            angleError(:,i) = deltaTheta;                                           % For plotting
        end

        for i = 1:steps
            robot.model.animate(qMatrix(i,:))
            EndEff = robot.model.fkine(robot.model.getpos()).T;
            updatedVert = (EndEff*[vert,ones(size(vert,1),1)]')';
            item.Vertices = updatedVert(:,1:3);
            drawnow();
        end

        ItemFuncMove(robot, QDest,QMain, 50, vert, item,1);
        
        ItemFuncMove(robot,QMain, QMain2, 200, vert, item,1);
        
        ItemFuncMove(robot, QMain2, QItem2, 100, vert, item,1);
        
        EndEff = LastLoc;
        updatedVert = (EndEff*[vert,ones(size(vert,1),1)]')';
        item(1).Vertices = updatedVert(:,1:3);
        drawnow();
        
        ItemFuncMove(robot,QItem2,QFunc2,200,vert, item,0);
        
        ItemFuncMove(robot,QFunc2,OriginalQ,100,vert, item,0);

    case 2
        s = lspb(0,1,steps);                                                        % Trapezoidal trajectory scalar
        for i = 1:steps
            x(1,i) = start_point(1) + s(i) * (end_point(1) - start_point(1));
            x(2,i) = start_point(2) + s(i) * (end_point(2) - start_point(2));
            x(3,i) = start_point(3) + s(i) * (end_point(3) - start_point(3));                                                           % Points in z
            theta(1,i) = 0;                                                         % Roll angle
            theta(2,i) = 0;                                                         % Pitch angle
            theta(3,i) = 0;
        end

        T = [rpy2r(theta(1,1),theta(2,1),theta(3,1)) x(:,1);zeros(1,3) 1];          % Create transformation of first point and angle
        q0 = QDest;                                                            % Initial guess for joint angles
        qMatrix(1,:) = robot.model.ikcon(T,q0);

        for i = 1:steps-1
            T = robot.model.fkine(qMatrix(i,:)).T;                                  % Get forward transformation at current joint state
            deltaX = x(:,i+1) - T(1:3,4);                                         	% Get position error from next waypoint
            Rd = rpy2r(theta(1,i+1),theta(2,i+1),theta(3,i+1));                     % Get next RPY angles, convert to rotation matrix
            Ra = T(1:3,1:3);                                                        % Current end-effector rotation matrix
            Rdot = (1/deltaT)*(Rd - Ra);                                            % Calculate rotation matrix error
            S = Rdot*Ra';                                                           % Skew symmetric!
            linear_velocity = (1/deltaT)*deltaX;
            angular_velocity = [S(3,2);S(1,3);S(2,1)];                              % Check the structure of Skew Symmetric matrix!!
            deltaTheta = tr2rpy(Rd*Ra');                                            % Convert rotation matrix to RPY angles
            xdot = W*[linear_velocity;angular_velocity];                          	% Calculate end-effector velocity to reach next waypoint.
            J = robot.model.jacob0(qMatrix(i,:));                                   % Get Jacobian at current joint state
            m(i) = sqrt(det(J*J'));
            if m(i) < epsilon                                                       % If manipulability is less than given threshold
                lambda = (1 - m(i)/epsilon)*5E-2;
            else
                lambda = 0;
            end
            invJ = inv(J'*J + lambda *eye(6))*J';                                   % DLS Inverse
            qdot(i,:) = (invJ*xdot)';                                               % Solve the RMRC equation (you may need to transpose the         vector)
            for j = 1:6                                                             % Loop through joints 1 to 6
                if qMatrix(i,j) + deltaT*qdot(i,j) < robot.model.qlim(j,1)          % If next joint angle is lower than joint limit...
                    qdot(i,j) = 0;                                                  % Stop the motor
                elseif qMatrix(i,j) + deltaT*qdot(i,j) > robot.model.qlim(j,2)      % If next joint angle is greater than joint limit ...
                    qdot(i,j) = 0;                                                  % Stop the motor
                end
            end
            qMatrix(i+1,:) = qMatrix(i,:) + deltaT*qdot(i,:);                       % Update next joint state based on joint velocities
            positionError(:,i) = x(:,i+1) - T(1:3,4);                               % For plotting
            angleError(:,i) = deltaTheta;                                           % For plotting
        end

        for i = 1:steps
            robot.model.animate(qMatrix(i,:))
            EndEff = robot.model.fkine(robot.model.getpos()).T;
            updatedVert = (EndEff*[vert,ones(size(vert,1),1)]')';
            item.Vertices = updatedVert(:,1:3);
            drawnow();
            EndEffItem1 = robot.model.fkine(robot.model.getpos()).T * transl(0,0,0.1);
            updatedVertItem1 = (EndEffItem1*[vertItem,ones(size(vertItem,1),1)]')';
            item2.Vertices = updatedVertItem1(:,1:3);
            drawnow();
        end   

        ItemFuncMove(robot, QDest,QMain, 50, vert, item,3, vertItem, item2);

        ItemFuncMove(robot,QMain, QMain2, 200, vert, item,3, vertItem, item2);
        
        ItemFuncMove(robot, QMain2, QItem2, 100, vert, item,3, vertItem, item2);
        
        EndEff = LastLoc;
        updatedVert = (EndEff*[vert,ones(size(vert,1),1)]')';
        item.Vertices = updatedVert(:,1:3);
        drawnow();
        EndEffItem1 = robot.model.fkine(robot.model.getpos()).T * transl(0,0,0.1);
        updatedVertItem1 = (EndEffItem1*[vertItem,ones(size(vertItem,1),1)]')';
        item2.Vertices = updatedVertItem1(:,1:3);
        drawnow();
        
        ItemFuncMove(robot,QItem2,QFunc2,200,vert, item,0);
        
        ItemFuncMove(robot,QFunc2,OriginalQ,100,vert, item,0);
end

end