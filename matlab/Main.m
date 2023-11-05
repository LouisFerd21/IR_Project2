
close all hidden
clear all
clear classes
close all
clc
hold on
axis equal
workspace = [-2 2 -2 2 -0 4];



robot = Kr10;
basetrDobot = [eye(3), [-0.45;-1;0.65]; 0 0 0 1];
robot2 = DobotMagician (basetrDobot) ;

PlaceObject('Room.ply',[0 0.05 0]);
PlaceObject('Table.ply', [-0.85,-1.23,0.65]);
PlaceObject('Table.ply', [0.85,-1.23,0.65]);
PlaceObject('Person.ply', [0,-2,0]);
PlaceObject('FireExthing.ply',[1,-2,0]);
PlaceObject ('estop.ply',[0.55,-1.4,0.67]);
PlaceObject ('fence.ply',[0,-1.65,0]);


[f,vert] = plyread('Box.PLY','tri');
BoxesL = [0.8 -0.023 0.93;        0 0.77 0.93;        -0.8 -0.023 0.93];

for i=1:3
    Box(i) = trisurf(f,vert(:,1)+BoxesL(i,1),vert(:,2)+BoxesL(i,2),vert(:,3)+BoxesL(i,3));
    BoxSLoc(:,:,i) = transl(BoxesL(i,1),BoxesL(i,2),BoxesL(i,3));
end

[fItem1,vertItem1,data] = plyread('Rubics.ply','tri');
tipeItem1 = 1;
RubicLoc = [-0.75  -0.85  0.7];
vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;

Item(1) = trisurf(fItem1,vertItem1(:,1)+ RubicLoc(1),vertItem1(:,2)+ RubicLoc(2) ,vertItem1(:,3)+ RubicLoc(3) ...
    ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
ItemLoc(:,:,1) = transl(RubicLoc);

[fItem2,vertItem2,data] = plyread('Bottle.ply','tri');
tipeItem2 = 3;
BottleLoc = [-0.75  -1.15  0.72];
vertexColours2 = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;

Item(2) = trisurf(fItem2,vertItem2(:,1)+ BottleLoc(1),vertItem2(:,2)+ BottleLoc(2) ,vertItem2(:,3)+ BottleLoc(3) ...
    ,'FaceVertexCData',vertexColours2,'EdgeColor','interp','EdgeLighting','flat');
ItemLoc(:,:,2) = transl(BottleLoc);

q0 = [-1.5708    0.0873    0.5345    2.4871         0];
qItemLoc1 = robot2.model.ikcon(ItemLoc(:,:,1),q0);
qItemLoc2 = robot2.model.ikcon(ItemLoc(:,:,2),q0);

    % 
    % % fig = figure('Name', 'Emergency Stop Button', 'Position', [100, 100, 300, 100]);
    % % btn = uicontrol('Style', 'pushbutton', 'String', 'Emergency Stop', 'Position', [100, 30, 100, 40], 'Callback', @stopButtonCallback);
    % % 
    % % emergencyStop = false;
    % 
    Choice = input('Enter the Box number (1 for games, 2 for school item, 3 for miscellaneous): ');
    switch Choice
        case 1
            % if emergencyStop == false
            BoxFunc1(robot, vert, Box(1));
            % end
            input('Press Enter to continue');
            % if emergencyStop == false
            BoxBack1(robot, vert, Box(1), 1);
            % end
            pause(3);
        case 2
            BoxFunc2(robot, vert, Box(2));
            input('Press Enter to continue');
            BoxBack2(robot, vert, Box(2), 1);
            pause(3);
        case 3
            BoxFunc3(robot, vert, Box(3));
            input('Press Enter to continue');
            BoxBack3(robot, vert, Box(3), 1);
            pause(3);
    end

    
        %% ItemPickandSort
    
PickAndSort(robot,robot2,vert,Box(tipeItem1),tipeItem1,qItemLoc1,vertItem1,Item(1));
PickAndSort(robot,robot2,vert,Box(tipeItem2),tipeItem2,qItemLoc2,vertItem2,Item(2));

% function stopButtonCallback(~, ~)
%         % Set the emergency stop flag to true
%         emergencyStop = true;
%         disp('Emergency stop button pressed.');
%     end
