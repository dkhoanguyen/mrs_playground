clear all;
close all;
clc;

%% Initialise motion capture

connectionType = 0;                                         % 0:Multicast, 1:Unicast
natNetDllPath  = strcat(cd,'\CNatNetML.dll');
serverIP = '192.168.1.1';
clientIP = '192.168.1.2';

nat = CNatNet();
[nat,isInitialized] = nat.initialize(connectionType, natNetDllPath);
[nat, isConnected] = nat.connect(clientIP, serverIP);
if ~isConnected
    return;
end

% Add termination callback
C = onCleanup(@()nat.terminate());

% Get version for general debugging
[nat,version] = nat.getVersion();

% Get description for debugging
[nat,serverIP,serverName,serverAppVer] = nat.getServerDescription();

% Display debugging information
disp('-----------------------------------------');
version
serverIP
serverName
serverAppVer
disp('-----------------------------------------');

%% Initialise robots
robotId = [15,16];
robotHardwareId = [1,2];
robots = containers.Map('KeyType','double','ValueType','any');
for idx=1:numel(robotId)
    id = robotId(idx);
    hardwareID = robotHardwareId(idx);
    robot = Robot(hardwareID);
    robots(id) = robot;
end

%% Initalise flocking
dt = 0.1;
sensingRange = 1.0;
dangerRange = 0.5;
maxV = 0.05;
distance = 0.5;

flockingBehaviour = Flocking(dt, sensingRange, dangerRange, maxV, distance);

%% Update loop

% % Last column for raw id
% animalStates = zeros(numel(robotId),5);
% robotStates = zeros()
while true 
    % Update position in real time

    % Obtain frame data
    [nat, isNew, frameData] = nat.update();
    % Extract all relevant rigid bodies
    for rBIdx = 1:frameData.nRigidBodies
        % Extract data
        id = frameData.RigidBodies{rBIdx,1}.id;
        x = frameData.RigidBodies{rBIdx,1}.x;
        y = frameData.RigidBodies{rBIdx,1}.y;
        % If the current ID is the robot ID, update robot
        if isKey(robots, id)
            % Update robot position
            robots(id) = robots(id).setPosition([x,y]);
        end
    end

    % Obtain robot states
    allRobotIDs = keys(robots);
    animalStates = zeros(length(allRobotIDs), 5);
    for i=1:length(allRobotIDs)
        id = allRobotIDs{i};
        state = robots(id).getState();
        animalStates(i,:) = [state, id];
    end

    robotStates = [];
    
    % Add flocking
    newAnimalStates = flockingBehaviour.step(animalStates, robotStates);
    
    % Update velocity
    for i=1:size(newAnimalStates,1)
        % Get id
        id = newAnimalStates(i,5);
        % Update velocity based on id
        robots(id) = robots(id).setVelocity(newAnimalStates(i,3:4));
        % Move robot
        robots(id).move();
    end

end
