clear all;
close all;
clc;

%% Initialise robots

numRobots = 3;
robots = [];
dt = 0.1;
nT = 100;

for i=3:numRobots
    robot = Robot(i);
    robot.initialise();
    robots = [robots, robot];
end

%%

for t = 1 : nT
    for idx = 1:numel(robots)
        vxs = 0.1;
        vys = 0;
        ws  = 0;
        yaw = 0;
        robots(idx).moveRaw(vxs,vys,ws,yaw);
    end
end