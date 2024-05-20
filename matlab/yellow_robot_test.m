nV          = 1;
ROBOT_ID(1) = bluetooth('OMNI-ROBOT-1',1);
dt          = 0.1;
nT          = 100;

for i = 1: nV
    fopen(ROBOT_ID(i));
end

for t = 1 : nT
    for i = 1: nV
        vxs(i) = 0.1;
        vys(i) = 0;
        ws(i)  = 0;
        yaw(i) = 0;
        robot  = move_robot(ROBOT_ID, nV, vxs, vys, ws, yaw);
    end
    pause(dt);
end

for i = 1: nV
    fclose(ROBOT_ID(i));
end