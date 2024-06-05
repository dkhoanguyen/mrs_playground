% main.m から呼び出されるスクリプト:モーションキャプチャからロボットの位置データを取得し，x，yawに格納する
% 生データのidとロボットのIDを合わせている → positionsがID順になっている

% データ取得
isNew = 0;
temp_position = zeros(2, nV);
temp_yaw = zeros(nV, 1);
[nat, isNew, frameData] = nat.update();

if isNew
    for i = 1:nV
        temp_position(1, frameData.RigidBodies{i,1}.id) = frameData.RigidBodies{i,1}.x;
%         temp_position(2, frameData.RigidBodies{i,1}.id) = frameData.RigidBodies{i,1}.y;   % Z-upの場合
%         temp_yaw(frameData.RigidBodies{i,1}.id) = frameData.RigidBodies{i,1}.pitch;       % Z-upの場合 
        temp_position(2, frameData.RigidBodies{i,1}.id) = - frameData.RigidBodies{i,1}.z;   %Y-upの場合
        temp_yaw(frameData.RigidBodies{i,1}.id) = frameData.RigidBodies{i,1}.yaw;         % Y-upの場合 
    end
    for i = 1:nV
        x(:, i) = temp_position(:, i);
        yaw(i,1) = temp_yaw(i,1);

    end
end