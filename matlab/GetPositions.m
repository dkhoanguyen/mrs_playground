% main.m ����Ăяo�����X�N���v�g:���[�V�����L���v�`�����烍�{�b�g�̈ʒu�f�[�^���擾���Cx�Cyaw�Ɋi�[����
% ���f�[�^��id�ƃ��{�b�g��ID�����킹�Ă��� �� positions��ID���ɂȂ��Ă���

% �f�[�^�擾
isNew = 0;
temp_position = zeros(2, nV);
temp_yaw = zeros(nV, 1);
[nat, isNew, frameData] = nat.update();

if isNew
    for i = 1:nV
        temp_position(1, frameData.RigidBodies{i,1}.id) = frameData.RigidBodies{i,1}.x;
%         temp_position(2, frameData.RigidBodies{i,1}.id) = frameData.RigidBodies{i,1}.y;   % Z-up�̏ꍇ
%         temp_yaw(frameData.RigidBodies{i,1}.id) = frameData.RigidBodies{i,1}.pitch;       % Z-up�̏ꍇ 
        temp_position(2, frameData.RigidBodies{i,1}.id) = - frameData.RigidBodies{i,1}.z;   %Y-up�̏ꍇ
        temp_yaw(frameData.RigidBodies{i,1}.id) = frameData.RigidBodies{i,1}.yaw;         % Y-up�̏ꍇ 
    end
    for i = 1:nV
        x(:, i) = temp_position(:, i);
        yaw(i,1) = temp_yaw(i,1);

    end
end