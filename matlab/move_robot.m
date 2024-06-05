%% ���{�b�g�֑��x�w�߂𑗂�v���O����
%%
% ���T�v: �ȉ��̂t�q�k�Q��
%    https://jp.mathworks.com/matlabcentral/answers/277255-connection-to-arduino-using-bluetooth
% ������: �O��l
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%���T�v:���x�w��@��΍��W�n
%vxs : ��΍��W�n�ł�x�������̓���(m/s)
%vys : ��΍��W�n�ł�y�������̓���(m/s)
%ws  : ��]�p���x����[rad/s]
%yaws :�@���{�b�g�̎p����[rad]

%���T�v:���x�w��@���{�b�g���W�n
%vxl : ���{�b�g���W�n�ł�x�������̑��x���͗�[m/s]
%vyl : ���{�b�g���W�n�ł�y�������̑��x���͗�[m/s]
%wl  : ��]�p���x��[rad/s]

function [robot] = move_robot(ROBOT_ID, nV,  vxs, vys, ws, yaw)

vxl = zeros(nV,1);
vyl = zeros(nV,1);
wl = ws;

for i = 1:vxl
    vxl(i,1) = vxs(i,1)*cos(yaw(i,1)) + vys(i,1)*sin(yaw(i,1));   %���x�w��@��΍��W�n�𑬓x�w��@���{�b�g���W�n�ɕϊ�
    vyl(i,1) = -vxs(i,1)*sin(yaw(i,1)) + vys(i,1)*cos(yaw(i,1));

    vxl(i) =  round( 1000*vxl(i)*0.5 ) ;       %���x�w��m/s �� mm/s�ϊ�(�����Ɉ��k)
    vyl(i) =  round( 1000*vyl(i)*0.5 );        %���x�w��m/s �� mm/s�ϊ�
    wl(i) =  round( 180/pi*wl(i)*0.5 );        %���x�w��rad/s �� �x/s�ϊ�

    fwrite(ROBOT_ID(i), 255); %�M���̐擪

    if (vxl(i) < 0);
        fwrite(ROBOT_ID(i), 251);  %���̒l�̏ꍇ
    else
        fwrite(ROBOT_ID(i), 252);  %���̒l�̏ꍇ
    end
    fwrite(ROBOT_ID(i), abs(vxl(i)));    %���x�w�߂𑗐M
    %  out_1 = fscanf(ROBOT_ID(i),'%d');

    if (vyl(i) < 0);
        fwrite(ROBOT_ID(i), 251);
    else
        fwrite(ROBOT_ID(i), 252);
    end
    fwrite(ROBOT_ID(i), abs(vyl(i)));
    %  out_2 = fscanf(ROBOT_ID(i),'%d');

    if (wl(i) < 0);
        fwrite(ROBOT_ID(i), 251);
    else
        fwrite(ROBOT_ID(i), 252);
    end
    fwrite(ROBOT_ID(i), abs(wl(i)));
    %  out_3 = fscanf(ROBOT_ID(i),'%d');

    %       message_1 = sprintf('%d : %d',i,vxl(i));
    %       message_2 = sprintf('%d : %d',i,vyl(i));
    %       message_3 = sprintf('%d : %d',i,wl(i));
    %       disp(message_1)
    %       disp(message_2)
    %       disp(message_3)
end

robot =1;
end
