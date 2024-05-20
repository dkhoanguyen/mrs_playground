%% ロボットへ速度指令を送るプログラム
%%
% ■概要: 以下のＵＲＬ参照
%    https://jp.mathworks.com/matlabcentral/answers/277255-connection-to-arduino-using-bluetooth
% ■著者: 三宅正人
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%■概要:速度指令@絶対座標系
%vxs : 絶対座標系でのx軸方向の入力(m/s)
%vys : 絶対座標系でのy軸方向の入力(m/s)
%ws  : 回転角速度入力[rad/s]
%yaws :　ロボットの姿勢列[rad]

%■概要:速度指令@ロボット座標系
%vxl : ロボット座標系でのx軸方向の速度入力列[m/s]
%vyl : ロボット座標系でのy軸方向の速度入力列[m/s]
%wl  : 回転角速度列[rad/s]

function [robot] = move_robot(ROBOT_ID, nV,  vxs, vys, ws, yaw)

vxl = zeros(nV,1);
vyl = zeros(nV,1);
wl = ws;

for i = 1:vxl
    vxl(i,1) = vxs(i,1)*cos(yaw(i,1)) + vys(i,1)*sin(yaw(i,1));   %速度指令@絶対座標系を速度指令@ロボット座標系に変換
    vyl(i,1) = -vxs(i,1)*sin(yaw(i,1)) + vys(i,1)*cos(yaw(i,1));

    vxl(i) =  round( 1000*vxl(i)*0.5 ) ;       %速度指令m/s を mm/s変換(半分に圧縮)
    vyl(i) =  round( 1000*vyl(i)*0.5 );        %速度指令m/s を mm/s変換
    wl(i) =  round( 180/pi*wl(i)*0.5 );        %速度指令rad/s を 度/s変換

    fwrite(ROBOT_ID(i), 255); %信号の先頭

    if (vxl(i) < 0);
        fwrite(ROBOT_ID(i), 251);  %負の値の場合
    else
        fwrite(ROBOT_ID(i), 252);  %正の値の場合
    end
    fwrite(ROBOT_ID(i), abs(vxl(i)));    %速度指令を送信
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
