clear all;
close all;
clc;

%%
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

% Ctrl + C でも終了関数が呼ばれるようにする
C = onCleanup(@()nat.terminate());

% バージョンを確認する
[nat,version] = nat.getVersion();

% 接続情報を確認する
[nat,serverIP,serverName,serverAppVer] = nat.getServerDescription();

% 接続情報を表示する
disp('-----------------------------------------');
version
serverIP
serverName
serverAppVer
disp('-----------------------------------------');

%%
[nat, isNew, frameData] = nat.update();