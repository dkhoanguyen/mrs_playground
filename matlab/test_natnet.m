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

% Ctrl + C �ł��I���֐����Ă΂��悤�ɂ���
C = onCleanup(@()nat.terminate());

% �o�[�W�������m�F����
[nat,version] = nat.getVersion();

% �ڑ������m�F����
[nat,serverIP,serverName,serverAppVer] = nat.getServerDescription();

% �ڑ�����\������
disp('-----------------------------------------');
version
serverIP
serverName
serverAppVer
disp('-----------------------------------------');

%%
[nat, isNew, frameData] = nat.update();