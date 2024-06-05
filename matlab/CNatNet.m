%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Summary: Class for NatNet on Matlab
%Author : Kazuyuki Kon@Kyoto-u
%Date   : 1/6/2013
%Requirement   : NatNet 2.2.0.0(or later), CNatNet.dll
%Usage: See testCNatNet.m
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
classdef CNatNet
    %CNATNET NatNet wrapper class(use CNatNet.dll)
    
    properties
        m_CNatNet;
        m_ClientIP = '192.168.1.2';
        m_ServerIP = '192.168.1.1';
        m_ConnectionType = 0; %0:Multicast, 1:Unicast
        m_dllPath = fullfile('c:','test','CNatNetML.dll');
        m_CNatNetInfo;
        m_cnt;
    end
    
    methods
        %概要：コンストラクタ
        function [obj] = CNatNet()
            obj.m_cnt = 0;
        end
        
        %概要：サーバー(ホスト)からの接続を解除する
        %[引数]
        % connectionType : 0:Multicast, 1:Unicast
        % dllPath : CNatNet.dll, NatNetML.dllのパス
        %[戻り値]
        % isInitialized : 初期化ができたかどうか(true:成功， false:失敗)
        function [obj,isInitialized] = initialize(obj, connectionType, dllPath)
            %Load assembly
            obj.m_dllPath = dllPath;
            obj.m_CNatNetInfo = NET.addAssembly( obj.m_dllPath );
            
            %Create instance
            obj.m_CNatNet = CNatNetML.CNatNetML();
            
            %Initalize NatNet
            obj.m_ConnectionType = connectionType;
            returnCode = obj.m_CNatNet.initialize( obj.m_ConnectionType )
            if returnCode == true
                disp('Initialize succeed');
                isInitialized = true;
            else
                disp('Initialize error') ;
                isInitialized = false;
            end
        end
        
        %概要：終了処理を行う．
        %[引数]
        %[戻り値]
        function [obj] = terminate(obj)
            obj.m_CNatNet.terminate();
        end
        
        %概要：サーバー(ホスト)に接続する
        %[引数]
        % clientIP : クライアント(受信側)のIPアドレス
        % serverIP : サーバー(ホスト)のIPアドレス
        %[戻り値]
        % isConnected : 接続の可否(true:成功，false:失敗)
        function [obj, isConnected] = connect(obj,clientIP, serverIP)
            obj.m_ClientIP = clientIP;
            obj.m_ServerIP = serverIP;
            m_cnt = 0;
            returnCode = obj.m_CNatNet.connect(obj.m_ClientIP ,obj.m_ServerIP );
            
            if returnCode == true
                disp('Connection succeed');
                isConnected = true;
            else
                disp('Connection error') ;
                isConnected = false;
            end
        end
        
        %概要：サーバー(ホスト)からの接続を解除する
        %[引数]
        %[戻り値]
        function [obj] = discconect(obj)
            obj.m_CNatNet.disconnect();
        end
        
        %概要：計測データを取得する
        %[引数]
        %[戻り値]
        %frameData : Latest measurement data from NatNet
        function [obj,isNew, frameData, frameDataNet] = update(obj)        
        %function [obj,isNew,frameData,frameDataNet] = update(obj)
            %最新のフレームデータを取得する         
            frameDataNet = obj.m_CNatNet.getFrameData();
            frameDataNet.RigidBodies(1)
            
            %if frameDataNet.iCount > obj.m_cnt
                obj.m_cnt = frameDataNet.iCount;
                isNew = true;
                
                %Copy data to matlab class
                frameData = Frame();
                frameData.iCount        = frameDataNet.iCount;
                frameData.Latency       = frameDataNet.Latency;
                frameData.nMarkerSets   = frameDataNet.nMarkerSet;
                frameData.nOtherMarkers = frameDataNet.nOtherMarker;
                frameData.nRigidBodies  = frameDataNet.nRigid;
                frameData.nSkeltons     = frameDataNet.nSkelton;
                
                %Copy RigidBodies
                for i = 1:frameData.nRigidBodies
                    frameData.RigidBodies{i,1} = RigidBody();
                    frameData.RigidBodies{i,1}.id = frameDataNet.RigidBodies.Get(i-1).id;
                    frameData.RigidBodies{i,1}.x  = frameDataNet.RigidBodies.Get(i-1).x;
                    frameData.RigidBodies{i,1}.y  = frameDataNet.RigidBodies.Get(i-1).y;
                    frameData.RigidBodies{i,1}.z  = frameDataNet.RigidBodies.Get(i-1).z;
%                     frameData.RigidBodies{i,1}.qx = frameDataNet.RigidBodies.Get(i-1).qx;
%                     frameData.RigidBodies{i,1}.qy = frameDataNet.RigidBodies.Get(i-1).qy;
%                     frameData.RigidBodies{i,1}.qz = frameDataNet.RigidBodies.Get(i-1).qz;
%                     frameData.RigidBodies{i,1}.qw = frameDataNet.RigidBodies.Get(i-1).qw;
                    
%                   frameData.RigidBodies{i,1}.roll = frameDataNet.RigidBodies.Get(i-1).roll;
                    frameData.RigidBodies{i,1}.pitch = frameDataNet.RigidBodies.Get(i-1).pitch;
                    frameData.RigidBodies{i,1}.yaw = frameDataNet.RigidBodies.Get(i-1).yaw;
%                     frameData.RigidBodies{i,1}.nMarker = frameDataNet.RigidBodies.Get(i-1).nMarker;
                    frameData.RigidBodies{i,1}.meanError = frameDataNet.RigidBodies.Get(i-1).meanError;
                    
%                     for j = 1:frameData.RigidBodies{i,1}.nMarker
%                         frameData.RigidBodies{i,1}.Markers{j,1}      = Marker();
%                         frameData.RigidBodies{i,1}.Markers{j,1}.id   = frameDataNet.RigidBodies.Get(i-1).Markers.Get(j-1).id;
%                         frameData.RigidBodies{i,1}.Markers{j,1}.x    = frameDataNet.RigidBodies.Get(i-1).Markers.Get(j-1).x;
%                         frameData.RigidBodies{i,1}.Markers{j,1}.y    = frameDataNet.RigidBodies.Get(i-1).Markers.Get(j-1).y;
%                         frameData.RigidBodies{i,1}.Markers{j,1}.z    = frameDataNet.RigidBodies.Get(i-1).Markers.Get(j-1).z;
%                         frameData.RigidBodies{i,1}.Markers{j,1}.size = frameDataNet.RigidBodies.Get(i-1).Markers.Get(j-1).size;
%                     end
                end
                
                %Copy Skeltons
                %Not implemented yet!!
                
                %Copy OtherMarkers
                for i = 1:frameData.nOtherMarkers
                    frameData.OtherMarkers{i,1} = Marker();
                    frameData.OtherMarkers{i,1}.id = frameDataNet.OtherMarkers.Get(i-1).id;
                    frameData.OtherMarkers{i,1}.x = frameDataNet.OtherMarkers.Get(i-1).x;
                    frameData.OtherMarkers{i,1}.y = frameDataNet.OtherMarkers.Get(i-1).y;
                    frameData.OtherMarkers{i,1}.z = frameDataNet.OtherMarkers.Get(i-1).z;
                    frameData.OtherMarkers{i,1}.size = frameDataNet.OtherMarkers.Get(i-1).size;
                end
                
                %Copy MarkerSets
                for i= 1:frameData.nMarkerSets
                    frameData.MarkerSets{i,1} = MarkerSet();
                    frameData.MarkerSets{i,1}.nMarker       = frameDataNet.MarkerSets.Get(i-1).nMarker;
                    frameData.MarkerSets{i,1}.MarkerSetName = char(frameDataNet.MarkerSets.Get(i-1).MarkerSetName);
                    
                    for j= 1:frameData.MarkerSets{i,1}.nMarker
                        frameData.MarkerSets{i,1}.Markers{j,1}.id = frameDataNet.MarkerSets.Get(i-1).Markers.Get(j-1).id;
                        frameData.MarkerSets{i,1}.Markers{j,1}.x = frameDataNet.MarkerSets.Get(i-1).Markers.Get(j-1).x;
                        frameData.MarkerSets{i,1}.Markers{j,1}.y = frameDataNet.MarkerSets.Get(i-1).Markers.Get(j-1).y;
                        frameData.MarkerSets{i,1}.Markers{j,1}.z = frameDataNet.MarkerSets.Get(i-1).Markers.Get(j-1).z;
                        frameData.MarkerSets{i,1}.Markers{j,1}.size = frameDataNet.MarkerSets.Get(i-1).Markers.Get(j-1).size;
                    end
                end
            %else
            %    isNew = false;
            %    frameData = Frame();
            %end
            
        end
        
        %概要：Clientで使用しているNatNetのバージョンを取得する
        %[引数]
        %[戻り値]
        % version : バージョン番号
        function [obj,version] = getVersion(obj)
            version = char( obj.m_CNatNet.getVersion() );
        end
        
        %概要：接続したサーバーの情報を取得する
        %[引数]
        %[戻り値]
        % serverIP     : 接続したサーバーのIPアドレス
        % serverName   : 接続したサーバーの名前
        % serverAppVer : 接続したサーバーのアプリケーションのバージョン
        function [obj,serverIP,serverName,serverAppVer] = getServerDescription(obj)
            %Server IP
            serverIP = obj.m_CNatNet.getServerIP();
            serverIP = sprintf('%d.%d.%d.%d',serverIP(1),serverIP(2),serverIP(3),serverIP(4))
            
            %Server name
            serverName = obj.m_CNatNet.getServerName();
            serverName = char( serverName );
            
            %Version of server app
            serverAppVer = char(obj.m_CNatNet.getServerAppVersion());
        end
    end
    
end

