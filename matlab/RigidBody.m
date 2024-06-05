%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Summary： MotionCaptureの計測データ格納用(剛体)
%Author  : Kazuyuki Kon
%Date   : 1/6/2013
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
classdef RigidBody
    %RigidBody Measurement of rigidbody
    %
    
    properties(SetAccess = public)
        id;  %ID of RigidBody
        x;   %x coordinate of RigidBody 
        y;   %y coordinate of RigidBody
        z;   %z coordinate of RigidBody
        qx;  %qx Quaternion of RigidBody
        qy;  %qy Quaternion of RigidBody
        qz;  %qz Quaternion of RigidBody
        qw;  %qw Quaternion of RigidBody
        
        roll;  %Euler angle
        pitch; %Euler angle
        yaw;   %Euler angle
        
        meanError;%Mean Error
        nMarker; %Number of member marker
        Markers; %Array of Marker
    end
    
    methods
        %概要：コンストラクタ
        %
        %
        function [obj] = RigidBody()
            obj.id = -1;
            obj.x  = 0;
            obj.y  = 0;
            obj.z  = 0;
            obj.qx  = 0;
            obj.qy  = 0;
            obj.qz  = 0;
            obj.qw  = 0;
            obj.roll = 0;
            obj.pitch = 0;
            obj.yaw = 0;           
            
            obj.nMarker = 0;
            obj.meanError = 0;
            obj.Markers = cell(1,1);
        end
    end
    
end

