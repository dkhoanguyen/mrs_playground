%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Summary�F MotionCapture�̌v���f�[�^�i�[�p�i�P�p�P�b�g�j
%Author  : Kazuyuki Kon
%Date   : 1/6/2013
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
classdef Frame
    %Frame ���̃N���X�̊T�v�������ɋL�q
    %   �ڍא����������ɋL�q
    
    properties(SetAccess = public)
        iCount;       %
        Latency;      %
        nMarkerSets;  %
        nOtherMarkers;%
        nRigidBodies; %
        nSkeltons;    %
        
        RigidBodies  = cell(1,1); %RigidBody
        Skeltons     = cell(1,1); %Skelton
        OtherMarkers = cell(1,1); %Marker
        MarkerSets   = cell(1,1); %MarkerSet
    end
    
    methods
        %�T�v�F�R���X�g���N�^
        %
        function [obj] = Frame()
            obj.iCount      = -1;
            obj.Latency     = 0;
            obj.nMarkerSets = 0;
            obj.nOtherMarkers = 0;
            obj.nRigidBodies  = 0;
            obj.nSkeltons    = 0;
                    
            obj.RigidBodies  = cell(1,1); %RigidBody
            obj.Skeltons     = cell(1,1); %Skelton
            obj.OtherMarkers = cell(1,1); %Marker
            obj.MarkerSets   = cell(1,1); %MarkerSet
        end        
    end
    
end

