classdef Robot
    %SHEEP Summary of this class goes here
    %   Detailed explanation goes here

    properties
        comms;
    end

    methods (Access=public)
        function obj = Robot(id)
            id_str = 'OMNI-ROBOT-' + string(id);
            obj.comms = bluetooth(id_str,1);
        end
        function init_done = initialise(obj)
            init_done = false;
            fopen(obj.comms);
        end
        function moveRaw(obj,vxs, vys, ws, yaw)
            wl = ws;
            vxl = vxs * cos(yaw) + vys * sin(yaw);
            vyl = -vxs * sin(yaw) + vys * cos(yaw);

            vxl =  round( 1000*vxl*0.5 ) ;       %速度指令m/s を mm/s変換(半分に圧縮)
            vyl =  round( 1000*vyl*0.5 );        %速度指令m/s を mm/s変換
            wl =  round( 180/pi*wl*0.5 );        %速度指令rad/s を 度/s変換
            
            %% Begin transmission
            fwrite(obj.comms, 255); 

            %% Send velocity command vxl
            if vxl < 0
                fwrite(obj.comms, 251);
            else
                fwrite(obj.comms, 252);
            end
            fwrite(obj.comms, abs(vxl));

            %% Send velocity command vyl
            if vyl < 0
                fwrite(obj.comms, 251);
            else
                fwrite(obj.comms, 252);
            end
            fwrite(obj.comms, abs(vyl));

            %% Send yaw command
            if wl < 0
                fwrite(obj.comms, 251);
            else
                fwrite(obj.comms, 252);
            end
            fwrite(obj.comms, abs(wl));
        end
    end
end

