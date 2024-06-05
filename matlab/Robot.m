classdef Robot
    % Class ROBOT reserved for handling bluetooth comms between MATLAB and
    % hardware robot

    properties (Access=private)
        m_ID;
        m_Comms;
        m_State;
    end

    methods (Access=public)
        function obj = Robot(id)
            obj.m_ID = id;
            obj.m_State = zeros(1,4);
        end
        function initDone = initialise(obj)
            %% Initialise communication
            idStr = 'OMNI-ROBOT-' + string(obj.m_ID);
            obj.m_Comms = bluetooth(idStr,1);
            fopen(obj.m_Comms);
            initDone = true;
        end

        %% Setters Getters
        function obj = setPosition(obj,position)
            obj.m_State(1:2) = position;
        end

        function setVelocity(obj, velocity)
            obj.m_State(3:4) = velocity;
        end

        function state = getState(obj)
            state = obj.m_State;
        end
        
        %% Control
        function move(obj)
            vx = obj.m_State(3);
            vy = obj.m_State(4);
            obj.moveRaw(vx, vy, 0, 0);
        end

        function moveRaw(obj,vxs, vys, ws, yaw)
            wl = ws;
            vxl = vxs * cos(yaw) + vys * sin(yaw);
            vyl = -vxs * sin(yaw) + vys * cos(yaw);
            vxl =  round( 1000*vxl*0.5 ) ;       
            vyl =  round( 1000*vyl*0.5 );        
            wl =  round( 180/pi*wl*0.5 );        
            
            %% Begin transmission
            fwrite(obj.m_Comms, 255); 

            %% Send velocity command vxl
            if vxl < 0
                fwrite(obj.m_Comms, 251);
            else
                fwrite(obj.m_Comms, 252);
            end
            fwrite(obj.m_Comms, abs(vxl));

            %% Send velocity command vyl
            if vyl < 0
                fwrite(obj.m_Comms, 251);
            else
                fwrite(obj.m_Comms, 252);
            end
            fwrite(obj.m_Comms, abs(vyl));

            %% Send yaw command
            if wl < 0
                fwrite(obj.m_Comms, 251);
            else
                fwrite(obj.m_Comms, 252);
            end
            fwrite(obj.m_Comms, abs(wl));
        end
    end
end

