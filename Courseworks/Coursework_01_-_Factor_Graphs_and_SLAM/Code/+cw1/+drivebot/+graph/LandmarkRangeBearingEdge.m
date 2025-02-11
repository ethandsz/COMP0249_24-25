classdef LandmarkRangeBearingEdge < g2o.core.BaseBinaryEdge
    methods(Access = public)
        function obj = LandmarkRangeBearingEdge()
            obj = obj@g2o.core.BaseBinaryEdge(2);
        end
        
        function initialEstimate(obj)
            platformState = obj.edgeVertices{1}.estimate(); % [x, y, theta]
            z = obj.z; % [range, bearing]
            
            % Compute landmark position using measurement
            lx = platformState(1) + z(1) * cos(platformState(3) + z(2)); % Corrected angle: theta + bearing
            ly = platformState(2) + z(1) * sin(platformState(3) + z(2));
            obj.edgeVertices{2}.setEstimate([lx; ly]);
        end
                
       function computeError(obj)
            platformState = obj.edgeVertices{1}.estimate();
            landmarkPos = obj.edgeVertices{2}.estimate();
            dx = landmarkPos(1) - platformState(1);
            dy = landmarkPos(2) - platformState(2);
            r = sqrt(dx^2 + dy^2);
            beta = atan2(dy, dx) - platformState(3);
            % Error = measured - predicted
            obj.errorZ = [obj.z(1) - r; 
                          g2o.stuff.normalize_theta(obj.z(2) - beta)];
        end
        
        function linearizeOplus(obj)
            platformState = obj.edgeVertices{1}.estimate();
            landmarkPos = obj.edgeVertices{2}.estimate();
            dx = landmarkPos(1) - platformState(1);
            dy = landmarkPos(2) - platformState(2);
            r = norm([dx dy]);
            
            % Jacobian w.r.t. platform (vertex 1)
            dR_dx = -dx / r;
            dR_dy = -dy / r;
            dR_dtheta = 0;
            dB_dx = dy / r^2;
            dB_dy = -dx / r^2;
            dB_dtheta = -1;
            obj.J{1} = -[dR_dx, dR_dy, dR_dtheta;
                         dB_dx, dB_dy, dB_dtheta];
            
            % Jacobian w.r.t. landmark (vertex 2)
            dR_dlx = dx / r;
            dR_dly = dy / r;
            dB_dlx = -dy / r^2;
            dB_dly = dx / r^2;
            obj.J{2} = -[dR_dlx, dR_dly;
                         dB_dlx, dB_dly];
        end
    end
end