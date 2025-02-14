classdef PlatformPredictionEdge < g2o.core.BaseBinaryEdge
    properties(Access = protected)
        dT; % Time step duration
    end
    
    methods(Access = public)
        function obj = PlatformPredictionEdge(dT)
            assert(dT >= 0);
            obj = obj@g2o.core.BaseBinaryEdge(3);            
            obj.dT = dT;
        end
       
        function initialEstimate(obj)
            xk = obj.edgeVertices{1}.x;       % Get x_k from Vertex 1
            theta_k = xk(3);                  % Extract orientation θ_k
        
            % Construct rotation matrix M (scaled by dT)
            M = obj.dT * [cos(theta_k), -sin(theta_k), 0;
                         sin(theta_k),  cos(theta_k), 0;
                         0, 0, 1];
        
            % Compute x_{k+1} = x_k + M * z (z is control input u)
            xkp1 = xk + M * obj.z; % Predict x_{k+1}
        
            % Wrap θ_{k+1} to [-pi, pi]
            xkp1(3) = g2o.stuff.normalize_theta(xkp1(3));
        
            % Set Vertex 2's state
            obj.edgeVertices{2}.x = xkp1;
        end
         
        function computeError(obj)
            xk = obj.edgeVertices{1}.x;       % Current state x_k
            xkp1 = obj.edgeVertices{2}.x;     % Next state x_{k+1}
            theta_k = xk(3);                  % Extract θ_k
        
            % Construct rotation matrix M (scaled by dT)
            M = obj.dT * [cos(theta_k), -sin(theta_k), 0;
                         sin(theta_k),  cos(theta_k), 0;
                         0, 0, 1];
        
            % Error = inv(M) * (x_{k+1} - x_k) - z
            error = inv(M) * (xkp1 - xk) - obj.z;
            obj.errorZ = error;
            obj.errorZ(3) = g2o.stuff.normalize_theta(obj.errorZ(3));
        end
        
        function linearizeOplus(obj)
            priorX = obj.edgeVertices{1}.x;
            c = cos(priorX(3));
            s = sin(priorX(3));
            dx = obj.edgeVertices{2}.x - priorX;
        
            % Jacobian for x_{k+1} (scaled by dT)
            obj.J{2} = [c, s, 0;
                       -s, c, 0;
                        0, 0, 1] / obj.dT;
        
            % Jacobian for x_k (scaled by dT)
            obj.J{1} = [-c, -s, -dx(1)*s + dx(2)*c;
                         s, -c, -dx(1)*c - dx(2)*s;
                         0,  0,               -1] / obj.dT;
        end
    end    
end