classdef PlatformPredictionEdge < g2o.core.BaseBinaryEdge
    % PlatformPredictionEdge summary of PlatformPredictionEdge
    %
    % This class stores the factor representing the process model which
    % transforms the state from timestep k to k+1
    %
    % The process model is as follows.
    %
    % Define the rotation vector
    %
    %   M = dT * [cos(theta) -sin(theta) 0; sin(theta) cos(theta) 0;0 0 1];
    %
    % The new state is predicted from 
    %
    %   x_(k+1) = x_(k) + M * [vx;vy;theta]
    %
    % Note in this case the measurement is actually the mean of the process
    % noise. It has a value of 0. The error vector is given by
    %
    % e(x,z) = inv(M) * (x_(k+1) - x_(k))
    %
    % Note this requires estimates from two vertices - x_(k) and x_(k+1).
    % Therefore, this inherits from a binary edge. We use the convention
    % that vertex slot 1 contains x_(k) and slot 2 contains x_(k+1).
    
    properties(Access = protected)
        % The length of the time step
        dT;
    end
    
    methods(Access = public)
        function obj = PlatformPredictionEdge(dT)
            % PlatformPredictionEdge for PlatformPredictionEdge
            %
            % Syntax:
            %   obj = PlatformPredictionEdge(dT);
            %
            % Description:
            %   Creates an instance of the PlatformPredictionEdge object.
            %   This predicts the state from one timestep to the next. The
            %   length of the prediction interval is dT.
            %
            % Outputs:
            %   obj - (handle)
            %       An instance of a PlatformPredictionEdge

            assert(dT >= 0);
            obj = obj@g2o.core.BaseBinaryEdge(3);            
            obj.dT = dT;
        end
       
        function initialEstimate(obj)

            xk = obj.edgeVertices{1}.x;       % Get x_k from Vertex 1
        
            theta_k = xk(3);                  % Extract orientation θ_k
        
            % Construct rotation matrix M (Equation 4 in Appendix A)
        
            M = obj.dT * [cos(theta_k), -sin(theta_k), 0;
        
                         sin(theta_k),  cos(theta_k), 0;
        
                         0, 0, 1];
        
            % Compute x_{k+1} = x_k + M * z (z is control input u)
        
            xkp1 = xk + M * obj.z;            % Predict x_{k+1}
        
            % Wrap θ_{k+1} to [-pi, pi]
        
            xkp1(3) = g2o.stuff.normalize_theta(xkp1(3));
        
            % Set Vertex 2's state
        
            obj.edgeVertices{2}.x = xkp1;
        
        end
         
        
        function computeError(obj)
            xk = obj.edgeVertices{1}.x;       % Current state x_k
            xkp1 = obj.edgeVertices{2}.x;     % Next state x_{k+1}
            theta_k = xk(3);                  % Extract θ_k
            % Construct rotation matrix M
            M = obj.dT * [cos(theta_k), -sin(theta_k), 0;
                         sin(theta_k),  cos(theta_k), 0;
                         0, 0, 1];
            invM = inv(M);                    % Compute M^{-1}
            delta_x = xkp1 - xk;              % Δx = x_{k+1} - x_k
            % Wrap the orientation difference
            delta_x(3) = g2o.stuff.normalize_theta(delta_x(3));
            % Error = M^{-1} * Δx - z (z is control input u)
            obj.errorZ = invM * delta_x - obj.z;
        end
        
                % Compute the Jacobians
         function linearizeOplus(obj)
            % Get the current state
            xk = obj.edgeVertices{1}.x;
            xk1 = obj.edgeVertices{2}.x;
            theta = xk(3);
            % Compute the rotation matrix M
            M = obj.dT * [cos(theta) -sin(theta) 0;
                          sin(theta) cos(theta) 0;
                          0 0 1];
            % The Jacobian with respect to xk
            dtheta = xk1 - xk;
            J1 = -(M' / obj.dT);
            % The Jacobian with respect to xk1
            J2 = M' / obj.dT;
            % Store the Jacobians
            obj.J{1} = J1;
            obj.J{2} = J2;
        end

    end    
end