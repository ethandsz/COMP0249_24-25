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
    %   M = [cos(theta) -sin(theta) 0; sin(theta) cos(theta) 0;0 0 1];
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
            % INITIALESTIMATE Compute the initial estimate of a platform.
            %
            % Syntax:
            %   obj.initialEstimate();
            %
            % Description:
            %   Compute the initial estimate of the platform x_(k+1) given
            %   an estimate of the platform at time x_(k) and the control
            %   input u_(k+1)

           % warning('PlatformPredictionEdge.initialEstimate: implement')
           %   M = [cos(theta) -sin(theta) 0; sin(theta) cos(theta) 0;0 0 1];
            %
            % The new state is predicted from 
            %
            %   x_(k+1) = x_(k) + M * [vx;vy;theta]

            x_k = obj.edgeVertices{1}.estimate();
            theta = x_k(3);
            M = [cos(theta) -sin(theta) 0; 
                sin(theta) cos(theta) 0;
                0 0 1];
            x_k1 = x_k + (obj.dT* M) * (obj.z);
            %x_k1 = x_k + ((M*obj.dT)) * obj.z;
            

            x_k1(3) = x_k(3) + obj.z(3) * obj.dT;
            x_k1(3) = g2o.stuff.normalize_theta(x_k1(3));
            obj.edgeVertices{2}.setEstimate(x_k1);
            %fprintf("Next State Estimate:\nx: %.2f\ny: %.2f\ntheta: %.2f\n\n\n", x_k1(1), x_k1(2), x_k1(3));
        end
        

        
        % function computeError(obj)
        %     % COMPUTEERROR Compute the error for the edge.
        %     %
        %     % Syntax:
        %     %   obj.computeError();
        %     %
        %     % Description:
        %     %   Compute the value of the error, which is the difference
        %     %   between the measurement and the parameter state in the
        %     %   vertex. Note the error enters in a nonlinear manner, so the
        %     %   equation has to be rearranged to make the error the subject
        %     %   of the formulat
        % 
        %     % e(x,z) = inv(M) * (x_(k+1) - x_(k))
        %     x_k = obj.edgeVertices{1}.estimate();
        %     x_k1 = obj.edgeVertices{2}.estimate();
        %     theta = x_k(3);
        %     M = [cos(theta) -sin(theta) 0; 
        %         sin(theta) cos(theta) 0;
        %         0 0 1];
        %     Mi = inv(M);
        %     dx = x_k1 - x_k;
        % 
        % 
        %     obj.errorZ = Mi\dx - (obj.z * obj.dT);
        %     obj.errorZ(3) = g2o.stuff.normalize_theta(obj.errorZ(3));
        % end
        % 
        % % Compute the Jacobians
        % function linearizeOplus(obj)
        %     % LINEARIZEOPLUS Compute the Jacobians for the edge.
        %     %
        %     % Syntax:
        %     %   obj.computeError();
        %     %
        %     % Description:
        %     %   Compute the Jacobians for the edge. Since we have two
        %     %   vertices which contribute to the edge, the Jacobians with
        %     %   respect to both of them must be computed.
        %     %
        %     x_k = obj.edgeVertices{1}.estimate();
        %     x_k1 = obj.edgeVertices{2}.estimate();
        %     dx = x_k1 - x_k;
        %     theta = x_k(3);
        %     c = cos(theta);
        %     s = sin(theta);
        % 
        %     M = [cos(theta) -sin(theta) 0; 
        %         sin(theta) cos(theta) 0;
        %         0 0 1];
        %     Mi = inv(M);
        %     obj.J{2} = Mi;
        %     %warning('PlatformPredictionEdge.linearizeOplus: implement')
        %     obj.J{1}(1, 1) = - c;
        %     obj.J{1}(1, 2) = - s;
        %     obj.J{1}(1, 3) = -dx(1) * s + dx(2) * c;
        %     obj.J{1}(2, 1) = s;
        %     obj.J{1}(2, 2) = - c;
        %     obj.J{1}(2, 3) = -dx(1) * c - dx(2) * s;
        %     obj.J{1}(3, 3) = -1;
        %    % obj.J{1} = -eye(3);
    % end
        function computeError(obj)
            priorX = obj.edgeVertices{1}.estimate(); %Check this state! Why is it so different than edgeVertices{2}???
            currentX = obj.edgeVertices{2}.estimate();
            
            c = cos(priorX(3));
            s = sin(priorX(3));
            Mi = [c s 0; -s c 0; 0 0 1];
            dx = (currentX - priorX);
            % Error = measurement (0) - predicted motion
            obj.errorZ = Mi * dx - (obj.z * obj.dT); % z is velocity scaled by dT
            obj.errorZ(3) = g2o.stuff.normalize_theta(obj.errorZ(3));
            obj.errorZ = obj.errorZ/2;
            obj.setMeasurement(obj.z);

          % obj.setMeasurement([obj.z(1); obj.z(2); g2o.stuff.normalize_theta(obj.z(3)) ]);

        end
        
        function linearizeOplus(obj)
            priorX = obj.edgeVertices{1}.estimate();
            currentX = obj.edgeVertices{2}.estimate();
            dx = obj.dT * (currentX - priorX);
            c = cos(priorX(3)) ;
            s = sin(priorX(3));
            Mi = [c s 0; -s c 0; 0 0 1];
            
            % Jacobian for x_{k+1}
            obj.J{2} = Mi;
            
            % Jacobian for x_k
            obj.J{1} = [-c, -s, -dx(1)*s + dx(2)*c;
                         s, -c, -dx(1)*c - dx(2)*s;
                         0,  0,               -1];
            % fprintf("J1:\n");
            % fprintf("%.2f %.2f %.2f\n", obj.J{1}');
            % fprintf("J2:\n");
            % 
            % fprintf("%.2f %.2f %.2f\n", obj.J{2}');

        end

        
    end    
end