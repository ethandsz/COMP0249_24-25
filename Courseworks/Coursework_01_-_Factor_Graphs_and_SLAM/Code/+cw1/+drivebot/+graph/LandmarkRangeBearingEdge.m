% % classdef LandmarkRangeBearingEdge < g2o.core.BaseBinaryEdge
% %     methods(Access = public)
% %         function obj = LandmarkRangeBearingEdge()
% %             obj = obj@g2o.core.BaseBinaryEdge(2);
% %         end
% % 
% %         function initialEstimate(obj)
% %             platformState = obj.edgeVertices{1}.estimate(); % [x, y, theta]
% %             z = obj.z; % [range, bearing]
% % 
% %             % Compute landmark position using measurement
% %             lx = platformState(1) + z(1) * cos(platformState(3) + z(2)); % Corrected angle: theta + bearing
% %             ly = platformState(2) + z(1) * sin(platformState(3) + z(2));
% %             obj.edgeVertices{2}.setEstimate([lx; ly]);
% %         end
% % 
% %        function computeError(obj)
% %             platformState = obj.edgeVertices{1}.estimate();
% %             landmarkPos = obj.edgeVertices{2}.estimate();
% %             dx = landmarkPos(1) - platformState(1);
% %             dy = landmarkPos(2) - platformState(2);
% %             r = sqrt(dx^2 + dy^2);
% %             beta = atan2(dy, dx) - platformState(3);
% %             % Error = measured - predicted
% %             obj.errorZ = [obj.z(1) - r; 
% %                           g2o.stuff.normalize_theta(obj.z(2) - beta)];
% %         end
% % 
% %         function linearizeOplus(obj)
% %             platformState = obj.edgeVertices{1}.estimate();
% %             landmarkPos = obj.edgeVertices{2}.estimate();
% %             dx = landmarkPos(1) - platformState(1);
% %             dy = landmarkPos(2) - platformState(2);
% %             r = sqrt(dx^2 + dy^2);
% % 
% %             % Jacobian w.r.t. platform (vertex 1)
% %             dR_dx = -dx / r;
% %             dR_dy = -dy / r;
% %             dB_dx = dy / r^2;
% %             dB_dy = -dx / r^2;
% %             dB_dtheta = -1;
% %             obj.J{1} = -[dR_dx, dR_dy, 0;
% %                          dB_dx, dB_dy, dB_dtheta];
% % 
% %             % Jacobian w.r.t. landmark (vertex 2)
% %             dR_dlx = dx / r;
% %             dR_dly = dy / r;
% %             dB_dlx = -dy / r^2;
% %             dB_dly = dx / r^2;
% %             obj.J{2} = -[dR_dlx, dR_dly;
% %                          dB_dlx, dB_dly];
% %         end
% %     end
% % end
% 
% classdef LandmarkRangeBearingEdge < g2o.core.BaseBinaryEdge
%     % LandmarkRangeBearingEdge summary of LandmarkRangeBearingEdge
%     %
%     % This class stores an edge which represents the factor for observing
%     % the range and bearing of a landmark from the vehicle. Note that the
%     % sensor is fixed to the platform.
%     %
%     % The measurement model is
%     %
%     %    z_(k+1)=h[x_(k+1)]+w_(k+1)
%     %
%     % The measurements are r_(k+1) and beta_(k+1) and are given as follows.
%     % The sensor is at (lx, ly).
%     %
%     %    dx = lx - x_(k+1); dy = ly - y_(k+1)
%     %
%     %    r(k+1) = sqrt(dx^2+dy^2)
%     %    beta(k+1) = atan2(dy, dx) - theta_(k+1)
%     %
%     % The error term
%     %    e(x,z) = z(k+1) - h[x(k+1)]
%     %
%     % However, remember that angle wrapping is required, so you will need
%     % to handle this appropriately in compute error.
%     %
%     % Note this requires estimates from two vertices - x_(k+1) and l_(k+1).
%     % Therefore, this inherits from a binary edge. We use the convention
%     % that vertex slot 1 contains x_(k+1) and slot 2 contains l_(k+1).
% 
%     methods(Access = public)
% 
%         function obj = LandmarkRangeBearingEdge()
%             % LandmarkRangeBearingEdge for LandmarkRangeBearingEdge
%             %
%             % Syntax:
%             %   obj = LandmarkRangeBearingEdge(landmark);
%             %
%             % Description:
%             %   Creates an instance of the LandmarkRangeBearingEdge object.
%             %   Note we feed in to the constructor the landmark position.
%             %   This is to show there is another way to implement this
%             %   functionality from the range bearing edge from activity 3.
%             %
%             % Inputs:
%             %   landmark - (2x1 double vector)
%             %       The (lx,ly) position of the landmark
%             %
%             % Outputs:
%             %   obj - (handle)
%             %       An instance of a ObjectGPSMeasurementEdge
% 
%             obj = obj@g2o.core.BaseBinaryEdge(2);
%             info_matrix = diag([1/(0.1^2), 1/(0.05^2)]); % Inverse covariance
%             obj.setInformation(info_matrix);
%         end
% 
%         function initialEstimate(obj)
%             % INITIALESTIMATE Compute the initial estimate of the landmark.
%             %
%             % Syntax:
%             %   obj.initialEstimate();
%             %
%             % Description:
%             %   Compute the initial estimate of the landmark given the
%             %   platform pose and observation.
% 
%            % warning('LandmarkRangeBearingEdge.initialEstimate: implement')
%                %    z_(k+1)=h[x_(k+1)]+w_(k+1)
%             %
%             % The measurements are r_(k+1) and beta_(k+1) and are given as follows.
%             % The sensor is at (lx, ly).
%             %
%             %    dx = lx - x_(k+1); dy = ly - y_(k+1)
%             %
%             %    r(k+1) = sqrt(dx^2+dy^2)
%             %    beta(k+1) = atan2(dy, dx) - theta_(k+1)
%             %
%             % The error term
%             %    e(x,z) = z(k+1) - h[x(k+1)]
% 
%             x_k = obj.edgeVertices{1}.estimate();
%             x_pos = x_k(1);
%             y_pos = x_k(2);
%             theta = x_k(3);
% 
%             range = obj.z(1);
%             bearing = obj.z(2);
%             %obj.setMeasurement(obj.z);
% 
%             lx = x_pos + range * cos(theta + bearing); 
%             ly = y_pos + range * sin(theta + bearing); 
% 
%             obj.edgeVertices{2}.setEstimate([lx; ly]);
% 
% 
%         end
% 
%         function computeError(obj)
%             % COMPUTEERROR Compute the error for the edge.
%             %
%             % Syntax:
%             %   obj.computeError();
%             %
%             % Description:
%             %   Compute the value of the error, which is the difference
%             %   between the predicted and actual range-bearing measurement.
% 
%             %warning('LandmarkRangeBearingEdge.computeError: implement')
%                 % The error term
%         %    e(x,z) = z(k+1) - h[x(k+1)]
%             platformState = obj.edgeVertices{1}.estimate();
%             landmarkPos = obj.edgeVertices{2}.estimate();
% 
%             dx = landmarkPos(1) - platformState(1);
%             dy = landmarkPos(2) - platformState(2);
% 
%             r = sqrt(dx^2 + dy^2);
%             beta = atan2(dy, dx) - platformState(3);
% 
%             % Error = measured - predicted
%             obj.errorZ = [obj.z(1) - r; 
%             g2o.stuff.normalize_theta(obj.z(2) - beta)];
%             %obj.setMeasurement([r; g2o.stuff.normalize_theta(obj.z(2))]);
% 
%         end
% 
%         function linearizeOplus(obj)
%             % linearizeOplus Compute the Jacobian of the error in the edge.
%             %
%             % Syntax:
%             %   obj.linearizeOplus();
%             %
%             % Description:
%             %   Compute the Jacobian of the error function with respect to
%             %   the vertex.
%             %
% 
%            % warning('LandmarkRangeBearingEdge.linearizeOplus: implement')
%             x_k = obj.edgeVertices{1}.estimate();
%             L = obj.edgeVertices{2}.estimate();
% 
%             dx = L(1) - x_k(1);
%             dy = L(2) - x_k(2);
% 
%             q = dx^2 + dy^2;
%             r = sqrt(q);
% 
%             obj.J{1} = [(dx/r) (dy/r) 0;
%                         -(dy/q) (dx/q) 1];
% 
%             obj.J{2} = [-(dx/r) -(dy/r);
%                         (dy/q) -(dx/q)];
% 
%             % platformState = obj.edgeVertices{1}.estimate();
%             % landmarkPos = obj.edgeVertices{2}.estimate();
%             % dx = landmarkPos(1) - platformState(1);
%             % dy = landmarkPos(2) - platformState(2);
%             % r = sqrt(dx^2 + dy^2);
%             % 
%             % % Jacobian w.r.t. platform (vertex 1)
%             % dR_dx = -dx / r;
%             % dR_dy = -dy / r;
%             % dB_dx = dy / r^2;
%             % dB_dy = -dx / r^2;
%             % dB_dtheta = -1;
%             % obj.J{1} = -[dR_dx, dR_dy, 0;
%             %              dB_dx, dB_dy, dB_dtheta];
%             % 
%             % % Jacobian w.r.t. landmark (vertex 2)
%             % dR_dlx = dx / r;
%             % dR_dly = dy / r;
%             % dB_dlx = -dy / r^2;
%             % dB_dly = dx / r^2;
%             % obj.J{2} = -[dR_dlx, dR_dly;
%             %              dB_dlx, dB_dly];
%         end        
%     end
% end

classdef LandmarkRangeBearingEdge < g2o.core.BaseBinaryEdge
    % LandmarkRangeBearingEdge summary of LandmarkRangeBearingEdge
    %
    % This class stores an edge which represents the factor for observing
    % the range and bearing of a landmark from the vehicle. Note that the
    % sensor is fixed to the platform.
    %
    % The measurement model is
    %
    %    z_(k+1)=h[x_(k+1)]+w_(k+1)
    %
    % The measurements are r_(k+1) and beta_(k+1) and are given as follows.
    % The sensor is at (lx, ly).
    %
    %    dx = lx - x_(k+1); dy = ly - y_(k+1)
    %
    %    r(k+1) = sqrt(dx^2+dy^2)
    %    beta(k+1) = atan2(dy, dx) - theta_(k+1)
    %
    % The error term
    %    e(x,z) = z(k+1) - h[x(k+1)]
    %
    % However, remember that angle wrapping is required, so you will need
    % to handle this appropriately in compute error.
    %
    % Note this requires estimates from two vertices - x_(k+1) and l_(k+1).
    % Therefore, this inherits from a binary edge. We use the convention
    % that vertex slot 1 contains x_(k+1) and slot 2 contains l_(k+1).
    
    methods(Access = public)
    
        function obj = LandmarkRangeBearingEdge()
            % LandmarkRangeBearingEdge for LandmarkRangeBearingEdge
            %
            % Syntax:
            %   obj = LandmarkRangeBearingEdge(landmark);
            %
            % Description:
            %   Creates an instance of the LandmarkRangeBearingEdge object.
            %   Note we feed in to the constructor the landmark position.
            %   This is to show there is another way to implement this
            %   functionality from the range bearing edge from activity 3.
            %
            % Inputs:
            %   landmark - (2x1 double vector)
            %       The (lx,ly) position of the landmark
            %
            % Outputs:
            %   obj - (handle)
            %       An instance of a ObjectGPSMeasurementEdge

            obj = obj@g2o.core.BaseBinaryEdge(2);
        end
        
        function initialEstimate(obj)
            % INITIALESTIMATE Compute the initial estimate of the landmark.
            %
            % Syntax:
            %   obj.initialEstimate();
            %
            % Description:
            %   Compute the initial estimate of the landmark given the
            %   platform pose and observation.

          %  warning('LandmarkRangeBearingEdge.initialEstimate: implement')
              % The measurement model is
            %
            %    z_(k+1)=h[x_(k+1)]+w_(k+1)
            %
            % The measurements are r_(k+1) and beta_(k+1) and are given as follows.
            % The sensor is at (lx, ly).
            %
            %    dx = lx - x_(k+1); dy = ly - y_(k+1)
            %
            %    r(k+1) = sqrt(dx^2+dy^2)
            %    beta(k+1) = atan2(dy, dx) - theta_(k+1)

            x_k = obj.edgeVertices{1}.estimate();
            x_pos = x_k(1);
            y_pos = x_k(2);
            theta = x_k(3);

            range = obj.z(1);
            bearing = obj.z(2);
            %obj.setMeasurement(obj.z);

            lx = x_pos + range * cos(theta + bearing); 
            ly = y_pos + range * sin(theta + bearing); 

            obj.edgeVertices{2}.setEstimate([lx; ly]);

           % lx = obj.edgeVertices{1}.x(1:2);
            %obj.edgeVertices{2}.setEstimate(lx);

        end
        
        function computeError(obj)
            % COMPUTEERROR Compute the error for the edge.
            %
            % Syntax:
            %   obj.computeError();
            %
            % Description:
            %   Compute the value of the error, which is the difference
            %   between the predicted and actual range-bearing measurement.

           % warning('LandmarkRangeBearingEdge.computeError: implement')
            error = obj.z - obj.edgeVertices{2}.estimate();
            estimate = obj.edgeVertices{2}.estimate();
            error(2) = g2o.stuff.normalize_theta(obj.z(2) - estimate(2));
            obj.errorZ = error/2;
            obj.setMeasurement(obj.z);
        end
        
        function linearizeOplus(obj)
            % linearizeOplus Compute the Jacobian of the error in the edge.
            %
            % Syntax:
            %   obj.linearizeOplus();
            %
            % Description:
            %   Compute the Jacobian of the error function with respect to
            %   the vertex.
            %

           % warning('LandmarkRangeBearingEdge.linearizeOplus: implement')

            obj.J{1} = eye(2, 3);

            obj.J{2} = eye(2);

            %             x_k = obj.edgeVertices{1}.estimate();
%             L = obj.edgeVertices{2}.estimate();
% 
%             dx = L(1) - x_k(1);
%             dy = L(2) - x_k(2);
% 
%             q = dx^2 + dy^2;
%             r = sqrt(q);
% 
%             obj.J{1} = [(dx/r) (dy/r) 0;
%                         -(dy/q) (dx/q) 1];
% 
%             obj.J{2} = [-(dx/r) -(dy/r);
%                         (dy/q) -(dx/q)];
% 
            % platformState = obj.edgeVertices{1}.estimate();
            % landmarkPos = obj.edgeVertices{2}.estimate();
            % dx = landmarkPos(1) - platformState(1);
            % dy = landmarkPos(2) - platformState(2);
            % r = sqrt(dx^2 + dy^2);
            % 
            % % Jacobian w.r.t. platform (vertex 1)
            % dR_dx = -dx / r;
            % dR_dy = -dy / r;
            % dB_dx = dy / r^2;
            % dB_dy = -dx / r^2;
            % dB_dtheta = -1;
            % obj.J{1} = [dR_dx, dR_dy, 0;
            %              dB_dx, dB_dy, dB_dtheta];
            % 
            % % Jacobian w.r.t. landmark (vertex 2)
            % dR_dlx = dx / r;
            % dR_dly = dy / r;
            % dB_dlx = -dy / r^2;
            % dB_dly = dx / r^2;
            % obj.J{2} = -[dR_dlx, dR_dly;
            %              dB_dlx, dB_dly];

            x = obj.edgeVertices{1}.estimate();
            dx = 0.5 * (obj.edgeVertices{2}.estimate() - x(1:2));
            r = norm(dx);
            
            obj.J{1} = ...
                [-dx(1)/r -dx(2)/r 0;
                dx(2)/r^2 -dx(1)/r^2 -1];
            % obj.J{2} = [ dx(1)/r,   dx(2)/r;
            %             -dx(2)/r^2, dx(1)/r^2 ];

        end        
    end
end