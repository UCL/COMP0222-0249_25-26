classdef WaypointController < ebe.core.ConfigurableComponent

    properties(Access = protected)

        waypoints;
        numWaypoints;
        waypointIndex;
        u;
        platformConfig;
        controllerConfig;

    end

    methods(Access = public)

        function obj = WaypointController(config)
            obj@ebe.core.ConfigurableComponent(config);
            obj.platformConfig = obj.config.platform;
            obj.controllerConfig = obj.config.platform.controller;
        end

        function start(obj)

            obj.waypointIndex = 1;
            obj.waypoints = obj.config.platformTrajectory.waypoints';

            obj.numWaypoints = size(obj.waypoints, 2);

            obj.u=[0;0];

        end

        function u = computeControlInputs(obj, x)
            
            % Work out distance to the target waypoint
            dX = obj.waypoints(:, obj.waypointIndex) - x(1:2);
            d = norm(dX);
            
            % If sufficiently close, switch to the next waypoint;
            if (d < 1)

                % If we've reached the end of the list of waypoints, return
                if (obj.waypointIndex == obj.numWaypoints)
                    %u = [];
                    %return;
                    obj.waypointIndex = 0;
                end

                obj.waypointIndex = obj.waypointIndex + 1;
                
                % Update to the new waypoint
                dX = x(1:2) - obj.waypoints(:, obj.waypointIndex);
                d = norm(dX);
            end
            
            % Compute the speed. We first clamp the acceleration, and then
            % clamp the maximum and minimum speed values.
            diffSpeed = 0.1 * d - obj.u(1);
            maxDiffSpeed = obj.controllerConfig.maxAcceleration * obj.controllerConfig.odomUpdatePeriod;
            diffSpeed = min(maxDiffSpeed, max(-maxDiffSpeed, diffSpeed));
            obj.u(1) = max(obj.controllerConfig.minSpeed, min(obj.controllerConfig.maxSpeed, obj.u(1) + diffSpeed));

            % Compute the steer angle. We first clamp the rate of change,
            % and then clamp the maximum and minimum steer angles.
            diffDelta = g2o.stuff.normalize_theta(atan2(dX(2), dX(1)) - x(3) - obj.u(2));
            maxDiffDelta = obj.controllerConfig.maxDiffDeltaRate * obj.controllerConfig.odomUpdatePeriod;
            diffDelta = min(maxDiffDelta, max(-maxDiffDelta, diffDelta));
            obj.u(2) = min(obj.controllerConfig.maxDelta, max(-obj.controllerConfig.maxDelta, obj.u(2) + diffDelta));

            % Now work out what the control input should bel
            psiDot = obj.u(1) * sin(obj.u(2)) / obj.platformConfig.B;
            u = [obj.u(1); psiDot];
        end
    end
end