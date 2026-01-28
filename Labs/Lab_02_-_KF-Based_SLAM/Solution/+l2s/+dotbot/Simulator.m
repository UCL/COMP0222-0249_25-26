classdef Simulator < ebe.core.EventBasedSimulator

    % This class simulates dot bot. Note that there is a cheat going on
    % here. The actual simulator runs with (x,y,theta) but only returns
    % (x,y). The reason is that we want a LINEAR system for this lab. For
    % lab 03, the full model will be directly exposed.

    properties(Access = public)
        % The state
        x;

        % The latest control input
        u;

        % The map which is the model of the environment
        map;

        % The landmarks
        landmarks

        % The linear system used to predict the state and the observation
        systemModel;

        % The controller
        platformController;

        % The store of the ground truth state and observations
        xTrueStore;
        timeStore;

    end

    methods(Access = public)
        
        % Construct the object
        function obj = Simulator(config)
            obj@ebe.core.EventBasedSimulator(config);
            obj.systemModel = l2s.dotbot.SimulatorSystemModel(config, config.perturbWithNoise);
            obj.platformController = l2s.dotbot.WaypointController(obj.config);
        end
        
        % Get the ground truth
        function x = xTrue(obj)
            x = obj.x(1:2);
        end

        function [timeHistory, xTrueHistory] = history(obj)
            timeHistory = obj.timeStore;
            xTrueHistory = obj.xTrueStore;
        end

        function start(obj)

            % Call the base class start functions
            start@ebe.core.EventBasedSimulator(obj);

            obj.platformController.start();

            % Prestore the results
            obj.timeStore = NaN(1, 0);%obj.config.maximumStepNumber + 1);
            obj.xTrueStore = NaN(length(obj.config.x0));%, obj.config.maximumStepNumber + 1);

            % Set the initialization callbabk
            obj.eventGeneratorQueue.insert(0, @obj.initialize);

            % If the map was not loaded, reject
            if ((isfield(obj.config, 'map') == false) || (isstruct(obj.config.map) == false))
                return;
            end

            obj.map = obj.config.map;

            % Are there any landmarks?
            if (isfield(obj.map, 'landmarks') == false)
                return
            end

            % Get the landmarks
            slamLandmarks = obj.map.landmarks.slam;

            % Just handle random case for now
            if (strcmp(slamLandmarks.configuration, 'random') == true)
                lms = [slamLandmarks.x_min;slamLandmarks.y_min] + ...
                    [slamLandmarks.x_max-slamLandmarks.x_min;slamLandmarks.y_max-slamLandmarks.y_min] .* ...
                    rand(2, slamLandmarks.numLandmarks);
            else
                lms = slamLandmarks.landmarks';
            end

            obj.landmarks = lms;
        end

        function map = getMap(obj)
            map = obj.map;
        end

    end

    methods(Access = protected)

        function handlePredictForwards(obj, dT, ~)
            obj.x = obj.systemModel.predictState(obj.x, obj.u, dT);
        end

        function initialize(obj)
            assert(obj.stepNumber == 0)

            % Initialize the ground truth state
            P0 = obj.config.P0;
            P0(3,3) = P0(3,3) * (pi / 180)^2;
            obj.x = obj.config.x0 + obj.noiseScale * ebe.utils.psd_sqrtm(P0) * randn(size(obj.config.x0));
            obj.initialized = true;

            % Construct and post the init event
            event = ebe.core.Event(obj.currentTime, 'init', obj.config.x0(1:2), obj.config.P0(1:2, 1:2));
            event.eventGeneratorStepNumber = obj.stepNumber;
            obj.outgoingEvents.insert(event);

            % Force the first odometry event; this also schedules
            % subsequent events
            obj.updateOdometry();

            % Schedule the timeout event
            obj.eventGeneratorQueue.insert(obj.currentTime + obj.config.heartbeatPeriod, ...
                    @obj.generateHeartbeat);

            % If no map is defined, we have no sensors to simulate
            if (isfield(obj.config, 'map') == false)
                return
            end

            % Schedule the bearing measurement
            if (isfield(obj.config.map.sensors, 'bearing')  == true)  && (obj.config.map.sensors.bearing.enabled == true)
                obj.eventGeneratorQueue.insert(obj.currentTime + obj.config.map.sensors.bearing.measurementPeriod, ...
                    @obj.predictBearingObservations);
            end

            % Schedule the GPS measurement
            if (isfield(obj.config.map.sensors, 'gps') == true) && (obj.config.map.sensors.gps.enabled == true)
                obj.eventGeneratorQueue.insert(obj.currentTime + obj.config.map.sensors.gps.measurementPeriod, ...
                    @obj.predictGPSObservation);
            end

             % Schedule the SLAM measurement
            if (isfield(obj.config.map.sensors, 'slam')  == true)  && (obj.config.map.sensors.slam.enabled == true)
                obj.eventGeneratorQueue.insert(obj.currentTime + obj.config.map.sensors.slam.measurementPeriod, ...
                    @obj.predictSLAMObservations);
            end      
        end

        function updateOdometry(obj)
         obj.u = obj.platformController.computeControlInputs(obj.x);
            if (isempty(obj.u))
                obj.carryOnRunning = false;
                return
            end

            % The odometry is specified as (x,y) velocity
            linearU = obj.u(1) * [cos(obj.x(3)); sin(obj.x(3))];

            linearU = linearU + obj.noiseScale * obj.config.platform.controller.odomSigma * randn(2, 1);
            
            % We pitch this as a velocity measurement
            event = ebe.core.Event(obj.currentTime, 'odom', linearU);
            event.eventGeneratorStepNumber = obj.stepNumber;

            obj.outgoingEvents.insert(event);
             obj.eventGeneratorQueue.insert(obj.currentTime + obj.config.platform.controller.odomUpdatePeriod, ...
                    @obj.updateOdometry);            
        end

        function predictBearingObservations(obj)

            sensorIDs = obj.isDetectedByBearingSensors();

            if (isempty(sensorIDs) == true)
                event = ebe.core.Event(obj.currentTime, 'null_obs');
            else
                nz = numel(sensorIDs);
                z = zeros(1, nz);
                for s = 1 : nz
                    sensor = obj.map.sensors.bearing.sensors(sensorIDs(s));
                    [z(s), R] = obj.systemModel.predictBearingObservation(obj.x, sensor.position, sensor.orientation);
                end
                event = ebe.core.Event(obj.currentTime, 'bearing', z, ...
                    R, sensorIDs); 
            end
            event.eventGeneratorStepNumber = obj.stepNumber;

            obj.outgoingEvents.insert(event);
            obj.eventGeneratorQueue.insert(obj.currentTime + ...
                obj.config.map.sensors.bearing.measurementPeriod, @obj.predictBearingObservations);            
        end

        function predictGPSObservation(obj)

            % We get the GPS observation only if the dotbot is not under an
            % occluder
            if (obj.isWithinOccluder() == true)
                event = ebe.core.Event(obj.currentTime, 'null_obs');
            else
                % Generate the observation
                [z, R] = obj.systemModel.predictGPSObservation(obj.x);

                % Post the event
                event = ebe.core.Event(obj.currentTime, 'gps', z, R);
            end
            event.eventGeneratorStepNumber = obj.stepNumber;

            obj.outgoingEvents.insert(event);
            obj.eventGeneratorQueue.insert(obj.currentTime + obj.config.map.sensors.gps.measurementPeriod, @obj.predictGPSObservation);
        end

        function predictSLAMObservations(obj)

            % Work out the relative distance to all the robots
            dX = obj.landmarks - obj.x(1:2);
            
            % Squared range to each landmark
            R2 = sum(dX.^2,1);
            R = sqrt(R2);

            % Find all the landmarks in range
            landmarkIDs = find(R <= obj.config.map.sensors.slam.detectionRange);
            
            if (isempty(landmarkIDs) == true)
                event = ebe.core.Event(obj.currentTime, 'null_obs');
            else
                numLandmarks = length(landmarkIDs);
                z = zeros(2, numLandmarks);
                [z(:, 1), R] = obj.systemModel.predictSLAMObservation(obj.x, obj.landmarks(:, landmarkIDs(1)));
                for l  = 2 : numLandmarks
                    z(:, l) = obj.systemModel.predictSLAMObservation(obj.x, obj.landmarks(:, landmarkIDs(l)));
                end
                event = ebe.core.Event(obj.currentTime, 'slam', z, R, landmarkIDs);
            end
            event.eventGeneratorStepNumber = obj.stepNumber;

            obj.outgoingEvents.insert(event);
            obj.eventGeneratorQueue.insert(obj.currentTime + obj.config.map.sensors.slam.measurementPeriod, @obj.predictSLAMObservations);
        end

        function generateHeartbeat(obj)
            event = ebe.core.Event(obj.currentTime, 'null_obs');
            event.eventGeneratorStepNumber = obj.stepNumber;
            obj.outgoingEvents.insert(event);
            obj.eventGeneratorQueue.insert(obj.currentTime + obj.config.heartbeatPeriod, ...
                    @obj.generateHeartbeat);
        end

        function storeStepResults(obj)
            % Store
            obj.stepNumber;
            obj.timeStore(obj.stepNumber + 1) = obj.currentTime;
            obj.xTrueStore(:, obj.stepNumber + 1) = obj.x;
        end
    end

    methods(Access = protected)

        % Method to check if a point is within any occluder
        function isInside = isWithinOccluder(obj)

            if (isfield(obj.map.sensors.gps, 'occluders') == false)
                isInside = false;
                return
            end

            occluders = obj.map.sensors.gps.occluders;

            isInside = false;
            for i = 1:length(occluders)
                occluder = occluders(i);
                if obj.x(1) >= occluder.x_min && obj.x(1) <= occluder.x_max && ...
                   obj.x(3) >= occluder.y_min && obj.x(3) <= occluder.y_max
                    isInside = true;
                    return;
                end
            end
        end
        
        % Method to check if a point is visible to any sensor
        function [sensorIDs] = isDetectedByBearingSensors(obj)
            sensorIDs = [];
            sensors = obj.map.sensors.bearing.sensors;

            for s = 1:length(sensors)
                sensor = sensors(s);
                sensorPos = sensor.position;
                dx = obj.x(1) - sensorPos(1);
                dy = obj.x(3) - sensorPos(2);
                distance = hypot(dx, dy);
                
                if distance <= sensor.detectionRange
                    angleToPoint = atan2(dy, dx);
                    relativeAngle = mod(angleToPoint - deg2rad(sensor.orientation) + pi, 2*pi) - pi;
                    
                    if abs(relativeAngle) <= deg2rad(sensor.detectionAngle)/2
                        sensorIDs(end + 1) = s;
                    end
                end
            end
        end
    end
end