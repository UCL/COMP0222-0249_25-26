classdef SLAMSystem < ebe.slam.SLAMSystem

    properties(Access = protected)

        % Although in the lectures we presented three types of states
        % (estimated, predicted, partial) from an implementation point of
        % view it's easier just to store the one set
        x;
        P;
        u;

        % Map stores landmark ID with the indices in the state vector
        landmarkIDStateVectorMap;

        % The linear system used to predict the state and the observation
        systemModel;

        % The map
        map

        % Store of the mean and covariance values
        timeStore;
        xStore;
        PStore;

        % For activity 4
        muckUp;
    end

    methods(Access = public)

        function obj = SLAMSystem(config)

            % Call base class
            obj@ebe.slam.SLAMSystem(config);

            % Set up the discrete time system for prediction
            obj.systemModel = l2s.dotbot.SystemModel(config);


            obj.muckUp = false;

            % Set up the event handlers
            obj.registerEventHandler('init', @obj.handleInitializationEvent);
            obj.registerEventHandler('null_obs', @obj.handleNoUpdate);
            obj.registerEventHandler('gps', @obj.handleGPSObservationEvent);
            obj.registerEventHandler('slam', @obj.handleSLAMObservationEvent);
            obj.registerEventHandler('odom', @obj.handleUpdateOdometryEvent);
            obj.registerEventHandler('bearing', @obj.handleBearingObservationEvent);

            % Set the name
            obj.setName('SLAMSystem');
        end

        function success = start(obj)
            start@ebe.slam.SLAMSystem(obj);
            obj.timeStore = [];
            obj.xStore = zeros(l2s.dotbot.SystemModel.NP, 0);
            obj.PStore = zeros(l2s.dotbot.SystemModel.NP, 0);

            % Set the dictionary which maps landmark ID to coefficient in
            % the state estimate.
            obj.landmarkIDStateVectorMap = configureDictionary("uint32", "double");

            % Get the map data
            if (isfield(obj.config, 'map'))
                obj.map = obj.config.map;
            end

            success = true;
        end

        % Return the current platform estimate
        function [x,P] = platformEstimate(obj)
            x = obj.x(1:l2s.dotbot.SystemModel.NP);
            P = obj.P(1:l2s.dotbot.SystemModel.NP, 1:l2s.dotbot.SystemModel.NP);
        end
        
        function [T, X, PX] = platformEstimateHistory(obj)
            T = obj.timeStore;
            X = obj.xStore;
            PX = obj.PStore;
        end
        
        function [x, P, landmarkIds] = landmarkEstimates(obj)

            landmarkIds = keys(obj.landmarkIDStateVectorMap);

            numberOfLandmarks = numel(landmarkIds);
           
            x = NaN(l2s.dotbot.SystemModel.NL, numberOfLandmarks);
            P = NaN(l2s.dotbot.SystemModel.NL, l2s.dotbot.SystemModel.NL, numberOfLandmarks);
            
            for l = 1 : numberOfLandmarks
                landmarkId = landmarkIds(l);
                offset = lookup(obj.landmarkIDStateVectorMap, landmarkId);
                idx = offset + (1:l2s.dotbot.SystemModel.NL);
                x(:, l) = obj.x(idx);
                P(:, :, l) = obj.P(idx, idx);
            end
            
        end

        function [T, X, PX] = estimateHistory(obj)
            T = obj.timeStore;
            X = obj.xStore;
            PX = obj.PStore;
        end

        function muckUpCovarianceMatrix(obj, muckUp)
            obj.muckUp = muckUp;
        end
    end

    methods(Access = protected)

        function success = handleNoPrediction(obj)
            obj.x = obj.x;
            obj.P = obj.P;
            success = true;
        end

        function success = handleNoUpdate(obj, ~)
            obj.x = obj.x;
            obj.P = obj.P;
            success = true;
        end

        function success = handlePredictForwards(obj, dT)

            NP = l2s.dotbot.SystemModel.NP;

            [obj.x(1:NP), FXd, QXd] = obj.systemModel.predictState(obj.x(1:NP), obj.u, dT);

            % This is an inefficient way to do it:
            %
            % FS = eye(numel(obj.x));
            % FS(1:NP, 1:NP) = FXd;
            % QS = eye(numel(obj.x));
            % QS(1:NP,1:NP) = QXd;
            % obj.P = FS * obj.P * FS' + QS
            %
            % The more efficient approach is exploit the structure of FS
            % and QS and expand by hand

            % Multiply the top left block for the platform state
            obj.P(1:NP,1:NP) = FXd * obj.P(1:NP, 1:NP) * FXd' + QXd;

            % Do the platform landmark-prediction blocks
            obj.P(1:NP, NP+1:end) = FXd * obj.P(1:NP, NP+1:end);
            obj.P(NP+1:end, 1:NP) = obj.P(1:NP, NP+1:end)';

            success = true;
        end

        function success = handleInitializationEvent(obj, event)
            obj.x = event.data;
            obj.P = event.covariance;
            obj.initialized = true;
            success = true;
        end

        function success = handleGPSObservationEvent(obj, event)
            [zPred, Hx, R] = obj.systemModel.predictGPSObservation(obj.x(1:2));

            % Expand to the full state
            HS = zeros(2, numel(obj.x));
            HS(:, 1:l2s.dotbot.SystemModel.NP) = Hx;

            % Kalman Filter Update
            nu = event.data - zPred;
            C = obj.P * HS';
            S = HS * C + R;
            W = C / S;
            obj.x = obj.x + W * nu;
            obj.P = obj.P - W * S * W';
            success = true;
        end

        function success = handleBearingObservationEvent(obj, event)

            x = obj.x;
            P = obj.P;

            % Update each measurement separately
            for s = 1 : numel(event.info)
                sensor = obj.map.sensors.bearing.sensors(event.info(s));
                [zPred, Hx, R] = obj.systemModel.predictBearingObservation(x(1:2), sensor.position, sensor.orientation);

                % Expand to full state
                HS = zeros(1, numel(obj.x));
                HS(:, 1:l2s.dotbot.SystemModel.NP) = Hx;
                
                % Kalman Filter Update
                nu = event.data(s) - zPred;
                nu = atan2(sin(nu), cos(nu));
                C = P * HS';
                S = HS * C + R;
                W = C / S;
                x = x + W * nu;
                P = P - W * S * W';
            end
            obj.x = x;
            obj.P = P;
            success = true;
        end

        % Handle a set of measurements of landmarks
        function handleSLAMObservationEvent(obj, event)
            assert(obj.stepNumber == event.eventGeneratorStepNumber)

            % Store useful values
            NL = l2s.dotbot.SystemModel.NL;
            NP = l2s.dotbot.SystemModel.NP;

            % Get the list of landmarks we know about
            knownLandmarkIDs = obj.landmarkIDStateVectorMap.keys();

            % Find the set of known landmarks in the observation set
            [existingLandmarks, idx] = intersect(event.info, knownLandmarkIDs);

            % First update with the known landmarks

            for o = 1 : numel(existingLandmarks)

                % The following two lines look up, for each known landmark,
                % what the index of that landmark state is in the state
                % vector
                offset = lookup(obj.landmarkIDStateVectorMap, existingLandmarks(o));
                landmarkIdx = offset + (1:NL);

                [zPred, Hx, Hm, ~] = ...
                    obj.systemModel.predictSLAMObservation(obj.x(1:NP), ...
                    obj.x(landmarkIdx));

                % ACTIVITY 3:
                %
                % Create the SLAM system observation matrix HS which
                % has to be of the right size and composition to update the
                % landmark and platform.
                HS = zeros(2, numel(obj.x));
                HS(:, 1:NP) = Hx;
                HS(:, landmarkIdx) = Hm;

                % Use HS to implement the Kalman filter update for the SLAM
                % system
                C = obj.P * HS';
                S = HS * C + event.covariance();
                nu = event.data(:, idx(o)) - zPred;
                K = C / S;
                obj.x = obj.x + K * nu;
                obj.P = obj.P - K * S * K';
            end

            % The remaining observations are of new landmarks
            [newLandmarks, idx] = setdiff(event.info, existingLandmarks);

            for o = 1 : numel(newLandmarks)

                % The following three lines are used to update the "book
                % keeping" in the SLAM system to tie landmark IDs with
                % their position the state vector. This code is currently
                % commented out (so the code will run). However, you MUST
                % uncomment it when you are implementing your solution to
                % activity 3, otherwise the code will not work properly.
                % Note that landmarkIdx is the index in the state vector
                % where the new landmark will be inserted.

                offset = length(obj.x);
                landmarkIdx = offset + (1:NL);
                obj.landmarkIDStateVectorMap = insert(obj.landmarkIDStateVectorMap, newLandmarks(o), offset);

                % ACTIVITY 2: Insert here the correct code to compute the J
                % and K matrices needed to augment the state with the
                % landmark. Check the slides on the augmentation operation
                % and the appendix of the coursework for the appropriate
                % expressions.

                J = zeros(offset + NL, offset);
                J(1:offset, 1:offset) = eye(offset);
                J(landmarkIdx, 1:NL) = eye(NL);

                K = zeros(offset + NL, NL);
                K(landmarkIdx, 1:NL) = eye(NL);

                obj.x = J * obj.x + K * event.data(:, idx(o));
                obj.P = J * obj.P * J' + K * event.covariance() * K';
            end

            % ACTIVITY 4
            if (obj.muckUp == true)
                for k = 1 : 2: length(obj.P)
                    obj.P(k:k+1, 1:k-1) = 0;
                    obj.P(k:k+1, k+2:end) = 0;
                end
            end
        end

        function success = handleUpdateOdometryEvent(obj, event)
                assert(obj.stepNumber == event.eventGeneratorStepNumber);
                obj.u = event.data;
                success = true;
            end

        function storeStepResults(obj)
            % Store the estimate for the future
            obj.timeStore(:, obj.stepNumber + 1) = obj.currentTime;
            obj.xStore(:, obj.stepNumber + 1) = obj.x(1:l2s.dotbot.SystemModel.NP);
            obj.PStore(:, obj.stepNumber + 1) = diag(obj.P(1:l2s.dotbot.SystemModel.NP, ...
                1:l2s.dotbot.SystemModel.NP));

        end
    end
end