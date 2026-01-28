classdef SimulatorSystemModel < handle

    % This system model is ONLY used by the simulator. Normally the system
    % model is shared between the simulator and the estimator. However, for
    % this lab only, there is a significant difference between the two
    % models - the actual simulator uses a nonlinear model with a vehicle
    % with orientation, but the SLAM system uses a linear model.

    properties(Access = protected)

        % Process noise sample
        Qd;
        QdSqrtm;

        % Observation model
        HGPS;

        % GPS
        RGPS;
        RGPSSqrtm;

        % Bearing
        RBearing;
        RBearingSqrt;

        % SLAM
        RSLAM;
        RSLAMSqrt;

        % The configuration
        config;

        % Short cut to check if we are simulating noise or not
        perturbWithNoise;
    end

    methods(Access = public)

        function obj = SimulatorSystemModel(config, perturbWithNoise)
            obj.config = config;

            if (nargin == 1)
                obj.perturbWithNoise = false;
            else
                obj.perturbWithNoise = perturbWithNoise;
            end
            
            obj.setupModels();
        end

        function [x] = predictState(obj, x, u, dT)

            sDT = u(1) * dT;

            x(1) = x(1) + sDT * cos(x(3));
            x(2) = x(2) + sDT * sin(x(3));
            x(3) = x(3) + dT * u(2);
            
            x(3) = atan2(sin(x(3)), cos(x(3)));

            if (nargout == 3)
                F = eye(3);
                F(1, 3) = -sDT * sin(x(3));
                F(2, 3)= sDT * cos(x(3));
                Q = obj.Qd;
            end
        end

        function [z, R] = predictGPSObservation(obj, x)

            z = obj.HGPS * x(1:2);

            if (obj.perturbWithNoise == true)
                z = z + obj.RGPSSqrtm * randn(size(z));
            end

            R = obj.RGPS;
        end

        function [z, R] = predictBearingObservation(obj, x, sensorXY, sensorTheta)

            dx = x(1) - sensorXY(1);
            dy = x(2) - sensorXY(2);
            z = atan2(dy, dx) - deg2rad(sensorTheta);
            z=atan2(sin(z), cos(z));

            if (obj.perturbWithNoise == true)
                z = z + obj.RBearingSqrt * randn(size(z));
            end

            z = g2o.stuff.normalize_theta(z);
            R = obj.RBearing;
        end

        function [z, R] = predictSLAMObservation(obj, x, lXY)

            z = lXY - x(1:2);

            if (obj.perturbWithNoise == true)
                z = z + obj.RSLAMSqrt * randn(size(z));
            end

            R = obj.RSLAM;

        end
    end

    methods(Access = protected)

        function setupModels(obj)
            % Set up the observation

            if (isfield(obj.config, 'map') == false)
                return
            end

            if (isfield(obj.config.map.sensors, 'gps'))
                obj.HGPS = eye(2);
                obj.RGPS = eye(2) * obj.config.map.sensors.gps.sigmaR^2;
    
                if (obj.perturbWithNoise == true)
                    obj.RGPSSqrtm = obj.config.map.sensors.gps.sigmaR;
                end
            end

            if (isfield(obj.config.map.sensors, 'bearing'))
                obj.RBearing = deg2rad(obj.config.map.sensors.bearing.sigmaR)^2;
                if (obj.perturbWithNoise == true)
                    obj.RBearingSqrt = deg2rad(obj.config.map.sensors.bearing.sigmaR);
                end
            end

            if (isfield(obj.config.map.sensors, 'slam'))
                obj.RSLAM = eye(2) * obj.config.map.sensors.slam.sigmaR^2;
                if (obj.perturbWithNoise == true)
                    obj.RSLAMSqrt = eye(2) * obj.config.map.sensors.slam.sigmaR;
                end
            end
        end
    end

end