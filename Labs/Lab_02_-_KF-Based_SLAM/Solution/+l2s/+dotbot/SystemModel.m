classdef SystemModel < handle

    properties(Access = public, Constant)
        % Platform state dimension
        NP = 2;
        
        % Landmark dimension
        NL = 2;
    end


    properties(GetAccess = public)

        % Continuous time process model for the platform state
        FXc;
        FXd;
        BXc;
        BXd;

        % Continuous time process noise model for the platform state
        QXc;
        QXd;
        QXdSqrtm;

        % Observation model for the platform state
        HXGPS;

        % GPS
        RGPS;
        RGPSSqrtm;

        % Bearing
        RBearing;
        RBearingSqrt;

        % SLAM
        RSLAM;
        RSLAMSqrt;

        cachedDT;

    end

    properties(Access = protected)
        config;
        perturbWithNoise;
    end

    methods(Access = public)

        function obj = SystemModel(config, perturbWithNoise)
            obj.config = config;

            if (nargin == 1)
                obj.perturbWithNoise = false;
            else
                obj.perturbWithNoise = perturbWithNoise;
            end
            
            obj.setupModels();

            obj.cachedDT = NaN;
        end

        function [x, FXd, QXd] = predictState(obj, x, u, dT)

            if (dT ~= obj.cachedDT)
                [obj.FXd, obj.QXd, obj.BXd] = ebe.utils.continuousToDiscrete(obj.FXc, ...
                    obj.QXc, obj.BXc, dT);
                if (obj.perturbWithNoise == true)
                    obj.QXdSqrtm = sqrtm(obj.QXd);
                end
                obj.cachedDT = dT;
            end

            pIdx = 1:l2s.dotbot.SystemModel.NP;
            x(pIdx) = obj.FXd * x(pIdx) + obj.BXd * u;

            if (obj.perturbWithNoise == true)
                x(pIdx) = x(pIdx) + obj.QXdSqrtm * randn(l2s.dotbot.SystemModel.NP, 1);
            end

            if (nargout == 3)
                FXd = obj.FXd;
                QXd = obj.QXd;
            end
        end

        function [z, Hx, R] = predictGPSObservation(obj, x)

           z = obj.HXGPS * x(1:l2s.dotbot.SystemModel.NP);

            if (obj.perturbWithNoise == true)
                z = z + obj.RGPSSqrtm * randn(size(z));
            end

            if (nargout == 3)
                Hx = obj.HXGPS;
                R = obj.RGPS;
            end
        end

        function [z, Hx, R] = predictBearingObservation(obj, x, sensorXY, sensorTheta)

            dx = x(1) - sensorXY(1);
            dy = x(2) - sensorXY(2);
            z = atan2(dy, dx) - sensorTheta * pi / 180;

            if (obj.perturbWithNoise == true)
                z = z + obj.RBearingSqrt * randn(size(z));
            end

            z = atan2(sin(z), cos(z));

            if (nargout == 3)
                d = dx^2 + dy^2;
                Hx(1:1:l2s.dotbot.SystemModel.NP) = [dy/d dx/d];
                R = obj.RBearing;
            end
        end

        function [z, Hx, Hm, R] = predictSLAMObservation(obj, x, mXY)

            z = mXY - x;

            if (obj.perturbWithNoise == true)
                z = z + obj.RSLAMSqrt * randn(size(z));
            end
            if (nargout == 4)
                Hx = -eye(2);
                Hm = eye(2);
                R = obj.RSLAM;
            end

        end

    end

    methods(Access = protected)

        function setupModels(obj)

            % Set up the continuous time process model
            obj.FXc=zeros(2);

            obj.QXc = eye(2) * obj.config.platform.controller.odomSigma^2;

            obj.BXc = [1 0;0 1];
                        
            if (isfield(obj.config, 'map') == false)
                return
            end
            
            if (isfield(obj.config.map, 'sensors') == false)
                return
            end

            % Set up the observation
            if (isfield(obj.config.map.sensors, 'gps'))
                obj.HXGPS = eye(2);
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