classdef SLAMSystemView < ebe.graphics.EstimatorView

    properties(Access = protected)

        % Geometry for the platform
        platformStateDrawer;

        % Geometry for the landmarks
        landmarkEstimateDrawer;

    end

    methods(Access = public)

        function obj = SLAMSystemView(config, eventGenerator)
            obj@ebe.graphics.EstimatorView(config, eventGenerator);
        end
        
        function start(obj)

            % Drawer for the platform state
            platformColour = ebe.graphics.DistinguishableColours.assignColour('SLAM Ground Truth');
            obj.platformStateDrawer = ebe.graphics.MeanCovarianceDrawer(platformColour);

            % Drawer for the landmark states
            landmarkColour = platformColour;%ebe.graphics.DistinguishableColours.assignColour('SLAM Ground Truth');

            obj.landmarkEstimateDrawer = ebe.graphics.MultipleMeanCovarianceDrawer(landmarkColour);
        end

        function visualize(obj, events)

            [x,P] = obj.estimator.platformEstimate();

            obj.platformStateDrawer.update(x ,P);

            [xl, Pl] = obj.estimator.landmarkEstimates();

            obj.landmarkEstimateDrawer.update(xl, Pl);

        end


    end

end