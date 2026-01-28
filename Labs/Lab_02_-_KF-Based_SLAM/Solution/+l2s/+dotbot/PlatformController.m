classdef PlatformController < ebe.core.ConfigurableComponent

    methods(Access = public)

        function obj = PlatformController(config)
            obj = obj@ebe.core.ConfigurableComponent(config);
        end

        function start(obj)
        end

        function u = computeControlInputs(obj, x, currentTime)
            currentTime = rem(currentTime, 20);

            if (currentTime < 5)
                u = [0.25;0];
            elseif (currentTime < 10)
                u = [0;0.25];
            elseif(currentTime < 15)
                u = [-0.25;0];
            else
                u = [0;-0.25];
            end
        end

    end

end