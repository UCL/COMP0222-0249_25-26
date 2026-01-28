% This script shows how we can do the same using the MainLoop class, which
% bundles everything together

import ebe.core.*;
import ebe.graphics.*;
import l2s.*;

% Get the scenario version
scenarioSubVersion = input('Enter subversion (a,b,c,d):' , 's');


% Find, load and parse the configuration file
configFile = sprintf('config/activity5/activity5%s.json', scenarioSubVersion);
config = ebe.utils.readJSONFile(configFile);

% Create the mainloop object, which manages everything
mainLoop = ebe.MainLoop(config);

% Create the simulator and register it
simulator = dotbot.Simulator(config);
mainLoop.setEventGenerator(simulator);

% Create the SLAM system and register it
slamSystem = dotbot.SLAMSystem(config);
mainLoop.addEstimator(slamSystem);

% Create the store for estimates
resultsAccumulator = ebe.slam.XPPlatformAccumulator();
mainLoop.addResultsAccumulator(resultsAccumulator);

% Set up the figure in which we draw everything
fig = FigureManager.getFigure("Simulator Output");
clf
hold on
axis([-20 60 -35 35])
axis square

% Set up the views which show the output of the simulator
simulatorViewer = ebe.graphics.ViewManager(config);
simulatorViewer.addView(dotbot.SimulatorView(config, simulator));
simulatorViewer.addView(dotbot.SLAMSystemView(config, slamSystem));

% Register the viewer with the mainloop
mainLoop.addViewer(simulatorViewer);

mainLoop.setGraphicsUpdatePeriod(5);

% Run the main loop until it terminates
mainLoop.run();

% Plot out state information
ebe.graphics.FigureManager.getFigure('Results');
clf

stateLabels = {'$x$','$y$'};

TEstimator = resultsAccumulator.timeStore;
XTrueHistory = resultsAccumulator.xTrueStore;

for f = 1 : 2
    PX = resultsAccumulator.PEstStore{1};
    X = resultsAccumulator.xEstStore{1};
    subplot(2,1,f)
    sigmaBound = 2 * sqrt(PX(f, :));
    plot(TEstimator, -sigmaBound, 'r--', 'LineWidth', 2)
    hold on
    plot(TEstimator, sigmaBound, 'r--', 'LineWidth', 2)
    stateError = X(f, :) - XTrueHistory(f, :);
    plot(TEstimator, stateError, 'LineWidth', 2);

    % Work out the axes
    maxError = max(abs(stateError));
    bound = 1.1 * max(maxError, max(sigmaBound));
    axis([TEstimator(1) TEstimator(end) -bound bound])
    
    xlabel('Time (s)')
    ylabel('Position $(ms)$', 'Interpreter','latex')
    title(stateLabels{f}, 'Interpreter','latex')
end