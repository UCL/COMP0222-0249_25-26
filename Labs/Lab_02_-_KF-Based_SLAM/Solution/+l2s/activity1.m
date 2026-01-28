% This task runs the simulator for the SLAM system

import ebe.core.*;
import ebe.graphics.*;
import l2s.*;

% Find, load and parse the configuration file
config = ebe.utils.readJSONFile('config/activity1.json');

% Create the mainloop object, which manages everything
mainLoop = ebe.MainLoop(config);

% Create the simulator and register it
simulator = l2s.dotbot.Simulator(config);
mainLoop.setEventGenerator(simulator);

% Create the SLAM system and register it
slamSystem = dotbot.SLAMSystem(config);
mainLoop.addEstimator(slamSystem);

% Create the store for estimates
resultsAccumulator = ebe.slam.XPPlatformAccumulator();
mainLoop.addResultsAccumulator(resultsAccumulator);

% Set up the figure in which we draw everything
fig = FigureManager.getFigure("Scenario Output");
clf
hold on
axis([-30 30 -30 30])
axis square

% Set up the views which show the output of the simulator
simulatorViewer = ebe.graphics.ViewManager(config);
simulatorView = l2s.dotbot.SimulatorView(config, simulator);
simulatorView.setCentreAxesOnTruth(true);
simulatorViewer.addView(simulatorView);
simulatorViewer.addView(dotbot.SLAMSystemView(config, slamSystem));

% Register the viewer with the mainloop
mainLoop.addViewer(simulatorViewer);

% Run the main loop until it terminates
mainLoop.run();

THistory = resultsAccumulator.timeStore;
XTrueHistory = resultsAccumulator.xTrueStore;

% Plot out state information
ebe.graphics.FigureManager.getFigure('Estimate Results');
clf

stateLabels = {'$x$','$y$'};

plot(THistory, XTrueHistory, 'LineWidth', 2);
xlabel('Time')
ylabel('State value')
legend(stateLabels, 'Interpreter', 'latex')

% Plot out covariance information
ebe.graphics.FigureManager.getFigure('Covariance Results');
clf

PEstStore = resultsAccumulator.PEstStore;

stateLabels = {'$x$','$y$'};

plot(THistory, PEstStore{1}, 'LineWidth', 2);
xlabel('Time')
ylabel('State value')
legend(stateLabels, 'Interpreter', 'latex')