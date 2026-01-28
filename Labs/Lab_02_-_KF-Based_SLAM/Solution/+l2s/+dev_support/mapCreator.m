function map = mapCreator(configFile, fileName)

% This function randomly populates a  map
% The specification consists of a set of boxes.
% Landmarks are uniformly sampled inside each box

% Load the configuration
config = ebe.utils.readJSONFile(configFile);

numRegions = numel(config.landmarkRegions)

landmarks = zeros(2, 0);

for m = 1 : numRegions
    region = config.landmarkRegions(m);
    lms = [region.xMin;region.yMin] + ...
        [region.xMax-region.xMin;region.yMax-region.yMin] .* ...
        rand(2, region.numLandmarks);
    landmarks = cat(2, landmarks, lms);
end

map = struct();

map.landmarks.slam.configuration = 'specified';
map.landmarks.slam.landmarks = landmarks';

sensors = struct();
sensors.enabled = true;
sensors.sigmaR = 1;
sensors.detectionRange = 10;
sensors.measurementPeriod = 0.2;

map.sensors.slam = sensors;

if (nargin == 2)
    % Convert the structure to a JSON string
    jsonString = jsonencode(map, 'PrettyPrint', true);  % Pretty formatting
    fileID = fopen(fileName, 'w');
    fprintf(fileID, '%s', jsonString);
    fclose(fileID);
end

