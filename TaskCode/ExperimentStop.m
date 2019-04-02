function ExperimentStop(fromPause)
if ~exist('fromPause', 'var'), fromPause = 0; end

% Close Screen
Screen('CloseAll');

% Close Serial Port
fclose('all');

% set velocity on exo to 0
VelocityArduino(Params.ArduinoPtr,Params.VelArduinoPins,0);

% quit
fprintf('Ending Experiment\n')
if fromPause, keyboard; end

end % ExperimentStop
