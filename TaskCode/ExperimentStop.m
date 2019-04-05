function ExperimentStop(fromPause)
if ~exist('fromPause', 'var'), fromPause = 0; end

% Close Screen
Screen('CloseAll');

% Close Serial Port
fclose('all');

% quit
fprintf('Ending Experiment\n')
if fromPause, keyboard; end

end % ExperimentStop
