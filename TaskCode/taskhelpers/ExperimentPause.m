function [Neuro,Params,done] = ExperimentPause(Params,Neuro)
% Display text then wait for subject to resume experiment

global Cursor
done = 0;

% Pause Screen
tex = 'Paused... Press ''p'' to continue, ''escape'' to quit, or ''d'' to debug';
DrawFormattedText(Params.WPTR, tex,'center','center',255);
Screen('Flip', Params.WPTR);

% set velocity on exo to 0
VelocityArduino(Params.ArduinoPtr,0);

KbCheck;
WaitSecs(.1);
while 1, % pause until subject presses p again or quits
    [~, ~, keyCode, ~] = KbCheck;
    if keyCode(KbName('p'))==1,
        break;
    end
    if keyCode(KbName('escape'))==1 || keyCode(KbName('q'))==1,
        done=1; % quit experiment
        break;
    end
    if keyCode(KbName('d'))==1,
        keyboard; % quit experiment
    end
    
    % grab and process neural data
    tim = GetSecs;
    if ((tim-Cursor.LastUpdateTime)>1/Params.UpdateRate),
        Cursor.LastUpdateTime = tim;
        Cursor.LastPredictTime = tim;
        if Params.BLACKROCK,
            Neuro = NeuroPipeline(Neuro);
        elseif Params.GenNeuralFeaturesFlag,
            Neuro.NeuralFeatures = VelToNeuralFeatures(Params);
        end
    end
end

Screen('Flip', Params.WPTR);
WaitSecs(.1);

end % ExperimentPause