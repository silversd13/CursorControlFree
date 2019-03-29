function [Data, Neuro, KF, Params] = RunTrial(Data,Params,Neuro,KF)
% Runs a trial, saves useful data along the way
% Each trial contains the following pieces
% 1) Inter-trial interval
% 2) Get the cursor to the start target (center)
% 3) Hold position during an instructed delay period
% 4) Get the cursor to the reach target (different on each trial)
% 5) Feedback

global Cursor

%% Set up trial

% Output to Command Line
fprintf('\nTrial: %i\n',Data.Trial)

% keep track of update times
dt_vec = [];
dT_vec = [];

% grab blackrock data and run through processing pipeline
if Params.BLACKROCK,
    Cursor.LastPredictTime = GetSecs;
    Cursor.LastUpdateTime = Cursor.LastPredictTime;
    Neuro = NeuroPipeline(Neuro);
end

%% Cursor Movements
tstart  = GetSecs;
Data.Events(end+1).Time = tstart;
Data.Events(end).Str  = 'Cursor Moving';
if Params.ArduinoSync, PulseArduino(Params.ArduinoPtr,Params.ArduinoPin,2); end

done = 0;
while ~done,
    % Update Time & Position
    tim = GetSecs;
    
    % for pausing and quitting expt
    if CheckPause,
        % add event to data structure
        Data.Events(end+1).Time = GetSecs;
        Data.Events(end).Str  = 'Pause';
        if Params.ArduinoSync, PulseArduino(Params.ArduinoPtr,Params.ArduinoPin,3); end
        
        % Pause
        [Neuro,Params,done] = ExperimentPause(Params,Neuro);
        
        % add event to data structure
        Data.Events(end+1).Time = GetSecs;
        Data.Events(end).Str  = 'Unpause';
        if Params.ArduinoSync, PulseArduino(Params.ArduinoPtr,Params.ArduinoPin,3); end
    end
    
    % Update Screen
    if (tim-Cursor.LastPredictTime) > 1/Params.ScreenRefreshRate,
        % time
        dt = tim - Cursor.LastPredictTime;
        dt_vec(end+1) = dt;
        Cursor.LastPredictTime = tim;
        Data.Time(1,end+1) = tim;
        
        % grab and process neural data
        if ((tim-Cursor.LastUpdateTime)>1/Params.UpdateRate),
            dT = tim-Cursor.LastUpdateTime;
            dT_vec(end+1) = dT;
            Cursor.LastUpdateTime = tim;
            if Params.BLACKROCK,
                [Neuro,Data] = NeuroPipeline(Neuro,Data);
                Data.NeuralTime(1,end+1) = tim;
            end
            if Params.GenNeuralFeaturesFlag,
                Neuro.NeuralFeatures = VelToNeuralFeatures(Params);
                if Params.BLACKROCK, % override
                    Data.NeuralFeatures{end} = Neuro.NeuralFeatures;
                    Data.NeuralTime(1,end) = tim;
                else,
                    Data.NeuralFeatures{end+1} = Neuro.NeuralFeatures;
                    Data.NeuralTime(1,end+1) = tim;
                end
            end
            if Neuro.DimRed.Flag,
                Neuro.NeuralFactors = Neuro.DimRed.F(Neuro.NeuralFeatures);
                Data.NeuralFactors{end+1} = Neuro.NeuralFactors;
            end
            KF = UpdateCursor(Params,Neuro,KF);
        end
        
        % cursor
        CursorRect = Params.CursorRect;
        x = Cursor.State(1)*cosd(Params.MvmtAxisAngle);
        y = Cursor.State(1)*sind(Params.MvmtAxisAngle);
        CursorRect([1,3]) = CursorRect([1,3]) + x + Params.Center(1); % add x-pos
        CursorRect([2,4]) = CursorRect([2,4]) + y + Params.Center(2); % add y-pos
        Data.CursorState(:,end+1) = Cursor.State;
        
        % draw
        Screen('FillOval',Params.WPTR,Params.CursorColor,CursorRect)
        if Params.DrawVelCommand.Flag,
            VelRect = Params.DrawVelCommand.Rect;
            VelRect([1,3]) = VelRect([1,3]) + Params.Center(1);
            VelRect([2,4]) = VelRect([2,4]) + Params.Center(2);
            x0 = mean(VelRect([1,3]));
            y0 = mean(VelRect([2,4]));
            xf = x0 + 0.1*Cursor.Vcommand*cosd(Params.MvmtAxisAngle);
            yf = y0 + 0.1*Cursor.Vcommand*sind(Params.MvmtAxisAngle);
            Screen('FrameOval', Params.WPTR, [100,100,100], VelRect);
            Screen('DrawLine', Params.WPTR, [100,100,100], x0, y0, xf, yf, 3);
        end
        Screen('DrawingFinished', Params.WPTR);
        Screen('Flip', Params.WPTR);
        
    end
    
end % Reach Target Loop

% Done with trial
Data.Events(end+1).Time = tstart;
Data.Events(end).Str  = 'Cursor Stopped';
if Params.ArduinoSync, PulseArduino(Params.ArduinoPtr,Params.ArduinoPin,4); end


%% Completed Trial - Give Feedback
Screen('Flip', Params.WPTR);

% output update times
if Params.Verbose,
    fprintf('      Screen Update: Goal=%iHz, Actual=%.2fHz (+/-%.2fHz)\n',...
        Params.ScreenRefreshRate,mean(1./dt_vec),std(1./dt_vec))
    fprintf('      System Update: Goal=%iHz, Actual=%.2fHz (+/-%.2fHz)\n',...
        Params.UpdateRate,mean(1./dT_vec),std(1./dT_vec))
end

end % RunTrial



