function [Neuro,KF,Params] = RunTask(Params,Neuro,KF)
% Explains the task to the subject, and serves as a reminder for pausing
% and quitting the experiment (w/o killing matlab or something)

global Cursor 
Cursor.ControlMode = Params.ControlMode;
Cursor.Assistance = 0;
Cursor.DeltaAssistance = 0;

% Fit Dimensionality Reduction Params & Kalman Filter
Neuro.DimRed.Flag = Params.DimRed.Flag; % reset for task
if Params.DimRed.Flag && Params.DimRed.InitAdapt,
    Neuro.DimRed.F = FitDimRed(...
        fullfile(Params.Datadir),Neuro.DimRed);
    if Cursor.ControlMode>3,
        KF = FitKF(Params,...
            fullfile(Params.Datadir),0,KF,[],Neuro.DimRed.F);
    else,
        KF = [];
    end
else, % no dim reduction
    if Cursor.ControlMode>3,
        KF = FitKF(Params,...
            fullfile(Params.Datadir),0,KF);
    else,
        KF = [];
    end
end

switch Params.ControlMode,
    case 1, % Mouse Position
        Instructions = [...
            '\n\nMouse Position Control\n\n'...
            '\nAt any time, you can press ''p'' to briefly pause the task.'...
            '\n\nPress the ''Space Bar'' to begin!' ];
    case 2, % Mouse Velocity
        Instructions = [...
            '\n\nMouse Velocity Control\n\n'...
            '\nAt any time, you can press ''p'' to briefly pause the task.'...
            '\n\nPress the ''Space Bar'' to begin!' ];
    case {3,4}, % Kalman Filter Decoder
        Instructions = [...
            '\n\nKalman Brain Control\n\n'...
            '\nAt any time, you can press ''p'' to briefly pause the task.'...
            '\n\nPress the ''Space Bar'' to begin!' ];
end
                
% output to screen
fprintf('\n\nFixed Control:\n')
fprintf('  Saving data to %s\n\n',fullfile(Params.Datadir,'BCI_Fixed'))

InstructionScreen(Params,Instructions);
mkdir(fullfile(Params.Datadir,'BCI_Fixed'));

% Trial Loop
[Neuro,KF,Params] = RunLoop(Params,Neuro,fullfile(Params.Datadir,'BCI_Fixed'),KF);


end % RunTask
