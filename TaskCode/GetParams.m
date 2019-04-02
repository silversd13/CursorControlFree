function Params = GetParams(Params)
% Experimental Parameters
% These parameters are meant to be changed as necessary (day-to-day,
% subject-to-subject, experiment-to-experiment)
% The parameters are all saved in 'Params.mat' for each experiment

%% Verbosity
Params.Verbose = true;

%% Experiment
Params.Task = 'CursorControlFree';
switch Params.ControlMode,
    case 1, Params.ControlModeStr = 'MousePosition';
    case 2, Params.ControlModeStr = 'MouseVelocity';
    case 3, Params.ControlModeStr = 'KalmanPosVel';
    case 4, Params.ControlModeStr = 'KalmanVelocity';
end

%% Control
Params.Gain             = 1;
Params.InitializationMode = 4; % 1-imagined mvmts, 2-shuffled imagined mvmts, 3-choose dir, 4-most recent KF
Params.MvmtAxisAngle    = -45;

%% Current Date and Time
% get today's date
now = datetime;
Params.YYYYMMDD = sprintf('%i',yyyymmdd(now));
Params.HHMMSS = sprintf('%02i%02i%02i',now.Hour,now.Minute,round(now.Second));

%% Data Saving

% if Subject is 'Test' or 'test' then can write over previous test
if strcmpi(Params.Subject,'Test'),
    Params.YYYYMMDD = 'YYYYMMDD';
    Params.HHMMSS = 'HHMMSS';
end

if IsWin,
    projectdir = 'C:\Users\ganguly-lab2\Documents\MATLAB\CursorControlFree';
elseif IsOSX,
    projectdir = '/Users/daniel/Projects/CursorControlFree/';
else,
    projectdir = '~/Projects/CursorControlFree/';
    butter(1,[.1,.5]);
end
addpath(genpath(fullfile(projectdir,'TaskCode')));

% create folders for saving
Params.ProjectDir = projectdir;
datadir = fullfile(projectdir,'Data',Params.Subject,Params.YYYYMMDD,Params.HHMMSS);
Params.Datadir = datadir;
mkdir(datadir);

%% Sync to Blackrock
Params.SerialSync = false;
Params.SyncDev = '/dev/ttyS1';
Params.BaudRate = 115200;

Params.ArduinoSync = true;

%% Timing
Params.ScreenRefreshRate = 10; % Hz
Params.UpdateRate = 10; % Hz

%% Cursor
Params.CursorColor = [0,102,255];
Params.CursorSize = 15;
Params.CursorRect = [-Params.CursorSize -Params.CursorSize ...
    +Params.CursorSize +Params.CursorSize];

%% Kalman Filter Properties
dt = 1/Params.UpdateRate;
if Params.ControlMode>=3,
    Params.KF.A = [...
        1       dt      0;
        0       .8      0;
        0       0       1];
    Params.KF.W = [...
        0       0       0;
        0       500     0;
        0       0       0];
    Params.KF.P = eye(3);
    Params.KF.InitializationMode = Params.InitializationMode; % 1-imagined mvmts, 2-shuffled
    if Params.ControlMode==4, % set velocity kalman filter flag
        Params.KF.VelKF = true;
    else,
        Params.KF.VelKF = false;
    end
end

%% Velocity Command Online Feedback
Params.DrawVelCommand.Flag = true;
Params.DrawVelCommand.Rect = [-425,-425,-350,-350];

%% BlackRock Params
Params.GenNeuralFeaturesFlag = true;
Params.ZscoreRawFlag = true;
Params.UpdateChStatsFlag = false;
Params.ZscoreFeaturesFlag = true;
Params.UpdateFeatureStatsFlag = false;
Params.SaveProcessed = false;
Params.SaveRaw = true;

Params.DimRed.Flag = false;
Params.DimRed.InitMode = 2; % 1-use imagined mvmts, 2-choose dir
Params.DimRed.InitAdapt = true;
Params.DimRed.InitFixed = ~Params.DimRed.InitAdapt;
Params.DimRed.Method = 1; % 1-pca, 2-fa
Params.DimRed.AvgTrialsFlag = false; % 0-cat imagined mvmts, 1-avg imagined mvmts
Params.DimRed.NumDims = [];

Params.Fs = 1000;
Params.NumChannels = 128;
Params.BufferTime = 2; % secs longer for better phase estimation of low frqs
Params.BufferSamps = Params.BufferTime * Params.Fs;
Params.BadChannels = [];
RefModeStr = {'none','common_mean','common_median'};
Params.ReferenceMode = 2; % 0-no ref, 1-common mean, 2-common median
Params.ReferenceModeStr = RefModeStr{Params.ReferenceMode+1};

% filter bank - each element is a filter bank
% fpass - bandpass cutoff freqs
% feature - # of feature (can have multiple filters for a single feature
% eg., high gamma is composed of multiple freqs)
Params.FilterBank = [];
Params.FilterBank(end+1).fpass = [.5,4];    % delta
Params.FilterBank(end).buffer_flag = true;
Params.FilterBank(end).hilbert_flag = true;
Params.FilterBank(end).phase_flag = false;
Params.FilterBank(end).feature = 1;

Params.FilterBank(end+1).fpass = [4,8];     % theta
Params.FilterBank(end).buffer_flag = true;
Params.FilterBank(end).hilbert_flag = true;
Params.FilterBank(end).phase_flag = false;
Params.FilterBank(end).feature = 1;

Params.FilterBank(end+1).fpass = [8,13];    % alpha
Params.FilterBank(end).buffer_flag = true;
Params.FilterBank(end).hilbert_flag = true;
Params.FilterBank(end).phase_flag = false;
Params.FilterBank(end).feature = 1;

% Params.FilterBank(end+1).fpass = [13,19];   % beta1
% Params.FilterBank(end).buffer_flag = false;
% Params.FilterBank(end).hilbert_flag = false;
% Params.FilterBank(end).phase_flag = false;
% Params.FilterBank(end).feature = 2;

% Params.FilterBank(end+1).fpass = [19,30];   % beta2
% Params.FilterBank(end).buffer_flag = false;
% Params.FilterBank(end).hilbert_flag = false;
% Params.FilterBank(end).phase_flag = false;
% Params.FilterBank(end).feature = 2;

% Params.FilterBank(end+1).fpass = [30,36];   % low gamma1 
% Params.FilterBank(end).buffer_flag = false;
% Params.FilterBank(end).hilbert_flag = false;
% Params.FilterBank(end).phase_flag = false;
% Params.FilterBank(end).feature = 2;
% 
% Params.FilterBank(end+1).fpass = [36,42];   % low gamma2 
% Params.FilterBank(end).buffer_flag = false;
% Params.FilterBank(end).hilbert_flag = false;
% Params.FilterBank(end).phase_flag = false;
% Params.FilterBank(end).feature = 2;
% 
% Params.FilterBank(end+1).fpass = [42,50];   % low gamma3
% Params.FilterBank(end).buffer_flag = false;
% Params.FilterBank(end).hilbert_flag = false;
% Params.FilterBank(end).phase_flag = false;
% Params.FilterBank(end).feature = 2;

Params.FilterBank(end+1).fpass = [70,77];   % high gamma1
Params.FilterBank(end).buffer_flag = false;
Params.FilterBank(end).hilbert_flag = false;
Params.FilterBank(end).phase_flag = false;
Params.FilterBank(end).feature = 2;

Params.FilterBank(end+1).fpass = [77,85];   % high gamma2
Params.FilterBank(end).buffer_flag = false;
Params.FilterBank(end).hilbert_flag = false;
Params.FilterBank(end).phase_flag = false;
Params.FilterBank(end).feature = 2;

Params.FilterBank(end+1).fpass = [85,93];   % high gamma3
Params.FilterBank(end).buffer_flag = false;
Params.FilterBank(end).hilbert_flag = false;
Params.FilterBank(end).phase_flag = false;
Params.FilterBank(end).feature = 2;

Params.FilterBank(end+1).fpass = [93,102];  % high gamma4
Params.FilterBank(end).buffer_flag = false;
Params.FilterBank(end).hilbert_flag = false;
Params.FilterBank(end).phase_flag = false;
Params.FilterBank(end).feature = 2;

Params.FilterBank(end+1).fpass = [102,113]; % high gamma5
Params.FilterBank(end).buffer_flag = false;
Params.FilterBank(end).hilbert_flag = false;
Params.FilterBank(end).phase_flag = false;
Params.FilterBank(end).feature = 2;

Params.FilterBank(end+1).fpass = [113,124]; % high gamma6
Params.FilterBank(end).buffer_flag = false;
Params.FilterBank(end).hilbert_flag = false;
Params.FilterBank(end).phase_flag = false;
Params.FilterBank(end).feature = 2;

Params.FilterBank(end+1).fpass = [124,136]; % high gamma7
Params.FilterBank(end).buffer_flag = false;
Params.FilterBank(end).hilbert_flag = false;
Params.FilterBank(end).phase_flag = false;
Params.FilterBank(end).feature = 2;

Params.FilterBank(end+1).fpass = [136,150]; % high gamma8
Params.FilterBank(end).buffer_flag = false;
Params.FilterBank(end).hilbert_flag = false;
Params.FilterBank(end).phase_flag = false;
Params.FilterBank(end).feature = 2;

% compute filter coefficients
for i=1:length(Params.FilterBank),
    [b,a] = butter(3,Params.FilterBank(i).fpass/(Params.Fs/2));
    Params.FilterBank(i).b = b;
    Params.FilterBank(i).a = a;
end

% unique pwr feature + all phase features
Params.NumBuffer = sum([Params.FilterBank.buffer_flag]);
Params.NumHilbert = sum([Params.FilterBank.hilbert_flag]);
Params.NumPhase = sum([Params.FilterBank.phase_flag]);
Params.NumPower = length(unique([Params.FilterBank.feature]));
Params.NumFeatures = Params.NumPower + Params.NumPhase;

%% Save Parameters
save(fullfile(Params.Datadir,'Params.mat'),'Params');

end % GetParams

