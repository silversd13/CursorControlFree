function KF = FitKF(Params,datadir,fitFlag,KF,TrialBatch,dimRedFunc)
% function KF = FitKF(Params,datadir,fitFlag,KF,TrialBatch)
% Uses all trials in given data directory to initialize matrices for kalman
% filter. Returns KF structure containing matrices: A,W,P,C,Q
% 
% datadir - directory containing trials to fit data on
% fitFlag - 0-fit on actual state,
%           1-fit on intended kinematics (refit algorithm)
%           2-fit on intended kinematics (smoothbatch algorithm)
% KF - kalman filter structure containing matrices: A,W,P,C,Q
% TrialBatch - cell array of filenames w/ trials to use in smooth batch
% dimRedFunc - function handle for dimensionality red. redX = dimRedFunc(X)

% Initialization of KF
if ~exist('KF','var'),
    KF = Params.KF;
end

% If Initialization Mode = 3, manually choose datadir & fit KF
if KF.InitializationMode==3 && fitFlag==0,
    datadir = uigetdir(datadir);
end

% ouput to screen
fprintf('\n\nFitting Kalman Filter:\n')
switch fitFlag,
    case 0,
        fprintf('  Initial Fit\n')
        switch KF.InitializationMode,
            case {1,2,3},
                fprintf('  Data in %s\n', datadir)
            case 4,
                fprintf('  Using most recent KF.\n')
        end
    case 1,
        fprintf('  ReFit\n')
        fprintf('  Data in %s\n', datadir)
    case 2,
        fprintf('  Smooth Batch\n')
        fprintf('  Data in %s\n', datadir)
        fprintf('  Trials: {%s-%s}\n', TrialBatch{1},TrialBatch{end})
end

% If Initialization Mode = 4, manually choose trial and load kf params, do
% not fit
if KF.InitializationMode==4 && fitFlag==0,
    f=load(fullfile(Params.ProjectDir,'TaskCode','persistence','kf_params.mat'));
    KF.R = f.KF.R;
    KF.S = f.KF.S;
    KF.T = f.KF.T;
    KF.ESS = f.KF.ESS;
    KF.C = f.KF.C;
    KF.Q = f.KF.Q;
    KF.Tinv = f.KF.Tinv;
    KF.Qinv = f.KF.Qinv;
    return
end

% grab data trial data
datafiles = dir(fullfile(datadir,'Data*.mat'));

Tfull = [];
Xfull = [];
Y = [];
T = [];
for i=1:length(datafiles),
    % load data
    load(fullfile(datadir,datafiles(i).name)) %#ok<LOAD>
    % ignore inter-trial interval data
    if strcmp(TrialData.Events(1).Str, 'Inter Trial Interval'),
        tidx = TrialData.Time >= TrialData.Events(2).Time;
    else,
        tidx = TrialData.Time >= TrialData.Events(1).Time;
    end
    % grab cursor pos and time
    Tfull = cat(2,Tfull,TrialData.Time(tidx));
    if fitFlag==0, % fit on true kinematics
        Xfull = cat(2,Xfull,TrialData.CursorState(:,tidx));
    else, % refit on intended kinematics
        Xfull = cat(2,Xfull,TrialData.IntendedCursorState(:,tidx));
    end
    T = cat(2,T,TrialData.NeuralTime(tidx));
    Y = cat(2,Y,TrialData.NeuralFeatures{tidx});
end

% interpolate to get cursor pos and vel at neural times
if size(Xfull,2)>size(Y,2)
    X = interp1(Tfull',Xfull',T')';
else,
    X = Xfull;
end

% if DimRed is on, reduce dimensionality of neural features
if exist('dimRedFunc','var'),
    Y = dimRedFunc(Y);
end

% full cursor state at neural times
D = size(X,2);

% if initialization mode returns shuffled weights
if fitFlag==0 && KF.InitializationMode==2, % return shuffled weights
    fprintf('  *Shuffled Weights\n')
    idx = randperm(size(Y,2));
    Y = Y(:,idx);
end

% fit kalman matrices
if KF.VelKF, % only use vel to fit C, set pos terms to 0
    C = (Y*X(2:end,:)') / (X(2:end,:)*X(2:end,:)');
    C = [zeros(size(C,1),1),C];
else,
    C = (Y*X') / (X*X');
end
Q = (1/D) * ((Y-C*X) * (Y-C*X)');

% update kalman matrices
switch fitFlag,
    case {0,1},
        % fit sufficient stats
        if KF.VelKF, % only use vel to fit C, set pos terms to 0
            X = X(2:end,:);
        end
        KF.R = X*X';
        KF.S = Y*X';
        KF.T = Y*Y';
        KF.ESS = D;
        KF.C = C;
        KF.Q = Q;
        KF.Tinv = inv(KF.T);
        KF.Qinv = inv(Q);
end

end % FitKF
