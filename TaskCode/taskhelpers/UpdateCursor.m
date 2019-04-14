function KF = UpdateCursor(Params,Neuro,KF)
% UpdateCursor(Params,Neuro)
% Updates the state of the cursor using the method in Params.ControlMode
%   1 - position control
%   2 - velocity control
%   3 - kalman filter  velocity
%
% Cursor - global structure with state of cursor [px,py,vx,vy,1]
% TaskFlag - 0-imagined mvmts, 1-clda, 2-fixed decoder
% TargetPos - 1d- coordinates of target position. used to assist
%   cursor to target
% KF - kalman filter struct containing matrices A,W,P,C,Q

global Cursor

% find vx and vy using control scheme
switch Cursor.ControlMode,
    case 1, % Move to Mouse
        X0 = Cursor.State;
        [x,y] = GetMouse();
        dx = x-Params.Center(1);
        dy = y-Params.Center(2);
        MvmtAxisUvec = [cosd(Params.MvmtAxisAngle),sind(Params.MvmtAxisAngle)];
        
        p = dot([dx,dy], MvmtAxisUvec);
        v = (p - X0(1))*Params.UpdateRate;
        
        
        % update cursor
        Cursor.State(1) = p;
        Cursor.State(2) = v;
        
        % Update Intended Cursor State
        X = Cursor.State;
        Vcom = (X(1) - X0(1))*Params.UpdateRate; % effective velocity command
        Cursor.IntendedState = Cursor.State; % current true position
        
    case 2, % Use Mouse Position as a Velocity Input (Center-Joystick)
        X0 = Cursor.State;
        [x,y] = GetMouse();
        dx = x-Params.Center(1);
        dy = y-Params.Center(2);
        MvmtAxisUvec = [cosd(Params.MvmtAxisAngle),sind(Params.MvmtAxisAngle)];
        v = Params.Gain * dot([dx,dy],MvmtAxisUvec);
        
        % assisted velocity
        if Cursor.Assistance > 0,
            Vcom = v;
            Vass = Cursor.Assistance*Vopt + (1-Cursor.Assistance)*Vcom;
        else,
            Vass = v;
        end
        
        % update cursor state
        Cursor.State(1) = Cursor.State(1) + Vass/Params.UpdateRate;
        Cursor.State(2) = Vass;
        
        % Update Intended Cursor State
        X = Cursor.State;
        Vcom = (X(1) - X0(1))*Params.UpdateRate; % effective velocity command
        Cursor.IntendedState = Cursor.State; % current true position
        
    case {3,4}, % Kalman Filter Input
        X0 = Cursor.State; % initial state, useful for assistance
        
        X = X0;
        if Neuro.DimRed.Flag,
            Y = Neuro.NeuralFactors;
        else,
            Y = Neuro.NeuralFeatures;
        end
        A = KF.A;
        W = KF.W;
        P = KF.P;
        
        % Kalman Predict Step
        X = A*X;
        P = A*P*A' + W;
        
        % Kalman Update Step
        C = KF.C;
        Q = KF.Q; % faster since avoids updating Qinv online
        K = P*C'/(C*P*C' + Q);
        X = X + K*(Y - C*X);
        P = P - K*C*P;
        
        % Store Params
        Cursor.State = X; % do not update the 1
        KF.P = P;
        
        % assisted velocity
        Vcom = X(2); % effective velocity command
        if Cursor.Assistance > 0,
            Vass = Cursor.Assistance*Vopt + (1-Cursor.Assistance)*Vcom;
            if abs(Vass)>250, % fast
                Vass = 250 * Vass / abs(Vass);
            end
            
            % update cursor state
            Cursor.State(1) = X0(1) + Vass/Params.UpdateRate;
            Cursor.State(2) = Vass;
        end
        
end

% update effective velocity command for screen output
try,
    Cursor.Vcommand = Vcom;
catch,
    Cursor.Vcommand = 0;
end

% write to arduino to exo
VelocityArduino(Params.ArduinoVelPtr, Cursor.Vcommand);

% bound cursor position to size of screen
pos = Cursor.State(1);
bound = min([...
    (Params.ScreenRectangle(3)-10)-Params.Center(1),...
    (Params.ScreenRectangle(4)-10)-Params.Center(2)]);
pos = max([pos,-bound]); % p-left
pos = min([pos,+bound]); % p-right
Cursor.State(1) = pos;

end % UpdateCursor

