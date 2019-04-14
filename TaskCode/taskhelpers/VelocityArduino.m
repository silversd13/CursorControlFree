function VelocityArduino(ptr,vel)
% vel is 1D velocity signal ~(-100:+100)

% reset digital pins
if ~exist('vel','var'),
    vel = 0;
end

% bound velocity btw -100 and +100
vel = max([vel,-100]);
vel = min([vel,+100]);

% remap vel to values btw 0 and 10^9
vel = round((vel+100)/200*(4095));

% write to arduino via i2c bus
vel_int  = (typecast(uint16(vel), 'uint8'));
write(ptr,[vel_int(2) , vel_int(1)],'uint8')

end