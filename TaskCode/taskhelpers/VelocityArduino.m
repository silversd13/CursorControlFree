function VelocityArduino(ptr,pins,vel)
% vel is 1D velocity signal ~(-100:+100)

% reset digital pins
if ~exist('vel','var'),
    vel = 0;
end

% bound velocity btw -100 and +100
vel = max([vel,-100]);
vel = min([vel,+100]);

% remap vel to values btw 0 and 10^9
x = (vel+100)/200*(2^9-1);
bin_str = dec2bin(x,9);

% set digital pins
for i=1:length(pins),
    pin = pins{i};
    bit = str2double(bin_str(i));
    writeDigitalPin(ptr, pin, bit);
end

end