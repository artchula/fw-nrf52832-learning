function [heart_rate] = processing()
%%clear data and close figures
clc
clearvars
hold off
close all

%% Read data with cvs file
raw_data = csvread('/Users/w.wiyarungmail.com/Documents/MATLAB/HeartRate/heart_rate_to_file/hr_1.txt');

%get red led value 
%red_data = raw_data(1:2000,1);
red_data = raw_data(:,1);
%get ir led value 
%ir_data = raw_data(1:2000,2);
%ir_data = raw_data(:,2);

%% Create sampling time
%400Hz Sampling Rate
L = size(red_data,1);
Fs = 100; 
T = 1/Fs;
%create timeing
t = (0:L-1)*T;

%% Plot raw data
figure(1);
hold on;
plot(t,red_data,'r')
%plot(t,ir_data,'g')
hold off;

%% DC Remove
DCRemover();

red_data_no_dc(1) = 0;
red_data_btw(1) = 0;

v(1) = 0;
v(2) = 0;
v(3) = 0;

for n=1:size(red_data,1)
    red_data_no_dc(n) = DCRemover(red_data(n));
    if n < 400
        red_data_no_dc(n) = 0;
    end
    
    [red_data_btw(n), v] = btw_filter(v,red_data_no_dc(n));

end


figure(2);
hold on;
plot(t,red_data_no_dc,'xg')

plot(t,red_data_btw,'xr')

%plot(t,ir_data,'g')
hold off;


%% Find local maximum
myCheckForBeat();

beat_detect_res(1) = 0;
beat_detect_index(1) = 0;
i = 1;
j = 1;
heart_rate(1) = 0;

for n=1:size(red_data,1)
    [beat_detect_res(n),pre] = myCheckForBeat(red_data_btw(n), n);
    
     if beat_detect_res(n) == 1
         beat_detect_index(i) = n-13;
         i = i+1;
     end
     
     if beat_detect_res(n) == 1 && pre < 2000
         heart_rate(j) = 60000/pre;
         j = j+1;
     end
end


%%

figure(3);
hold on;
plot(red_data_btw,'xr')
plot(beat_detect_index,red_data_btw(beat_detect_index),'og')

mean(heart_rate)

figure(4);
hold on;
plot(heart_rate,'xb')


wsigma = 1;
x_minus = 70; P_minus = 10^2; % Initial conditions 
%Phik = 0.95;
Phik = 1.0;
Qk = wsigma^2; Hk = 1; Rk = 80; 

x_plus(1) = 0;

for k = 1:size(heart_rate,2)
    % Estimate
    z = heart_rate(k);
    Kk = P_minus*Hk*inv(P_minus*(Hk^2)+Rk); 
    x_plus(k) = x_minus + Kk*(z-Hk*x_minus); 
    P_plus = ((1-Kk*Hk)^2)*P_minus+(Rk*Kk^2); % Predict
    x_minus = Phik*x_plus(k);
    P_minus = P_plus*Phik^2+Qk;
end
% Plot results
plot(x_plus,'og')

mean(x_plus)


end


%% find peak
function [beatDetected, beatPeriod] = myCheckForBeat(sample, index)
    
    BEATDETECTOR_INIT_HOLDOFF               = 2000;%   // in ms, how long to wait before counting
    BEATDETECTOR_MASKING_HOLDOFF            = 200;%     // in ms, non-retriggerable window after beat detection
    BEATDETECTOR_FIND_EAGE_COUNT            = 10;
    %increasing decreasing
    
    BEATDETECTOR_STATE_INIT = '1';
    BEATDETECTOR_STATE_FIND_INCREASING = '2';
    BEATDETECTOR_STATE_FIND_DECREASING = '3';
    BEATDETECTOR_STATE_MAYBE_DETECTED = '4';
    BEATDETECTOR_STATE_MASKING = '5';
    
    persistent state;
    persistent old_sample;
    persistent count;
    persistent tsNewBeat;
    persistent tsOldBeat;
   
    
    if nargin == 0
        state = BEATDETECTOR_STATE_INIT;
        old_sample = 0;
        count = 0;
        tsNewBeat = 0;
        tsOldBeat = 0;
        beatDetected = 0;
        beatPeriod = 0;
    else
        millis = index*10;
        beatDetected = 0;
        beatPeriod = 0;
        
        switch state
            case BEATDETECTOR_STATE_INIT
                if (millis > BEATDETECTOR_INIT_HOLDOFF)
                    state = BEATDETECTOR_STATE_FIND_INCREASING;
                end    
            case BEATDETECTOR_STATE_FIND_INCREASING   
                if old_sample < sample
                    count = count+1;
                elseif  old_sample > sample && count > BEATDETECTOR_FIND_EAGE_COUNT
                    count = 0;
                    state = BEATDETECTOR_STATE_FIND_DECREASING;
                    tsOldBeat = tsNewBeat;
                    tsNewBeat = millis; 
                else
                    count = 0;
                end
            case BEATDETECTOR_STATE_FIND_DECREASING    
                if old_sample > sample
                    count = count+1;
                else 
                    count = 0;
                    state = BEATDETECTOR_STATE_FIND_INCREASING;
                    
                end                
                if count > BEATDETECTOR_FIND_EAGE_COUNT
                    count = 0;
                    state = BEATDETECTOR_STATE_MAYBE_DETECTED; 
                end
            case BEATDETECTOR_STATE_MAYBE_DETECTED
                beatDetected = 1;
                beatPeriod = tsNewBeat-tsOldBeat;
                
                state = BEATDETECTOR_STATE_MASKING;
            case BEATDETECTOR_STATE_MASKING
                if (millis - tsNewBeat > BEATDETECTOR_MASKING_HOLDOFF)
                    state = BEATDETECTOR_STATE_FIND_INCREASING;
                end
        end
        old_sample = sample;
    end
    
    

end





