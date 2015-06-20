%# load ecg: simulate noisy ECG
% Fs=500;
% x = repmat(ecg(Fs), 1, 8);
% x = x + randn(1,length(x)).*0.18;
x=acc_data;

%# plot noisy signal
figure
subplot(911), plot(x), set(gca, 'YLim', [-1 1], 'xtick',[])
title('noisy')

%# sgolay filter
frame = 15;
degree = 0;
y = sgolayfilt(x, degree, frame);
subplot(912), plot(y), set(gca, 'YLim', [-1 1], 'xtick',[])
title('sgolayfilt')

%# smooth
window = 30;
%#y = smooth(x, window, 'moving');
%#y = smooth(x, window/length(x), 'sgolay', 2);
y = smooth(x, window/length(x), 'rloess');
subplot(913), plot(y), set(gca, 'YLim', [-1 1], 'xtick',[])
title('smooth')

%# moving average filter
window = 15;
h = ones(window,1)/window;
y = filter(h, 1, x);
subplot(914), plot(y), set(gca, 'YLim', [-1 1], 'xtick',[])
title('moving average')

%# moving weighted window
window = 7;
h = gausswin(2*window+1)./window;
y = zeros(size(x));
for i=1:length(x)
    for j=-window:window;
    	if j>-i && j<(length(x)-i+1) 
    		%#y(i) = y(i) + x(i+j) * (1-(j/window)^2)/window;
    		y(i) = y(i) + x(i+j) * h(j+window+1);
    	end
    end
end
subplot(915), plot( y ), set(gca, 'YLim', [-1 1], 'xtick',[])
title('weighted window')

%# gaussian
window = 7;
h = normpdf( -window:window, 0, fix((2*window+1)/6) );
y = filter(h, 1, x);
subplot(916), plot( y ), set(gca, 'YLim', [-1 1], 'xtick',[])
title('gaussian')

%# median filter
window = 15;
y = medfilt1(x, window);
subplot(917), plot(y), set(gca, 'YLim', [-1 1], 'xtick',[])
title('median')

%# filter
order = 15;
h = fir1(order, 0.1, rectwin(order+1));
y = filter(h, 1, x);
subplot(918), plot( y ), set(gca, 'YLim', [-1 1], 'xtick',[])
title('fir1')

%# lowpass Butterworth filter
fNorm = 25 / (Fs/2);               %# normalized cutoff frequency
[b,a] = butter(10, fNorm, 'low');  %# 10th order filter
y = filtfilt(b, a, x);
subplot(919), plot(y), set(gca, 'YLim', [-1 1])
title('butterworth')