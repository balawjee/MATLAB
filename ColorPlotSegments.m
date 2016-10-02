%generate function
t1=-6.28:0.001:6.28;
t2=6.28:0.001:15;
y1=sin(t1) + 0.5*sin(2*t1) + 1.1*cos(0.5*t1) + 15*sin(6*t1);
y2=cos(5*t2) + 3.5*sin(20*t2) + 20*cos(3.5*t2) + 5*sin(0.6*t2);
data=[y1 y2];
t=[t1 t2];
%mix gaussian white noise
data=awgn(data,0.3);
plot(t,data);

%define segments in data
rs=1:250:length(data);

display_plot=1; %flag to indicate whether the plot should be displayed or not
display_segments=0; %flag to indicate whether the segments should be separated by black vertical lines

%to plot data - each segment has a different color
if display_plot==1   
    cc=hsv(length(rs));

    plot(data(1:rs(1)),'color',cc(1,:));
    hold on;
    for h=1:(length(rs)-1)
        plot(rs(h):rs(h+1),data(rs(h):rs(h+1)),'color',cc(h+1,:));hold on;            
        if display_segments==1
            ylim = get(gca,'YLim');           
            plot([rs(h) rs(h)],ylim,'color','k');            
        end
    end
    %to plot the last segment
    if display_segments==1
        ylim = get(gca,'YLim');        
        plot([rs(end) rs(end)],ylim,'color','k');        
    end
    plot(rs(h+1):length(data),data(rs(h+1):end),'color',cc(h+1,:));
    hold off;
end

