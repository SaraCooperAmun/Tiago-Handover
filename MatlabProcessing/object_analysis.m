%By: Sara Cooper
%Analyse and plot object trajectory in human-to-robot handover from rosbag
%data
bag = rosbag('aruco_marker_data50.bag');
bagselect1 = select(bag, 'Topic', '/object_pos');
ts = timeseries(bagselect1);
my = readMessages(bagselect1);
a = bagselect1.MessageList;
alldata = ts.Data;
figure
height_data = ts.Data(:,3);
wristTime = zeros(size(height_data,1), 1);
speed = [];
for k = 1:(size(height_data,1))
                
                if wristTime(1,1) ~= 0
                    speed(k,1) = abs((height_data(k) - height_data(k-1))/(wristTime(k) - wristTime(k-1)));
                end
                wristTime(k) = a{k,'Time'};
                
         
end
subplot(2,1,1);
plot(wristTime,height_data);
title("Object trajectory in z");
xlabel('Time');
ylabel('z');

subplot(2,1,2);
plot(wristTime,speed);
title("Object velocity in z");
xlabel('Time');
ylabel('Speed in z');


