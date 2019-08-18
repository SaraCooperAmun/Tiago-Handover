%By: Sara Cooper
%Human hand position and velocity processing, acquired from Nuitrack and
%Astra Orbbec camera (stored in rosbag)

%Read rosbag data
bag = rosbag('hand_tracking_sample.bag');
bagselect1 = select(bag, 'Topic', '/handpos');
bagSelection = select(bagselect1,'Time',[bag.StartTime, bag.EndTime]);
ts = timeseries(bagSelection);
bagselect2 = select(bag, 'Topic', '/handvel');
bagSelection2 = select(bagselect2,'Time',[bag.StartTime, bag.EndTime]);

%Plot the hand position and velocity in x,y and z; where z indicaes hand
%moved up/down and y indicates hand moved forward/backwards

ts_Z = timeseries(bagSelection, 'Z');
ts_X = timeseries(bagSelection, 'X');
ts_Y = timeseries(bagSelection, 'Y');
ts_vel = timeseries(bagSelection2, 'X');
ts_vel_y = timeseries(bagSelection2, 'Y');
ts_vel_z = timeseries(bagSelection2, 'Z');
figure
subplot(3,1,1); 
plot(ts_X, 'LineWidth', 3)
title("Hand position in x");
xlabel('Time');
ylabel('X');

subplot(4,1,2); 
plot(ts_Y, 'LineWidth', 3)
title("Hand position in y");
xlabel('Time');
ylabel('Y');

subplot(4,1,3); 
plot(ts_Z, 'LineWidth', 3)
title("Hand position in z");
xlabel('Time');
ylabel('Z');

figure
subplot(3,1,1); 

plot(ts_vel, 'LineWidth', 3)
title("Velocity in x");
xlabel('Time');
ylabel('V_X');
subplot(3,1,2); 

plot(ts_vel, 'LineWidth', 3)
title("Velocity in y");
xlabel('Time');
ylabel('V_Y');
subplot(3,1,3); 

plot(ts_vel, 'LineWidth', 3)
title("Velocity in z");
xlabel('Time');
ylabel('V_Z');



%COMPUTE THRESHOLDS FOR VELOCITY

a = bagselect2.MessageList;

vel = ts_vel.Data;
vel_y = ts_vel_y.Data;
vel_z = ts_vel_z.Data;

%Velocity in x
[pks, locs] = findpeaks(vel,'MinPeakHeight',0.3);
figure
hold on
plot(array, vel);
plot(locs, vel(locs), 'rv','MarkerFaceColor','r');
grid on
xlabel('Samples')
ylabel('VelocityX')
title('Velocity in x')    

mean_vel_x = mean(pks); 
disp("average v x");
disp(mean_vel_x);
    
%Velocity in y
[pks, locs] = findpeaks(vel_y,'MinPeakHeight',0.3);
figure
hold on
plot(array, vel_y);
plot(locs, vel_y(locs), 'rv','MarkerFaceColor','r');
grid on
xlabel('Samples')
ylabel('VelocityY')
title('Velocity in y')    

mean_vel_y = mean(pks); 
disp("average v y");
disp(mean_vel_y);

%Velocity in z
[pks, locs] = findpeaks(vel_z,'MinPeakHeight',0.3);
figure
hold on
plot(array, vel_z);
plot(locs, vel_z(locs), 'rv','MarkerFaceColor','r');
grid on
xlabel('Samples')
ylabel('VelocityZ')
title('Velocity in z')    

mean_vel_z = mean(pks); 
disp("average v z");
disp(mean_vel_z);
    
    
%COMPUTE THRESHOLDS FOR HAND POSITION
 
%X position
    
array = [];
for k = 1:(size(x,1))
                array(k) = k;
                
         
end


[pks, locs] = findpeaks(x,'MinPeakHeight',1.2);
figure
hold on
plot(array, x);
plot(locs, x(locs), 'rv','MarkerFaceColor','r');
grid on
    
xlabel('Samples')
ylabel('X distance')
title('X distance')
mean__x = mean(pks);
    
%Y position
    
array = [];
for k = 1:(size(y,1))
    array(k) = k;
                
         
end


[pks, locs] = findpeaks(y,'MinPeakHeight',0.2);
figure
hold on
plot(array, y);
plot(locs, y(locs), 'rv','MarkerFaceColor','r');
grid on
    
xlabel('Samples')
ylabel('Y distance')
title('Y distance')
mean_y = mean(pks);
disp("Mean y");
disp(mean_y);

%Z position
array = [];
for k = 1:(size(z,1))
                array(k) = k;
                
         
end

[pks, locs] = findpeaks(z,'MinPeakHeight',-0.1);
figure
hold on
plot(array, z);
plot(locs, z(locs), 'rv','MarkerFaceColor','r');
grid on
   
xlabel('Samples')
ylabel('Z distance')
title('Z distance')
mean_z = mean(pks);
disp("Mean z");
disp(mean_z);


    
    