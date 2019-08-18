%By: Sara Cooper
%Preliminary code for OpenFace gaze analysis. Includes basic plotting of
%vertical gaze trajectory, used for future gaze detection

T = readtable('openface_sample_2019006.csv'); %CSV file, output acquired with OpenFace
T = table2array(T);
gaze_y = T(:,13); %Gaze_angle_y indicates vertical gaze behavior. The more positive the output the higher the person is looking
frames = T(:,1);
gaze_y_good = [0];
frame_good = [];
counter=1;
last = size(gaze_y);

%Save the data points where confidence threshold is higher than 0.9, as
%otherwise it means face was blurry (Tiago's head moving) or was not being
%detected
gaze_y_coord = [];
for k = 1:size(gaze_y)
    if T(k,4)>0.9
        gaze_y_good(counter) = gaze_y(k);
        frame_good(counter) = frames(k);
        gaze_y_coord(counter) = gaze_y(7);
        counter = counter+1;
        
    end
end
figure
plot(frame_good,gaze_y_coord);
title("Gaze_y");
xlabel('Samples');
ylabel('gaze_y');
