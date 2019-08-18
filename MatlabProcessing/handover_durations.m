%By: Sara Cooper
%Reads rosbag data about handover sections and plots object trajectory and
%velocity profiles

filename = '2019017block5';
bag = rosbag('patientriggers2019017block5block1.bag');

%Read rosbag data on topic /chatter, storing the ROS logs for events
bagselect1 = select(bag, 'Topic', '/chatter');
start = bag.StartTime;
a = bagselect1.MessageList;
bagSelection = select(bagselect1,'Time',[bag.StartTime,bag.StartTime + 6000]); 
msgs = readMessages(bagSelection);
count=size(msgs,1);

robot = 0;
in_ob = 0;
fin_ob = 0;
count_robot = 0;
human=0;
count_human = 0;

response = true;
%Calculate how many robot-to-humans handovers and human-to-robot handovers there have been
for n = 1:count
            value = char(msgs{n}.Data);
            if  ischar(value) && strcmp(value,'robot')
                 count_robot = count_robot + 1;
                 
            elseif  ischar(value) && strcmp(value,'human')
        
                  count_human = count_human + 1;
     
            end
    
end

robot_matrix = nan(count_robot, 18);
robot_matrix_human = nan(count_human, 13);

%Adjust the cue_type value based on the shape id of each participant. Some
%participants do not have behavioral response matrix, in such case response
%= false
if response == true
    T = readtable('TrialData-2019017-5.csv');
    T = table2array(T);
    cue_types = T(:,4);
    RT = T(:,6);
    cue_times = [];
    counter = 1;
    for i = 1:size(T,1)
        if cue_types(i) == 2
            robot_matrix(counter, 16) =RT(i);
            counter = counter+1;

        end

    end
    counter = 1;
    for i = 1:size(T,1)
         if cue_types(i) == 3
                        robot_matrix_human(counter, 13) =RT(i);
                        counter = counter+1;
         end
    end
end


count_robot = 0;

%For each robot-to-human hanover obtain the duration for each section based
%on logged text and rosbag timings
for n = 1:count
        value = char(msgs{n}.Data);
        if  ischar(value) && strcmp(value,'robot')
            robot = n;
            cue_time = a{robot, 'Time'};
            disp(value);
            disp(n);
            count_robot = count_robot + 1;
        elseif ischar(value) && strcmp(value,'human')
            robot = 0;
        end
        if  robot ~= 0 && ischar(value) && strcmp(value,'Starts moving')
            time_move = a{n, 'Time'};
            disp(n);
            robot_moving_time = time_move - cue_time; 
            robot_matrix(count_robot, 1) = robot_moving_time;
            time_grasp = a{n+3, 'Time'};
            robot_moving_grasp_time = time_grasp - time_move; 
            robot_matrix(count_robot, 2) =robot_moving_grasp_time;

        end  
        
        if  robot ~= 0 && ischar(value) && contains(value,'Error')
            error = [str2num(value((strfind(value, 'x:')+3):(strfind(value, 'y:')-2))),str2num(value((strfind(value, 'y:')+3):strfind(value,'z')-2)), str2num(value((strfind(value, 'z')+4):end-1))] ;
            robot_matrix(count_robot, 3) = error(1);
            robot_matrix(count_robot, 4) = error(2);
            robot_matrix(count_robot, 5) = error(3);
        end  
        
        if  robot ~= 0 && ischar(value) && contains(value,'Initial force')
            space_force = strfind(value, ' ');
            init_fx = str2num(value(space_force(2)+2:space_force(3)-1));
            init_fy = str2num(value(space_force(3)+1:space_force(4)-1));
            init_fz = str2num(value((space_force(4)+1):end-2));
            robot_matrix(count_robot, 6) = init_fx;
            
            
        end 
        
        if  robot ~= 0 && ischar(value) && contains(value,'Force after grasp')
            space_force = strfind(value, ' ');
            after_fx = str2num(value(space_force(3)+2:space_force(4)-1));
            after_fy = str2num(value(space_force(4)+1:space_force(5)-1));
            after_fz = str2num(value((space_force(5)+1):end-2));
            robot_matrix(count_robot,7) = after_fx;
            
        end  
        
        if  robot ~= 0 && ischar(value) && contains(value,'Object weight')
            weight = str2num(value(strfind(value, ':')+1:end-1));
            robot_matrix(count_robot, 8) = weight;
            
        end  

        if  robot ~= 0 && ischar(value) && contains(value,'threshold')
            threshold = str2num(value(strfind(value, ':')+2:end-1));
            robot_matrix(count_robot, 9) = threshold;
            
        end  

        

        if  robot ~= 0 && ischar(value) && strcmp(value,'Initiate object transfer') %or Force after grasp?
            time_raise = a{n, 'Time'};
            robot_raise_time = time_raise - time_grasp; 
            robot_matrix(count_robot, 10) =robot_raise_time;
            combine_time = robot_moving_grasp_time + robot_raise_time;
            robot_matrix(count_robot, 11) =combine_time;

        end
        if robot ~= 0 && ischar(value) && strcmp(value,'Human starts taking object') 
            transfer_time = a{n, 'Time'};
            object_transfer_wait_time = transfer_time - time_raise; 
            robot_matrix(count_robot, 12) =object_transfer_wait_time;
        end   
        if robot ~= 0 && ischar(value) && contains(value,'Total object transfer duration') 
            transfer_time2 = a{n, 'Time'};
            object_transfer_time = str2num(value(strfind(value, ':')+5:end-1));
            robot_matrix(count_robot, 13) = object_transfer_time;
            
        end             
        if robot ~= 0 && ischar(value) && contains(value,'Stop') 
            retreat_time = a{n+1, 'Time'};
            object_stop_time = retreat_time - transfer_time2;
            robot_matrix(count_robot, 14) = object_stop_time;
            total_robot_duration = str2num(value(strfind(value, ':')+4:end-1));
            robot_matrix(count_robot, 15) = total_robot_duration;     
        end

end


%Plot object pulling behavior for each case
count_robot = 0;
for n = 1:count
    value = char(msgs{n}.Data);
    
    if  ischar(value) && strcmp(value,'robot')
        robot = n;
        cue_time = a{robot, 'Time'};
        count_robot = count_robot + 1;
       
    end
    if robot ~= 0 && ischar(value) && (strcmp(value, 'Initiate object transfer'))
            in_ob = n+1;
    end
    if in_ob ~= 0 && contains(value, 'Gripper opening')
        fin_ob = n-3; 
        
    end
    if fin_ob ~= 0
        allmsg = readMessages(bagSelection, in_ob:fin_ob);
        start = bagSelection.StartTime;
        finish = bagSelection.EndTime;
        filtertime = select(bagSelection, 'Time', [start finish]);
        messs = filtertime.MessageList;
        p = table2array(messs(:, {'Time'}));
        k = 1;
        indices = 1;
        while k <(size(allmsg,1))
            data = allmsg{k}.Data;
            disp(data);
            if (strcmp(string(data),"Human starts taking object"))
                k = k+1; 
                data = allmsg{k}.Data;
            end
            if ~((strcmp(string(data),"Gripper opening")))
                disp("yes");
                if contains(data, 'Fx')
                    data_fx = data((strfind(data,':')+1):end);
                    wristData(indices,1) = str2num(data_fx);
                    k = k+2;
                    data = allmsg{k}.Data;
                    data_fz = data((strfind(data,':')+1):end);
                    wristData(indices,2) = str2num(data_fz);
                    indices = indices +1;
                end
            else
                break
            end
            k = k+1;
        end
        wristTime = zeros(size(wristData,1), 1);
        for k = 1:(size(wristData,1))
                wristTime(k) = a{k,'Time'};
                
         
        end
        figure
        plot(wristTime, wristData(:,1)); 
        title("/wrist_ft torque x when human grasping");
        xlabel('Time');
        ylabel('f.x');
        figure
        plot(wristTime, wristData(:,2)); 

        title("/wrist_ft torque z when human grasping");
        xlabel('Time');
        ylabel('f.z');

        fin_ob = 0;
        in_ob = 0;
        robot = 0; 
     
        max_value = max(wristData(:,1));
        max_value_fz = max(wristData(:,2));
        robot_matrix(count_robot, 17) =max_value;
        robot_matrix(count_robot, 18) = max_value_fz;

        

        
    end
   

end


%Save the output to file
header = {'Robot moving time', 'Robot grasping time', 'ErrorX', 'ErrorY', 'ErrorZ', 'InitFx', 'FinFx', 'Weight', 'Threshold', 'Raise time', 'Offer time', 'Transfer wait time', 'Transfer time', 'Stop time', 'Total robot duration', 'RT', 'Max f.x', 'Max f.z'};
output = [header; num2cell(robot_matrix)];
preprocessed=xlswrite(filename, output, 1);  
robot_matrix_average = nan(3, 18); 
robot_matrix_average(1,1) = nanmean(robot_matrix(:,1)); 
robot_matrix_average(1,2) = nanmean(robot_matrix(:,2));
robot_matrix_average(1,3) = nanmean(robot_matrix(:,3));
robot_matrix_average(1,4) = nanmean(robot_matrix(:,4));
robot_matrix_average(1,5) = nanmean(robot_matrix(:,5));
robot_matrix_average(1,6) = nanmean(robot_matrix(:,6));
robot_matrix_average(1,7) = nanmean(robot_matrix(:,7));
robot_matrix_average(1,8) = nanmean(robot_matrix(:,8));
robot_matrix_average(1,9) = nanmean(robot_matrix(:,9));
robot_matrix_average(1,10) = nanmean(robot_matrix(:,10));
robot_matrix_average(1,11) = nanmean(robot_matrix(:,11));
robot_matrix_average(1,12) = nanmean(robot_matrix(:,12));
robot_matrix_average(1,13) = nanmean(robot_matrix(:,13));
robot_matrix_average(1,14) = nanmean(robot_matrix(:,14));
robot_matrix_average(1,15) = nanmean(robot_matrix(:,15));
robot_matrix_average(1,16) = nanmean(robot_matrix(:,16));
robot_matrix_average(1,17) = nanmean(robot_matrix(:,17));
robot_matrix_average(1,18) = nanmean(robot_matrix(:,18));


robot_matrix_average(2,1) = min(robot_matrix(:,1));
robot_matrix_average(2,2) =  min(robot_matrix(:,2));
robot_matrix_average(2,3) =  min(robot_matrix(:,3));
robot_matrix_average(2,4) =  min(robot_matrix(:,4));
robot_matrix_average(2,5) =  min(robot_matrix(:,5));
robot_matrix_average(2,6) =  min(robot_matrix(:,6));
robot_matrix_average(2,7) =  min(robot_matrix(:,7));
robot_matrix_average(2,8) =  min(robot_matrix(:,8));
robot_matrix_average(2,9) =  min(robot_matrix(:,9));
robot_matrix_average(2,10) =  min(robot_matrix(:,10));
robot_matrix_average(2,11) =  min(robot_matrix(:,11));
robot_matrix_average(2,12) =  min(robot_matrix(:,12));
robot_matrix_average(2,13) =  min(robot_matrix(:,13));
robot_matrix_average(2,14) =  min(robot_matrix(:,14));
robot_matrix_average(2,15) =  min(robot_matrix(:,15));
robot_matrix_average(2,16) =  min(robot_matrix(:,16));
robot_matrix_average(2,17) =  min(robot_matrix(:,17));
robot_matrix_average(2,18) =  min(robot_matrix(:,18));


robot_matrix_average(3,1) = max(robot_matrix(:,1));
robot_matrix_average(3,2) = max(robot_matrix(:,2));
robot_matrix_average(3,3) = max(robot_matrix(:,3));
robot_matrix_average(3,4) = max(robot_matrix(:,4));
robot_matrix_average(3,5) = max(robot_matrix(:,5));
robot_matrix_average(3,6) = max(robot_matrix(:,6));
robot_matrix_average(3,7) = max(robot_matrix(:,7));
robot_matrix_average(3,8) = max(robot_matrix(:,8));
robot_matrix_average(3,9) = max(robot_matrix(:,9));
robot_matrix_average(3,10) = max(robot_matrix(:,10));

robot_matrix_average(3,11) = max(robot_matrix(:,11));
robot_matrix_average(3,12) = max(robot_matrix(:,12));
robot_matrix_average(3,13) = max(robot_matrix(:,13));
robot_matrix_average(3,14) = max(robot_matrix(:,14));
robot_matrix_average(3,15) = max(robot_matrix(:,15));
robot_matrix_average(3,16) = max(robot_matrix(:,16));
robot_matrix_average(3,17) = max(robot_matrix(:,17));
robot_matrix_average(3,18) = max(robot_matrix(:,18));

header = {'Robot moving time', 'Robot grasping time', 'ErrorX', 'ErrorY', 'ErrorZ', 'InitFx', 'FinFx', 'Weight', 'Threshold','Raise time', 'Offer time', 'Transfer wait time', 'Transfer time', 'Stop time', 'Total robot duration', 'RT', 'Max f.x', 'Max f.z'};

output = [header; num2cell(robot_matrix_average)];

%Save the averages, max and minimum for each handover stage
preprocessed=xlswrite(filename, output, 2);  


%Human to robot handover duration processing

init_ob_move = 0;
init_object_move = 0;
first_object_move = 0;

count_human = 0;

for n = 1:count
        value = char(msgs{n}.Data);
      
        if  ischar(value) && strcmp(value,'human')
        human = n;
        cue_time = a{human, 'Time'};
        count_human = count_human + 1;
    
    
        end
        if  human ~= 0 && ischar(value) && contains(value, 'Human starts grasping')
            human_move = a{n, 'Time'};
        end
        if  human ~= 0 && ischar(value) && contains(value, 'Object moving at speed ')
            if first_object_move == 0
                first_object_move = n;
            end
            last_object_move = n;

        end   

        if human ~= 0 && ischar(value) && first_object_move~=0 && contains(value, 'Object raise time')
            
            move_array = [];
            position = zeros(n-first_object_move-3, 3);
            time_move = a{n, 'Time'};
            for i = 1:(n-first_object_move-3)
                value = char(msgs{first_object_move+i-1}.Data);
                object_pos = value(strfind(value,'position')+10:end-1);
                space = strfind(object_pos, ' ');
                move_array(i) = str2num(value((strfind(value, 'd')+2):end));
                if (move_array(i) > 0.01)
                    human_grab = a{first_object_move, 'Time'};
                    human_reach_time = human_grab - cue_time;
                    robot_matrix_human(count_human, 1) =human_reach_time;
                    
                end
               

            end
            disp("finish");
            max_speed = max(move_array); 
            object_height = msgs{n-1}.Data;
            index = strfind(object_height, ',');
            robot_matrix_human(count_human, 2) =max_speed;
            robot_matrix_human(count_human, 3) =str2num(object_height((index(2)+2):end-1));
            value = char(msgs{n-2}.Data);
            move_array(i+1) = str2num(value((strfind(value, ':')+1):end));
            raise_object_time = a{first_object_move+i+1, 'Time'};       
            raising_time = raise_object_time - human_grab;        
            robot_matrix_human(count_human, 4) =raising_time;
            start = bagSelection.StartTime;
            finish = bagSelection.EndTime;
            filtertime = select(bagSelection, 'Time', [start finish]);
            messs = filtertime.MessageList;
            p = table2array(messs(:, {'Time'}));

            object_moving = zeros(1,size(move_array,2));
            for k = 1:(size(move_array,2))
                        object_moving(k) = p(k);

            end
        
            plot(object_moving, move_array);
                
            title("Object speed (z)");
            xlabel('Time');
            ylabel('Velocity in z');

            init_object_move = 0;
            first_object_move = 0;
        end

        if human ~= 0 && ischar(value) && contains(value,'Reach goal ')
            robot_reach = a{n, 'Time'};
            robot_response_time = robot_reach - raise_object_time; 
            robot_matrix_human(count_human, 5) =robot_response_time;
            robot_position = [];
            robot_position = [str2num( value(strfind(value, 'x:')+3 : strfind(value, 'y:')-1)), str2num(value(strfind(value, 'y:')+3 : strfind(value, 'z:')-6)),str2num(value(strfind(value, 'z:')+3 : strfind(value, 'orientation')-6))] ; 
            marker_pos_v = msgs{n-1}.Data;
            index = strfind(marker_pos_v, ' ');
            marker_x = str2num(marker_pos_v((index(2)+2):index(3)-1));
            marker_y = str2num(marker_pos_v((index(3)+1):index(4)-1));
            marker_z = str2num(marker_pos_v((index(4)+1):strfind(marker_pos_v, ')')-1));
            error = [abs(robot_position(1)-marker_x), abs(robot_position(2)-marker_y), abs(robot_position(3)-marker_z)];
            robot_matrix_human(count_human, 6) =error(1);
            robot_matrix_human(count_human,7) = error(2);
            robot_matrix_human(count_human,8) = error(3);


        end   
        if  human ~= 0 && ischar(value) && contains(value,'Object transfer duration')
           transfer_time = a{n, 'Time'}; 
            object_transfer_time = str2num(value(strfind(value, ':')+5:end-1)); 
            robot_matrix_human(count_human, 9) =object_transfer_time;
            
        end             
        if human ~= 0 && ischar(value) && contains(value,'Duration to place object')
            place_time = a{n, 'Time'};
            index = strfind(value, ':');
            object_place_time = str2num(value(index(1)+7:end-1)); 
            robot_matrix_human(count_human, 10) =object_place_time;
        
        end
        
        if human ~= 0 && ischar(value) && contains(value,'Stop moving') 
            retreat_time = a{n, 'Time'};
            robot_stop_time = retreat_time - place_time; 
            robot_matrix_human(count_human, 11) =robot_stop_time;
            total_robot_duration = str2num(value(strfind(value, ':')+4:end-1));
            robot_matrix_human(count_human, 12) = total_robot_duration;
        
        end
  
    
end




%Save data

robot_matrix_average2 = nan(3, 13); 

robot_matrix_average2(1,1) = nanmean(robot_matrix_human(:,1)); 
robot_matrix_average2(1,2) = nanmean(robot_matrix_human(:,2));
robot_matrix_average2(1,3) = nanmean(robot_matrix_human(:,3));
robot_matrix_average2(1,4) = nanmean(robot_matrix_human(:,4));
robot_matrix_average2(1,5) = nanmean(robot_matrix_human(:,5));
robot_matrix_average2(1,6) = nanmean(robot_matrix_human(:,6));
robot_matrix_average2(1,7) = nanmean(robot_matrix_human(:,7));
robot_matrix_average2(1,8) = nanmean(robot_matrix_human(:,8));

robot_matrix_average2(1,9) = nanmean(robot_matrix_human(:,9));
robot_matrix_average2(1,10) = nanmean(robot_matrix_human(:,10));
robot_matrix_average2(1,11) = nanmean(robot_matrix_human(:,11));
robot_matrix_average2(1,12) = nanmean(robot_matrix_human(:,12));
robot_matrix_average2(1,13) = nanmean(robot_matrix_human(:,13));


robot_matrix_average2(2,1) = min(robot_matrix_human(:,1));
robot_matrix_average2(2,2) =  min(robot_matrix_human(:,2));
robot_matrix_average2(2,3) =  min(robot_matrix_human(:,3));
robot_matrix_average2(2,4) =  min(robot_matrix_human(:,4));
robot_matrix_average2(2,5) =  min(robot_matrix_human(:,5));
robot_matrix_average2(2,6) =  min(robot_matrix_human(:,6));
robot_matrix_average2(2,7) =  min(robot_matrix_human(:,7));
robot_matrix_average2(2,8) =  min(robot_matrix_human(:,8));
robot_matrix_average2(2,9) =  min(robot_matrix_human(:,9));
robot_matrix_average2(2,10) =  min(robot_matrix_human(:,10));

robot_matrix_average2(2,11) =  min(robot_matrix_human(:,11));
robot_matrix_average2(2,12) =  min(robot_matrix_human(:,12));
robot_matrix_average2(2,13) =  min(robot_matrix_human(:,13));


robot_matrix_average2(3,1) = max(robot_matrix_human(:,1));
robot_matrix_average2(3,2) = max(robot_matrix_human(:,2));
robot_matrix_average2(3,3) = max(robot_matrix_human(:,3));
robot_matrix_average2(3,4) = max(robot_matrix_human(:,4));
robot_matrix_average2(3,5) = max(robot_matrix_human(:,5));
robot_matrix_average2(3,6) = max(robot_matrix_human(:,6));
robot_matrix_average2(3,7) = max(robot_matrix_human(:,7));
robot_matrix_average2(3,8) = max(robot_matrix_human(:,8));
robot_matrix_average2(3,9) = max(robot_matrix_human(:,9));
robot_matrix_average2(3,10) = max(robot_matrix_human(:,10));
robot_matrix_average2(3,11) = max(robot_matrix_human(:,11));
robot_matrix_average2(3,12) = max(robot_matrix_human(:,12));
robot_matrix_average2(3,13) = max(robot_matrix_human(:,13));


header = {'Human start', 'Max velocity', 'Max height', 'Raise time', 'Robot reach time', 'ErrorX', 'ErrorY', 'ErrorZ', 'Transfer time', 'Place time', 'Stop time', 'Total robot duration', 'RT'};
output = [header; num2cell(robot_matrix_human)];
preprocessed=xlswrite(filename, output, 3);  
header = {'Human start', 'Max velocity', 'Max height', 'Raise time', 'Robot reach time','ErrorX', 'ErrorY', 'ErrorZ', 'Transfer time', 'Place time', 'Stop time', 'Total robot duration', 'RT'};
output = [header; num2cell(robot_matrix_average2)];
preprocessed=xlswrite(filename, output, 4); 







