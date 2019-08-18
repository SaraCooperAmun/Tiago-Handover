%By: Sara Cooper
%Plotting of gripper behavior, indicating gripper joint opening/close
%proportioanlly to human pull force in robot-to-human handover

fid=fopen('gripperData1.txt');
tline = fgetl(fid);
tlines = cell(0,1);

while ischar(tline)
    tlines{end+1,1} = tline;
    tline = fgetl(fid);
end
mylines = tlines(8:end-2); 
joints = zeros(size(mylines,1),1);
dates = zeros(size(mylines,1),1);
fz = zeros(size(mylines,1),1);
for k = 1:size(mylines,1)
    a = char(mylines(k));
    joints(k,1) = str2num(a(strfind(a, 'Positions')+12:strfind(a, 'Fx')-2));
    fz(k,1) = str2num(a(strfind(a, 'Fz')+3:strfind(a, 'Fz')+6));
    dates(k,1) = datenum(a(strfind(a, '2019'):end-1), 'dd-mmm-yyyy HH:MM:SS');
    
end
figure
plot(joints,fz);
title("Gripper behavior");
xlabel('joints');
ylabel('Fz');


figure
plot(fz,joints);
title("Gripper behavior");
ylabel('joints');
xlabel('Fz');

figure
plot(dates,joints);
title("Gripper behavior");
ylabel('joints');
xlabel('date');

figure
plot(dates,fz);
title("Gripper behavior");
ylabel('fz');
xlabel('date');

figure
plot(dates, joints)
title("Gripper behavior");
ylabel('joints');
xlabel('date');
set(gca, 'XDir','reverse')
grid

