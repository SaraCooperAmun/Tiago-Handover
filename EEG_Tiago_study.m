%%% Created by Samuel Bennett at the University of Stirling (Psychtoolbox
%%% aspect) and Sara Cooper (Robotics aspect)
%%% Pilot script for a Joint Action Planning Study.
%%% This script uses a DAQ device to emulate Parallel port triggers, please
%%% use the Universal Library installation to allow for communi0000000000000000000000cation.
%%% To use, need 64 Bit PC, Psychtoolbox 3, and GStreamer Installed due to
%%% restrictions with Psychtoolbox-3, as well as Robotics System Toolbox
% Participant Data, must be saved as string.


%Edit for each block: participant ID, block number, and shape id,
%corresponding to the cue meaning sheet
participantID = '2019001'; 
block = 0; 
ros_stop = false;
shape_id = 0;
rosshutdown


% Main Study Design.
%--------------------------------------------------------------------------
%                                Setup
%---------------------------------------0000000000000000000000000000000-----------------------------------
% Here we call some default settings for setting up Psychtoolbox
PsychDefaultSetup(2);
KbName('UnifyKeyNames');

% Initialize access to the inpoutx64 low-level I/O driver
%config_io;
% Create an instance of the io64 object
ioObj = io64;

% Initialize the interface to the inpoutx64 system driver
status = io64(ioObj);

% Write a value to the default LPT1 printer output port (at 0x378)
address = hex2dec('CFF8');

% Get screen number and choose the slave monitor.
% Slave Monitor is the non main monitor, setup in Screen Resolution.
screens = Screen('Screens');
screenNumber = max(screens);
hz =  Screen('FrameRate', screenNumber);
hzInterval =  1000  / hz;


% Define black and white (white will be 1 and black 0). This is because
% luminace values are genrally defined between 0 and 1.
white = WhiteIndex(screenNumber);
black = BlackIndex(screenNumber);

% Make luminance Grey i.e. mid point between white and black.
grey = (white / 2);

% Open an on screen window and color it grey..
[window, windowRect] = PsychImaging('OpenWindow', screenNumber, grey);

% Get the size of the on screen window in pixels, these are the last two
% numbers in "windowRect" and "rect"
[screenXpixels, screenYpixels] = Screen('WindowSize', window);

% Get the centre coordinate of the window in pixels.
[xCenter, yCenter] = RectCenter(windowRect);

% Query color depth of the pixel in bits.
pixelDepth = Screen('PixelSize', window);

% Queries the display size in mm as reported by the operating system.
[width, height] = Screen('DisplaySize', screenNumber);

% Get the maximum coded luminance level (this should be 1)
maxLum = Screen('ColorRange', window);
nBlock = 1;
blockList = 1:nBlock; % Blocklist.
% Trial Number, condition, and block information setup. Specify number of
% trials per block
if strcmp(participantID, 'sara') %do not save the researcher's data
    nSet = 6;  
elseif block == 0 
    nSet = 7;
  
else
    nSet = 7;
   
end

%Specify meaning of each shape set, where [circle, square, diamond]
if (shape_id == 0)
           
            cueMeaning = ["robot", "human", "not moving"];
elseif (shape_id == 1)
            cueMeaning = ["not moving", "robot", "human"];
elseif (shape_id == 2)
            cueMeaning = ["human", "not moving", "robot"];
elseif (shape_id == 3)
            cueMeaning = [ "robot","not moving", "human"];
elseif (shape_id == 4)
            cueMeaning = ["not moving", "human", "robot"];
elseif (shape_id == 5)
            cueMeaning = ["human", "robot", "not moving"];

end

nConditions = 3;
lBlock = nConditions * nSet; 
numTrials = nBlock * lBlock;
nState = 0;
conditionOne = 0;
conditionTwo = 0;
conditionThree = 0;
roboTrigger = 0;

% Response Matrix cue.
respMatrix = nan(11, numTrials);
%respMatrix = nan(9, numTrials);

% Turn off the cursor.
HideCursor();

%Set up ROS connection and corresponding publishers/subscribers. EDIT IP of
%the robot accordingly
rosshutdown
setenv('ROS_IP', '10.68.0.131') 
setenv('ROS_MASTER_URI','http://10.68.0.1:11311')
rosinit
pub = rospublisher('/action','std_msgs/String'); %publishes the action to perform by the robot (robot, human)
sub = rossubscriber( '/trigger'); %reads whether the robot is moving or not moving

handover_result = "None";
state = "not moving";
msg = rosmessage(pub);
msg.Data = 'None'; 
send(pub,msg);
robotMove = false;
sub_stop = rossubscriber('/stop'); 
roboStart = 0;
roboDuration = 0;


%--------------------------------------------------------------------------
%                            Conversions
%--------------------------------------------------------------------------
% All distances in cm.
h = height/10;
w = width/10;
pixelSizeY = h/screenYpixels;
pixelSizeX = w/screenXpixels;
pixelSizeAverage = (pixelSizeY + pixelSizeX)/2;

%--------------------------------------------------------------------------
%                            Cue Information
%--------------------------------------------------------------------------
% Specify all cue characteristics that will not change between trials.
lineWidth = 10; % ###Edit Start###
cueRadius = 17; 
xEccentricy = xCenter / 2; % The eccentricty of the Cue objects from center.
x1 = xCenter + xEccentricy; % x value for poly 1.
x2 = xCenter - xEccentricy; % x value for poly 2. ###Edit End###
objectColour = [0 0 0];
objectSides = 4;
objectDeg = linspace(0, 360, objectSides + 1) + 45;
objectRad = objectDeg * (pi / 180);

%--------------------------------------------------------------------------
%                         Fixation Information
%--------------------------------------------------------------------------
% Setup the text type for the window
Screen('TextFont', window, 'Ariel');
Screen('TextSize', window, 36);

% Fixation length and width. ###Edit Start###
fixCrossDimPix = 17;
fixWidthPix = 10;

% Fixation cross, using DrawLines. 
fixXCoords1 = [-fixCrossDimPix fixCrossDimPix 0 0];
fixYCoords1 = [0 0 -fixCrossDimPix fixCrossDimPix];
fixXCoords2 = [-fixCrossDimPix fixCrossDimPix 0 0];
fixYCoords2 = [0 0 -fixCrossDimPix fixCrossDimPix];
fixationCoords1 = [fixXCoords1; fixYCoords1];
fixationCoords2 = [fixXCoords2; fixYCoords2];
%###Edit End###

%--------------------------------------------------------------------------
%                            GO Information
%--------------------------------------------------------------------------
% GO length and width.###Edit Start###
goRadius = 9;
goWidthPix = 10; % ###Edit End###

% X and Y co ordinates for the GO X object.
numSides = 4;
goDeg = linspace(0, 360, numSides + 1) + 45;
goRad = goDeg * (pi / 180);

% Calculate the X and Y co ordinates of the line.
yGOVector = sin(goRad) .* goRadius;
xGOVector = cos(goRad) .* goRadius;
goCoords = [xGOVector; yGOVector];
goCoords(:,end) = []; % Remove last columns as 360 degrees = 0 degrees.

% Need to switch column 2 and 3 so lines converge in middle.
go1 = goCoords(:,2);
go2 = goCoords(:,3);
goCoords(:,3) = go1;
goCoords(:,2) = go2;

%--------------------------------------------------------------------------
%                           Optional Responses
%--------------------------------------------------------------------------
% State the response keys that we are interested in.
escapeKeyID = KbName('ESCAPE');
responseKeyID = KbName('0');
KbQueueCreate(); % As using key release, may need to check numSlots to see if they need to be increased above default 10000.
KbQueueStart();

for nTrial = 1:numTrials
    %----------------------------------------------------------------------
    %                          Introduction
    %----------------------------------------------------------------------
    % Timing for start of study.
    tic;
    start_trial = GetSecs;
    % Introduction screen and prompt for button press to start.
    if nTrial == 1
        DrawFormattedText(window, 'Press 0 button to begin the test',...
           'center', 'center', black);
        Screen('Flip', window);
        KbStrokeWait;
    end
    %----------------------------------------------------------------------
    %                            Block Setup
    %----------------------------------------------------------------------
    % Block setup for the study.
    if mod(nTrial, lBlock) == 1
        pos = randi(length(blockList));
        trialCondition = blockList(pos);
        blockList = setdiff(blockList, trialCondition);
        block = block + 1;
       


        % Adds a pause between blocks.
        if nTrial > 1
        DrawFormattedText(window, 'The block is finished.- \n\n If you wish to have a break please take one now. \n\n Press Any Key To Continue',...
           'center', 'center', black);
        Screen('Flip', window);
        KbStrokeWait;
        end
    end
    
    %----------------------------------------------------------------------
    %                             Fixation
    %----------------------------------------------------------------------
    % Set up alpha-blending for smooth (anti-aliased) lines, essential.
    

   if nTrial > 1
              [ ~, firstPress, firstRelease, lastPress, lastRelease ] = KbQueueCheck();

    while (firstPress(responseKeyID) ==0)
        pause(0.2);
     
         [ ~, firstPress, firstRelease, lastPress, lastRelease ] = KbQueueCheck();
        
      

    end
    
   end
    
    Screen('BlendFunction', window, 'GL_SRC_ALPHA', 'GL_ONE_MINUS_SRC_ALPHA');

    % Draw the fixation cross.### Edit Start###
    Screen('DrawLines', window, fixationCoords1,...
    fixWidthPix, [0 0 0], [x1 yCenter], 2);
    Screen('DrawLines', window, fixationCoords2,...
    fixWidthPix, [0 0 0], [x2 yCenter], 2);

    % Flip to the screen to display the fixation cross. ###Edit End###
    Screen('Flip', window);
    
    % Fixation Duration, change (n) v000000000000000000000000alue for range, and add a value for minimum. 
    fixDuration = 2; % Written in ms, converted to seconds for pause(). 
    if nTrial == 1
       [ ~, firstPress, firstRelease, lastPress, lastRelease ] = KbQueueCheck();

    while (firstPress(responseKeyID) ==0)
        pause(0.2);
     
        [ ~, firstPress, firstRelease, lastPress, lastRelease ] = KbQueueCheck();
        
      

    end
    end
    
    
    
    pause (fixDuration);
    
    %----------------------------------------------------------------------
    %                             Cue
    %----------------------------------------------------------------------
    % This is the information for presenting the cue.
    % Randomly select the cue that is presented.
    cueState = ceil(rand * 3);
    
    % If we have already reached the set number, re roll cueState.
    while (cueState == 1 && conditionOne == nSet) || (cueState == 2 && conditionTwo == nSet) || (cueState == 3 && conditionThree == nSet)
        cueState = ceil(rand * 3);
    end
    
    % Setup the cue dependent on the cue state. Done by working out the co
    % ordinates of a line that connects at the end.
    if cueState == 1 % Circle-ish
        cueColour = [255 255 255];
        numSides = 15;
        polyDeg = linspace(0, 360, numSides + 1) + 30;
        polyRad = polyDeg * (pi / 180);
        conditionOne = conditionOne + 1;
       
    elseif cueState == 2 % Square
        cueColour = [255 255 255];
        numSides = 4;
        polyDeg = linspace(0, 360, numSides + 1) + 45;
        polyRad = polyDeg * (pi / 180);
        conditionTwo = conditionTwo + 1;
      
    elseif cueState == 3 % Diamond
        cueColour = [255 255 255];
        numSides = 4;
        polyDeg = linspace(0, 360, numSides + 1);
        polyRad = polyDeg * (pi / 180);
        conditionThree = conditionThree + 1;
       
    end
    robCond = cueMeaning(cueState);
    if robCond ~= "not moving"
        msg = rosmessage(pub);
        msg.Data = convertStringsToChars(robCond);
        send(pub, msg);
    else
        handover_result = "None";
        roboDuration = 0;
    end
    
    % Calculate the X and Y co ordinates of the line. 
    xPolyVector1 = cos(polyRad) .* cueRadius + x1;
    xPolyVector2 = cos(polyRad) .* cueRadius + x2;
    yPolyVector = sin(polyRad) .* cueRadius + yCenter;
    yObjectVector = sin(objectRad) .* cueRadius + yCenter;
    xObjectVector1 = cos(objectRad) .* cueRadius + x1;
    xObjectVector2 = cos(objectRad) .* cueRadius + x2;
    
    % Draw the rect to the screen
    Screen('FillPoly', window, cueColour, [xPolyVector1; yPolyVector]', lineWidth);
    Screen('FillPoly', window, cueColour, [xPolyVector2; yPolyVector]', lineWidth);
    Screen('Flip',window);
    % Send the cue trigger.
    io64(ioObj,address,cueState);
    pause(0.002);
    io64(ioObj,address,0);
    
    cueDuration = 0.2; % The cue duration, pause uses seconds, not ms. 
    pause(cueDuration)

   
    
    %----------------------------------------------------------------------
    %                          Grey Screen
    %----------------------------------------------------------------------
    % Flip to the screen
    Screen('Flip', window, grey);
    
    % Duration of 800ms, for random use ((rand * n) +  n) ./ 1000.
    ISIDuration = 1.3; %original 1.3
    pause(ISIDuration); % ###Edit End###
    
        %----------------------------------------------------------------------
    %                          Keyboard Reset
    %----------------------------------------------------------------------
    % Reset the event and button queue just as Go signal appears for RT.
    KbQueueFlush();
    KbEventFlush();
    
    %----------------------------------------------------------------------
    %                            Go Signal
    %----------------------------------------------------------------------  
    goStart = GetSecs;
    
    % The cue duration, pause uses seconds, not ms. 
    goDuration = 0.4;
    goMove = 0;
    goTrigger = 0;
    
    % Send the trigger.
    io64(ioObj,address,10); % Send a value of 10 for GO start.
    pause(0.002);
    io64(ioObj,address,0);
    
    % Setup as a while loop to allow for trigger to be sent at button press.
   % Draw the Go objects. ###Edit Start###
    Screen('DrawLines', window, fixationCoords1,...
    fixWidthPix, [0 1 0], [x1 yCenter], 2);
    Screen('DrawLines', window, fixationCoords2,...
    fixWidthPix, [0 1 0], [x2 yCenter], 2);
        
    % Flip to the screen 
    Screen('Flip', window);
 
    if robCond ~= "robot" 
    [ ~, ~, firstRelease, ~, lastRelease ] = KbQueueCheck();
    
    while goMove <= goDuration

    % Check duration of robo section.
    goMove = GetSecs - goStart;
    if goTrigger == 0
        [ ~, ~, firstRelease, ~, lastRelease ] = KbQueueCheck();
        if (firstRelease(responseKeyID) || lastRelease(responseKeyID) >= 1) && goTrigger == 0
            % Send the cue trigger.
            io64(ioObj,address,96); % No reason to use 32, except that it is the KeyCode for spacebar.
            pause(0.01);
            io64(ioObj,address,0);
            goTrigger = 1;
            goMove = GetSecs - goStart;
            if robCond ~= "not moving"
                msg = rosmessage(pub);
                msg.Data = convertStringsToChars("released");
                send(pub, msg);
            end
        elseif firstRelease(escapeKeyID) || lastRelease(escapeKeyID)
            ShowCursor;
            sca;
            return
        end
    end

    
    
    end
    else
        pause(goDuration);
    end
    Screen('Flip', window, grey);
    % Check the queue and if button is released, send trigger.
    if robCond ~= "robot" 
    while goTrigger == 0
        [ ~, ~, firstRelease, ~, lastRelease ] = KbQueueCheck();
        if (firstRelease(responseKeyID) || lastRelease(responseKeyID) >= 1) && goTrigger == 0
            % Send the cue trigger.
            io64(ioObj,address,96); % No reason to use 32, except that it is the KeyCode for spacebar.
            pause(0.01);
            io64(ioObj,address,0);
            goTrigger = 1;
            goMove = GetSecs - goStart;
        elseif firstRelease(escapeKeyID) || lastRelease(escapeKeyID)
            ShowCursor;
            sca;
            return
        end
    end
    end
   
    %----------------------------------------------------------------------
    %                        Robot Interaction
    %----------------------------------------------------------------------

    
    
    randStim = (rand * 500) + 1000; 
    stimDuration = randStim / 1000;
    roboMove = 0;
    action_finish = false;
    
    % Setup as a while loop to allow for trigger to be sent at button press.
   
    while action_finish == false
    % Flip to the screen
    Screen('Flip', window, grey);
     if robCond ~= "not moving"
         
       
           while(roboTrigger == 0)
                  disp("robottrigger");
                  try
                     state = sub.LatestMessage.Data; 
                     disp(state);
                  catch ME
                      state = "not moving";
                  end
                 
                  if state == "moving"
                      if robotMove == false
                          robotMove = true; 
                          roboStart = GetSecs;
                          
                          io64(ioObj,address,20); %robot moving trigger
                          pause(0.002);
                          io64(ioObj,address,0);
                      end
                      disp("robot finally moving");
                      
                    % Check the queue and if button is released, send trigger.
                      if (firstRelease(responseKeyID) || lastRelease(responseKeyID) >= 1) && roboTrigger == 0
                        % Send the cue trigger.
                        if robCond == "robot"
                            io64(ioObj,address,96); 
                            pause(0.002);
                            io64(ioObj,address,0);
                            goTrigger = 1;
                            goMove = GetSecs - goStart;
                            msg = rosmessage(pub);
                            msg.Data = convertStringsToChars("released");
                            send(pub, msg);
           
                        end
                        roboTrigger = 1;
                        disp("trigger sent");
                      elseif firstRelease(escapeKeyID) || lastRelease(escapeKeyID)
                        ShowCursor;
                        sca;
                        return
                      else
                         [ ~, ~, firstRelease, ~, lastRelease ] = KbQueueCheck();

                        
                      end
                      
                      
                  end
           end
        else
            roboStart = GetSecs;
            robotMove = true;
        end
       

    roboTrigger = 0;
     if robCond ~= "not moving"
        while (robotMove == true)
         try
        
             if(sub.LatestMessage.Data ~= "moving")
                robotMove = false;
                action_finish = true;
                disp("stopped moving");
                handover_result = sub.LatestMessage.Data;
                roboStop = GetSecs;
                roboDuration = roboStop - roboStart;
             end
         end
         try
             disp(sub_stop.LatestMessage.Data);
             %Pauses matlab script until the robot script tells it to
             %continue
             if(sub_stop.LatestMessage.Data == "stop")
                handover_result = "Unsuccesfull";
                roboDuration = 0;
                robotMove = false;
                action_finish = true;
                rosshutdown;
                restart = 0;
                disp("shut down");
                while(restart == 0)
                    
                      disp('restarting');
                try 
                    rosinit
                    pub = rospublisher('/action','std_msgs/String'); 
                    sub = rossubscriber( '/trigger');
                    handover_result = "None";
                    state = "not moving";
                    msg = rosmessage(pub); 
                    msg.Data = 'None'; 
                    send(pub,msg);
                    robotMove = false;
                    sub_stop = rossubscriber('/stop');
                    pause(1);
                    disp('reestablish');
                    go_robot = 0;
                    
                    while (go_robot == 0)
                      try
                        if (sub_stop.LatestMessage.Data == "go" )
                            go_robot = 1;
                            restart = 1;
                        else
                            pause(0.2);
                           end
                        
                      catch ME
                          pause(0.2);
                      end
                    end

                  catch ME
                     pause(0.5);
                end
                    
                end
  
             end
         end
            
        
         
         

         
       



        end
     else
         action_finish = true;

        end
    

    end
    action_finish = false;
    robotMove = false;
    roboTrigger = 0;
    disp(action_finish);
    
           
      
    
    %----------------------------------------------------------------------
    %                       Button Press Output
    %----------------------------------------------------------------------
    % Button press return value, accept if cue is true.
    trialEnd = GetSecs - start_trial;
    stimToc = toc;


    % If escape is pressed, finish study.
    if firstRelease(escapeKeyID) || lastRelease(escapeKeyID)
        ShowCursor;
        sca;
        return
    end
    

    responseID = find(lastRelease(responseKeyID));
    
    % Trial response time calculation, and indicating responseCode.
    if length(responseID) == 1
        RT = lastRelease(responseKeyID(responseID)) - goStart;
        responseID2 = find(lastRelease(responseKeyID));
        responseCode = 1;
    else
        RT = 0;
        responseID2 = 0;
        responseCode = 0;
    end
    
    % Information about Accuracy, if single (1) or joint movement (2) and
    % release key, or no movement (3) and no release key then ACC = 1.
    if (responseCode == 1 && cueState == 1 ) || (responseCode == 0 && cueState == 3) || (responseCode == 1 && cueState == 2)
        ACC = 1;
    else
        ACC = 0;
    end
    
    %----------------------------------------------------------------------
    %                          Response Matrix
    %----------------------------------------------------------------------
    % Record the trial data into the output matrix.
    respMatrix(1,nTrial) = nTrial;
    respMatrix(2,nTrial) = block;
    respMatrix(3,nTrial) = trialCondition;
    respMatrix(4,nTrial) = cueState;
    respMatrix(5,nTrial) = shape_id;
    respMatrix(6,nTrial) = RT;
    respMatrix(7,nTrial) = responseCode;
    respMatrix(8,nTrial) = ACC;
    respMatrix(9,nTrial) = roboDuration;
    
    % Time testing.
    
    respMatrix(10,nTrial) = stimToc;
    if handover_result == "Succesfull"
        handover_result = 1;
    else
        handover_result = 0;
    end
    respMatrix(11,nTrial) = handover_result;
    handover_result = "None";

 
end
%--------------------------------------------------------------------------
%                          Study Completion
%--------------------------------------------------------------------------
% Turn off button entry.
KbQueueStop();

% Invert respMatrix so it is easy to read.
respMatrix = respMatrix.';
%With handover result
respTable = array2table(respMatrix,'VariableNames', {'Trial_Number','Block_Number','Block_Condition', 'Cue_Type', 'Shape_meaning', 'RT', 'Response', 'Accuracy', 'Robo_Duration', 'Trial_Duration', 'Handover_result'});
msg = rosmessage(pub);

% End screen information.
if block ~=6
    
    DrawFormattedText(window, 'The block is finished. \n\n If you wish to have a break please take one now. \n\n Press Any Key To Continue',...
           'center', 'center', black);
    msg.Data = convertStringsToChars("blockdone");

else
    DrawFormattedText(window, 'Thank you for taking part in this study. \n\n Please press any button and tell the researcher to finish.',...
    'center', 'center', black);
    msg.Data = convertStringsToChars("finished");

end

send(pub, msg);
if participantID ~= "sara" 
    directory = ['C:\toolbox\Data\Joint_Action\BehaviouralData\' participantID '\' num2str(block)];
    mkdir (directory);
    writetable(respTable, ['C:\toolbox\Data\Joint_Action\BehaviouralData\' participantID '\' num2str(block) '\' 'TrialData-' participantID '-' num2str(block) '.csv'], 'Delimiter', ',');
end
pause(5);
Screen('Flip', window);
KbStrokeWait;

% Clear the screen.
sca;

%--------------------------------------------------------------------------
%                             Save Files
%--------------------------------------------------------------------------
% Create Folder and save files.


% Clear all.
clc;
clearvars;
