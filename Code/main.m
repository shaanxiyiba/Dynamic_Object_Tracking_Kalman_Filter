obj_vid = 'object_video.mp4'; % name of video file
frames = VideoReader(obj_vid); % read frames from video
width = frames.width; % get width of frame
height = frames.height; % get height of frame
no_of_frames = frames.NumberOfFrames; % get number of frames

% The next lines of code extract the dark frame from the input video. 
% The dark frame is computed as a simple average across all frames. 
% Since the object to be tracked only occupies a small percentage of the
% pixels in any given frame, changes in it's position or intensity should
% not affect the dark frame statistics.

step_frame = 1; % no. of frames to be stepped over
dark = zeros(height,width,3); % initialize dark frame as zeros

% average over all frames
for i=1:step_frame:no_of_frames-step_frame
    dark = dark + double(read(frames,i));
end
dark_frame = step_frame*dark/(no_of_frames);

threshold = 10; % Threshold for generating binary image of the noise

% Kalman Filter Definition
% Time increment
dt=0.5;

% Transition matrices
A = [1 0 dt 0;
     0 1 0 dt;
     0 0 1 0 ;
     0 0 0 1 ;
     ];

B = [(dt^2)/2 (dt^2)/2 dt dt]';

% acceleration
u = 4e-3;

% Position (x,y) transition matrix
H = [1 0 0 0;
     0 1 0 0];
 
% State covariance matrix
State_Uncertainty = 10;
S = State_Uncertainty * eye(size(A,1)); % diagonal since state variables are independent

% Diagonal measurement noise covariance matrix
Meas_Unertainty = 1;
R = Meas_Unertainty * eye(size(H,1));

% State transition noise covariance matrix
% Dyn_Noise_Variance = (0.01)^2;

Q = [(dt^2)/4 0 (dt^3)/2 0;
     0 (dt^2)/4 0 (dt^3)/2;
     (dt^3/2) 0 (dt^2) 0;
     0 (dt^3)/2 0 (dt^2);
     ];

% Initialize Kalman filter 
kalman_output = [];
x = [height/2; width/2; 0; 0;]; % Initial values

% Extract centroids
moving = zeros(height,width,no_of_frames);
labeled_frames = zeros(height,width,no_of_frames);
bb=0;
for i=1:no_of_frames-1
    current_frame = double(read(frames,i));
    moving(:,:,i) = (abs(current_frame(:,:,1) - dark_frame(:,:,1)) > threshold)...
                   |(abs(current_frame(:,:,2) - dark_frame(:,:,2)) > threshold)...
                   |(abs(current_frame(:,:,3) - dark_frame(:,:,3)) > threshold);
    moving(:,:,i) = bwmorph(moving(:,:,i),'erode',2);
    labeled_frames(:,:,i) = bwlabel(moving(:,:,i),4); 
    stats{i} = regionprops(labeled_frames(:,:,i),'basic');
    [n_obj,features] = size(stats{i});
    area = 0;
    if(n_obj ~= 0) 
         for k=1:n_obj
             if(stats{i}(k).Area > area)
                id(i) = k;
                area = stats{i}(k).Area;
             end
         end
    centroid(:,:,i) = stats{i}(id(i)).Centroid;
    else
        centroid(:,:,i) = [rand*200 rand*200];
        bb = bb+1;
    end
    i %indicates the frame number
end

% Static centroid tracking
for r=1:no_of_frames-1
    centroid_frames = read(frames,r);
    centroid_frames = insertShape(centroid_frames,'circle',[centroid(1,1,r) centroid(1,2,r) sqrt(stats{r}(id(r)).Area/pi)],'LineWidth',1,'Color','blue');
    marked_centroids(:,:,:,r) = centroid_frames;
    centroid_x(r) = centroid(1,1,r);
    centroid_y(r) = centroid(1,2,r);
end

% Kalman filter tracking
for r=1:no_of_frames-1
    kalman_frames = read(frames,r);
    kalman_frames = insertShape(kalman_frames,'circle',[centroid(1,1,r) centroid(1,2,r) sqrt(stats{r}(id(r)).Area/pi)],'LineWidth',1,'Color','blue');
    % Updating frames using the Kalman filter
     
    % Fetch centroid for every frame
    input = [centroid(1,1,r); centroid(1,2,r)];
    
    % Estimate the next state
    x = A*x + B*u;
    % Estimate the error covariance 
    S = A*S*A' + Q;
    % Kalman Gain Calculations
    K = S*H'*inv(H*S*H'+R);
    % Update the estimation with current input
    x = x + K*(input - H*x);
    % Update the error covariance
    S = (eye(size(S,1)) - K*H)*S;
    % Save the measurements for plotting
    kalman_output = H*x;
    output_frames = insertShape(kalman_frames,'circle',[kalman_output(1) kalman_output(2) 4],'LineWidth',2,'Color','red');
    tracking_output_x(r) = kalman_output(1);
    tracking_output_y(r) = kalman_output(2);
    tracking_frames(:,:,:,r) = output_frames;
    r
end