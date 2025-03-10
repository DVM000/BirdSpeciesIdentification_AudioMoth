% This script loads MP3 files, applies MFCCs, passes them through a pre-trained neural network,
% and then thresholds the output to detect bird calls. Results are visualized for manual evaluation of detection.

% Load the bird song audio file
[song, FS] = audioread('XC895702.wav'); bird = '\it Falco Naumanni'; % Label the bird species 
[song, FS] = audioread('XC975924.wav'); bird = '\it Falco tinnunculus';
%[song, FS] = audioread('XC840017.wav'); bird = '\it Columba livia';
%[song, FS] = audioread('XC861083.wav'); bird = '\it Sturnus unicolor';
%[song, FS] = audioread('XC977002.mp3'); bird = '\it Streptopelia decaocto';
%[song, FS] = audioread('20210505_171900.WAV');  bird = '\it Falco Naumanni';

song = song(:,1); % Use the first channel in case of stereo audio (mono audio)

% NN parameters
th = 0.5;                                   % Threshold for detection (0.5 by default)
NNfunction = @(x) NeuralNetworkFunction(x); % Trained Neural Network
average_frames = false;

% Define sampling parameters
fs = 32000;   % Desired sampling rate (lower resolution for processing)
[P, Q] = rat(fs / FS);  % Resampling ratio to convert the audio sample rate
song = resample(song, P, Q); % Resample audio to match fs

% Spectrogram parameters
lwindow = 1024; % Window size for spectrogram
T = (length(song)-1) / fs;  % Duration of the song in seconds
t = 0:1/fs:(length(song)-1)/fs; % Time vector
f = fs/1024:fs/1024:fs/2; % Frequency vector for spectrogram

% Compute the spectrogram
[spec, f, tspec, psd] = spectrogram(song, lwindow, lwindow/2, f, fs); % Spectrogram computation

% MFCCs computation
nbanks = 41; % Number of Mel-frequency filter banks
twindow = lwindow/fs; % 0.032; % Time window for MFCC calculation
[dctcoeff, d] = mfccs(song', nbanks, twindow, fs, 1e10);  % Apply MFCC feature extraction

% Neural network classification (use pre-trained neural network)
% Feeding the MFCC coefficients into the neural network for classification
[y] = NNfunction([dctcoeff(:, 2:13) d(:, 2:13)]'); % Feed MFCCs into the network
y1 = y(1,:);  % Use the output from the neural network (first row, which is the predicted positive class probabilities)

% Initialize detection arrays and thresholds
samples = 16;  % Number of samples to average for detection (if average_frames is true)
detect = zeros(1, length(y)); % Store detection results by using threshold

% Compute detection based on thresholded output
if average_frames == false
    detect = y>th;
    %detect = y;
else
    % Detection loop
    for k = 1:samples:length(y1)-samples
        % Apply thresholding to classify detection at each time frame
        if (sum(y1(k:k+samples-1)) > th*samples)
            detect(k:k+samples) = 1;
        end
    end
end


% Visualization: Plot detection results, spectrogram, and song waveform
% Split the visualizacion into partial plots, each one of nframes*twindow seconds
figure
nframes = 60; % Number of frames to plot per subplot (time resolution)
t1 = 0:lwindow/fs:lwindow/fs*length(y); % Time vector for plotting

for i = 1:floor(length(t1)/nframes)
     % Plot the spectrogram
    subplot(3, 1, 3)
    surf(tspec(1+nframes*2*(i-1):i*2*nframes), f, log(abs(spec(:,1+nframes*2*(i-1):i*2*nframes))),'EdgeColor','None')
    xlim([tspec(1+nframes*2*(i-1)) tspec(i*2*nframes)])
    view(2)
    xlabel('Time')
    ylabel('Frequency')
    ylim([0 15000])

    % Plot the detection results for different thresholds
    subplot(3, 1, 1)
    a1 = plot(t1(1+nframes*(i-1):i*nframes), detect(1,1+nframes*(i-1):i*nframes), 'Color', [0 0.5 0], 'LineWidth', 1.5);
    hold on
    a2 = plot(t1(1+nframes*(i-1):i*nframes), y(1,1+nframes*(i-1):i*nframes), '--', 'Color', [0 0 0.5]); 
    xlim([t1(1+nframes*(i-1)) t1(i*nframes+1)]);
    xlim([t1(1+nframes*(i-1)) t1(i*nframes+1)]);
    ylim([0 1]);
    lgd = legend([a1;a2], ['NN output >', num2str(th)],'NN output', 'Location', 'southwest');
    lgd.FontSize = 6;
    xlabel('Time (s)')
    ylabel('NN detection')
    title(bird)

    % Plot the original song waveform
    subplot(3, 1, 2)
    plot(t(1+nframes*lwindow*(i-1):i*lwindow*nframes), song(1+nframes*lwindow*(i-1):i*lwindow*nframes))
    xlim([t(1+nframes*lwindow*(i-1)) t(i*lwindow*nframes)])
    xlabel('Time(s)')
    ylabel('Amplitude')

    % Play a segment of the song
    sound(song(1+nframes*lwindow*(i-1):i*lwindow*nframes), fs)

    % Pause for visualization update
    pause
end

%saveas(gcf,'Test.png')