% This script loads MP3 files, applies MFCCs, passes them through a pre-trained neural network,
% and then thresholds the output to detect bird calls. Results are visualized for manual evaluation of detection.

% Load the bird song audio file
song = audioread('XC895702.wav');%'XC895705.wav'); % Example file
bird = '\it Falco Naumanni'; % Label the bird species (LaTeX formatted for titles)
song = song(:,1); % Use the first channel in case of stereo audio (mono audio)

% Define sampling parameters
fs = 32000;   % Desired sampling rate (lower resolution for processing)
FS = 44100;   % Original sampling rate of the audio file
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
twindow = lwindow/fs;%0.0232; % Time window for MFCC calculation
[dctcoeff, d] = mfccs(song', nbanks, twindow, fs);  % Apply MFCC feature extraction

% Neural network classification (use pre-trained neural network)
% Feeding the MFCC coefficients into the neural network for classification
[y] = finalNeuralNetworkFunction15khz([dctcoeff(:, 2:13) d(:, 2:13)]'); % Feed MFCCs into the network
y1 = y(1,:);  % Use the output from the neural network (first row, which is the predicted positive class probabilities)

% Initialize detection arrays and thresholds
samples = 16;  % Number of samples to average for detection
detect = zeros(1, length(y)); % Store detection results by using threshold

% Define thresholds for detection (adjust these for tuning)
th = 0.4;  % Threshold for detection 

% Detection loop: Compute detection based on thresholded output
for k = 1:samples:length(y1)-samples
    % Apply thresholding to classify detection at each time frame
    if (sum(y1(k:k+samples-1)) > th*samples)
        detect(k:k+samples) = 1;  
    end
end

% Visualization: Plot detection results, spectrogram, and song waveform
figure
nframes = 50; % Number of frames to plot per subplot (time resolution)
t1 = 0:lwindow/fs:lwindow/fs*length(y); % Time vector for plotting

for i = 1:floor(length(t1)/nframes)
    % Plot the detection results for different thresholds
    subplot(3, 1, 1)
    plot(t1(1+nframes*(i-1):i*nframes), detect(1,1+nframes*(i-1):i*nframes))
    xlim([t1(1+nframes*(i-1)) t1(i*nframes+1)])
    ylim([0 1])
    xlabel('Time (s)')
    ylabel(['th>', num2str(th)])
    title(bird)

    % Plot the original song waveform
    subplot(3, 1, 2)
    plot(t(1+nframes*lwindow*(i-1):i*lwindow*nframes), song(1+nframes*lwindow*(i-1):i*lwindow*nframes))
    xlim([t(1+nframes*lwindow*(i-1)) t(i*lwindow*nframes)])
    xlabel('Time(s)')
    ylabel('Amplitude')

    % Plot the spectrogram
    subplot(3, 1, 3)
    surf(tspec(1+nframes*2*(i-1):i*2*nframes), f, log(abs(spec(:,1+nframes*2*(i-1):i*2*nframes))),'EdgeColor','None')
    xlim([tspec(1+nframes*2*(i-1)) tspec(i*2*nframes)])
    view(2)
    xlabel('Time')
    ylabel('Frequency')
    ylim([0 15000])

    % Play a segment of the song
    sound(song(1+nframes*lwindow*(i-1):i*lwindow*nframes), fs)

    % Pause for visualization update
    pause
end

