% DATA_EXTRACT.M
% This script extracts audio features (MFCC coefficients) for a given set of audio files.
% It processes each audio file in the directory, resampling the audio to a target sampling 
% frequency, calculating MFCC coefficients, and storing the results along with metadata.

% Initialize counters and storage variables
trecord = 0;       % Total duration of processed audio in seconds
ninputsloc = 0;    % Total number of MFCC inputs for the current species
currentIndex = 1;  % Index for bird cell array
fileIndex = 1;     % Index for the current audio file in matfiles
processingComplete = false;  % Flag to indicate completion of all files
bird = {};         % Cell array to store processed data for the species

% Process each audio file in the directory
while ~processingComplete
    % Read the current audio file
    fprintf('Processing file %s\n', matfiles(fileIndex).name);
    [stereo, FS] = audioread(fullfile(filedir, matfiles(fileIndex).name)); % Load audio file
    [P, Q] = rat(fs / FS);  % Calculate resampling ratio to match target sampling frequency (fs)
    
    % Resample the audio and process if its length is sufficient
    resampledAudio = resample(stereo(:, 1), P, Q); % Resample left channel to target frequency
    if length(resampledAudio) > lwindow
        % Store resampled audio and its metadata
        bird{currentIndex, 1} = resampledAudio;  % Resampled audio signal
        bird{currentIndex, 2} = fs;             % Target sampling frequency
        
        % Create a time vector for the resampled audio
        timeVector = 0:1/fs:(length(stereo) / FS);
        bird{currentIndex, 3} = timeVector(1:end-1); % Truncate to match the length of the signal
        
        % Calculate MFCC coefficients and their deltas using mfccs.m
        twindow = lwindow/fs; % Temporal window length
        [mfccCoeffs, mfccDeltas] = mfccs(resampledAudio', nbanks, twindow, fs);
        bird{currentIndex, 4} = [mfccCoeffs, mfccDeltas]; % Store coefficients and deltas
        
        % Add target metadata
        bird{currentIndex, 5} = label;             % Label vector for this species
        bird{currentIndex, 6} = matfiles(fileIndex).name; % Filename of the processed audio
        
        % Update total duration and input count
        trecord = trecord + length(stereo(:, 1)) / FS; % Convert sample count to seconds
        ninputsloc = ninputsloc + size(mfccCoeffs, 1); % Number of MFCC feature vectors (number of input samples)
        
        % Increment the cell array index
        currentIndex = currentIndex + 1;
    end
    
    % Check if all files have been processed
    if fileIndex == length(matfiles)
        processingComplete = true;
    else
        fileIndex = fileIndex + 1; % Move to the next file
    end
end

% Reset the processing complete flag (if reused elsewhere in the script)
processingComplete = false;
