% MATLAB Script for Extracting MFCC Characteristics of Bird Audio Recordings
% This script processes MP3 audio recordings (downloaded from Xeno-Canto dataset)
% to extract Mel-Frequency Cepstral Coefficients (MFCCs) as input features
% and assigns labels (0/1) to indicate if the audio belongs to a target 
% bird species: Lesser Kestrel.
% The extracted data is saved for later use in machine learning training.

%% INPUT SETUP
clear all;   % Clear workspace variables
nbanks = 41; % Number of mel filter banks for MFCC calculation
N = 11;      % Number of classes (bird species + background)
lwindow = 1024; % Window length for signal processing
fs = 32000;     % Target sampling frequency in Hz

% Resampling configuration
FS = 44100;         % Original sampling frequency of files
[P,Q] = rat(fs/FS); % Resampling ratio to achieve target fs

% Initialize data storage
data = {};

% Define species information for processing
% Format: {'SpeciesName', 'LabelIndex', 'FilePrefix', 'DirectoryPath'}
speciesList = {
    'FalcoNaumanni', 1, 'f*', 'dataset\LesserKestrel';
    'StreptopeliaDecaocto', 2, 'Tors*', 'dataset\Other\StreptopeliaDecaocto';
    'ColumbaLivia', 3, 'Pal*', 'dataset\Other\ColumbaLivia';
    'PasserDomesticus', 4, 'Gor*', 'dataset\Other\PasserDomesticus';
    'FalcoTinnunculus', 5, 'Cer*', 'dataset\Other\FalcoTinnunculus';
    'ColoeusMonedula', 6, 'Gra*', 'dataset\Other\Coloeusmonedula';
    'ApusApus', 7, 'Ven*', 'dataset\Other\ApusApus';
    'EmberizaCalandra', 8, 'Tri*', 'dataset\Other\EmberizaCalandra';
    'UpupaEpops', 9, 'Abu*', 'dataset\Other\UpupaEpops';
    'SturnusUnicolor', 10, 'stu*', 'dataset\Other\SturnusUnicolor';
    'Background', 11, 'no-target*', 'dataset\Other\Background';
};

% Initialize counters for total duration and inputs for each species
totalDuration = zeros(1, N); 
ninputs = zeros(1, N);

current_path = pwd;
%% PROCESS EACH SPECIES
for i = 1:size(speciesList, 1)
    % Get species information
    speciesName = speciesList{i, 1};
    labelIndex = speciesList{i, 2};
    filePrefix = speciesList{i, 3};
    filedir = speciesList{i, 4};
    
    % Find all relevant files in the directory with the specified prefix
    matfiles = dir(fullfile(filedir, filePrefix));
    nfilesb = length(matfiles); % Number of audio files found
    
    % Set label vector for the species: one-hot enconding
    label = zeros(1, N);
    label(labelIndex) = 1;
    
    % Call external script for data extraction
    % Assumes 'data_extract' processes audio files and returns 'bird', 
    % 'trecord' (total duration), and 'ninputsloc' (number of inputs)
    cd(current_path);
    data_extract;
    
    % Append extracted data for this species
    data = [data; bird]; 
    
    % Store total duration and input count for this species
    totalDuration(labelIndex) = trecord;
    ninputs(labelIndex) = ninputsloc;
end

% Summarize total number of inputs across all species
totalInputs = sum(ninputs);

%% SAVE EXTRACTED DATA
save('extracted_data.mat', 'data', 'fs', 'nbanks', 'totalInputs', ...
    'totalDuration', 'ninputs');

