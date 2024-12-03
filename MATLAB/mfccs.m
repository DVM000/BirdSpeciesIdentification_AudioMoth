% MFCCS.M
% Function to calculate MFCCs (Mel-frequency cepstral coefficients) and delta coefficients 
% for an input signal 'y'. It applies Mel-filter banks, Hamming windowing, and discrete cosine
% transformation to obtain MFCCs and calculates deltas using a finite difference method.

function [dctcoeff, delta] = mfccs(y, nbanks, twindow, fs)

% Define the segment window length in samples
lwindow = round(twindow * fs);

% Calculate the number of windows needed and pad the signal if necessary
nwindow = ceil(length(y) / lwindow);           % Number of segments
padLength = nwindow * lwindow - length(y);      % Calculate padding required
y = [y, zeros(1, padLength)];                   % Pad the signal with zeros to fit exact window size

% Parameters for Fourier Transform and frequency vector for filter bank
NFFT = 1024;                                    % FFT length
freqVector = (fs / 2) * linspace(0, 1, NFFT / 2); % Frequency vector (half-spectrum)

% Create the Mel-filter bank using the external melfilterbanks function
% H is the filter bank matrix, and indf gives the filter center frequencies
[H, indf] = melfilterbanks(300, fs / 2, nbanks, freqVector, fs, 0, 1);

% Initialize MFCC coefficients matrix
dctcoeff = zeros(nwindow, nbanks);

%%%% CALCULATE MFCCs %%%%
for frameIdx = 1:nwindow
    % Apply a Hamming window to the current segment
    segment = y((frameIdx - 1) * lwindow + 1 : frameIdx * lwindow) .* hamming(lwindow)';

    % Compute the magnitude spectrum (Periodogram)
    spectrum = abs(fft(segment, NFFT));
    spectrum = spectrum(2:end/2 +1);    %DVM       % Take only half of the spectrum

    % Apply Mel-filter banks and compute log-energy for each filter
    Efilterbank = zeros(1, nbanks);
    for bankIdx = 1:nbanks
        Efilterbank(bankIdx) = log10(sum(spectrum .* H(bankIdx, :)));
    end

    % Calculate the Discrete Cosine Transform (DCT) of the log-energy for MFCC coefficients
    dctcoeff(frameIdx, :) = dct(Efilterbank);
end

%%%% CALCULATE DELTAS %%%%
% Deltas are computed using finite differences over a context window size of N frames
N = 2; % Context window size for delta calculation
delta = zeros(nwindow, nbanks);                 % Initialize delta coefficients

% Extend dctcoeff with N copies of the first and last rows for edge cases
dctcoeffExtended = [repmat(dctcoeff(1, :), N, 1); dctcoeff; repmat(dctcoeff(end, :), N, 1)];

% Calculate the denominator for delta normalization
denominator = 2 * sum((1:N).^2);

% Calculate delta coefficients for each frame
for frameIdx = 1:nwindow
    deltaValue = zeros(1, nbanks);
    % Sum the weighted difference between the surrounding frames
    for offset = 1:N
        deltaValue = deltaValue + offset * (dctcoeffExtended(frameIdx + N + offset, :) - dctcoeffExtended(frameIdx + N - offset, :));
    end
    % Normalize by the denominator and store in delta matrix
    delta(frameIdx, :) = deltaValue / denominator;
end

end
