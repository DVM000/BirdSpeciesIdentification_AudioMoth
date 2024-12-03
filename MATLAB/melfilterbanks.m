function [H, indf] = melfilterbanks(flow, fhigh, nbanks, f, fs, l, n)
% MELFILTERBANKS Calculates the values of Mel filter banks.
%
% Inputs:
% - flow:      Lowest frequency in Hz.
% - fhigh:     Highest frequency in Hz.
% - nbanks:    Number of filter banks.
% - f:         Frequency vector (DFT points).
% - fs:        Sampling frequency in Hz.
% - l:         Scale type (0: Mel, 1: Hybrid Mel-Linear).
% - n:         Normalization type (0: None, 1: Area normalization).
%
% Outputs:
% - H:         Filter bank matrix.
% - indf:      Indices corresponding to central frequencies of each filter.

    k = length(f); % Number of DFT points

    %%% Determine central frequencies based on scale type
    if l == 0
        % Pure Mel scale
        mflow = 1125 * log(1 + flow / 700);
        mfhigh = 1125 * log(1 + fhigh / 700);
        mel = linspace(mflow, mfhigh, nbanks + 2);
        hertz = 700 * (exp(mel / 1125) - 1); % Convert Mel to Hz

    elseif l == 1
        % Hybrid Mel-Linear scale
        fspaced = (fhigh - flow) / nbanks; % Initial linear spacing
        tolerance = 1e-3; % Tolerance for convergence
        max_fspaced = fhigh;
        min_fspaced = 0;

        % Refine linear spacing using hybrid Mel-linear logic
        while true
            fl = flow:fspaced:flow + 10 * fspaced; % Linear spaced frequencies
            mel_spacing = 1125 * log(1 + fl(end) / 700) - 1125 * log(1 + fl(end - 1) / 700);
            fm = 1125 * log(1 + fl(end) / 700):mel_spacing:...
                 1125 * log(1 + fl(end) / 700) + (nbanks + 1 - 10) * mel_spacing;
            hertz = [fl(1:end-1), 700 * (exp(fm / 1125) - 1)]; % Combine linear and Mel

            % Adjust linear spacing based on the final frequency
            if hertz(end) > fhigh
                max_fspaced = fspaced;
                fspaced = (fspaced + min_fspaced) / 2;
            else
                min_fspaced = fspaced;
                fspaced = (fspaced + max_fspaced) / 2;
            end

            % Check convergence
            if abs(hertz(end) - fhigh) <= tolerance
                break;
            end
        end

    else
        error('Invalid scale type. Use 0 for Mel or 1 for Hybrid Mel-Linear.');
    end

    %%% Find indices corresponding to central frequencies
    indf = zeros(1, length(hertz));
    for i = 1:length(hertz)
        if length(find(f>=hertz(i)))>0
            indf(i) = find(f>=hertz(i),1);
        else
            indf(i) = length(f);  % Assign the last index if frequency exceeds the range
        end
    end
 
    %%% Create the triangular filter bank matrix
    H = zeros(nbanks, k); % Initialize filter bank matrix
    for j = 1:nbanks
        if n == 0
            normalization = 1; % No normalization
        elseif n == 1
            normalization = 2 / (f(indf(j + 2)) - f(indf(j))); % Area normalization
        else
            error('Invalid normalization type. Use 0 for none or 1 for area normalization.');
        end

        % Define the triangular filter for each bank
        H(j,:) = [zeros(1,indf(j)-1) ...
            normalization*(((indf(j)):indf(j+1))-indf(j))/(indf(j+1)-indf(j))...
            normalization*(indf(j+2)-((indf(j+1)+1):indf(j+2)))/(indf(j+2)-indf(j+1))...
            zeros(1,(k-indf(j+2)))];
    end

    % Return indices of the filter banks (exclude the first and last frequencies)
    indf = indf(2:nbanks + 1); 
end
