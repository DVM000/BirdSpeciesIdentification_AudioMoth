# -*- coding: utf-8 -*-
"""
Processing multiple .wav audio files in specified folder
Author: delia
Modified: 2026
"""

import numpy as np
import librosa
import librosa.display
import scipy.signal
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from scipy.io import wavfile
from scipy.signal import resample_poly
import scipy.fftpack
import os
import argparse

# ---------------------- FUNCIONES ----------------------

def melfilterbanks(flow, fhigh, nbanks, f, fs, l, n):
    """
    MELFILTERBANKS Calculates the values of Mel filter banks.
    
    Inputs:
    - flow:      Lowest frequency in Hz.
    - fhigh:     Highest frequency in Hz.
    - nbanks:    Number of filter banks.
    - f:         Frequency vector (DFT points).
    - fs:        Sampling frequency in Hz.
    - l:         Scale type (0: Mel, 1: Hybrid Mel-Linear).
    - n:         Normalization type (0: None, 1: Area normalization).
    
    Outputs:
    - H:         Filter bank matrix.
    - indf:      Indices corresponding to central frequencies of each filter.
    """
    k = len(f)  # Number of DFT points

    # Determine central frequencies based on scale type
    if l == 0:
        # Pure Mel scale
        mflow = 1125 * np.log(1 + flow / 700)
        mfhigh = 1125 * np.log(1 + fhigh / 700)
        mel = np.linspace(mflow, mfhigh, nbanks + 2)
        hertz = 700 * (np.exp(mel / 1125) - 1)  # Convert Mel to Hz

    elif l == 1:
        # Hybrid Mel-Linear scale
        fspaced = (fhigh - flow) / nbanks  # Initial linear spacing
        tolerance = 1e-3  # Tolerance for convergence
        max_fspaced = fhigh
        min_fspaced = 0

        # Refine linear spacing using hybrid Mel-linear logic
        while True:
            fl = np.arange(flow, flow + 10 * fspaced, fspaced)  # Linear spaced frequencies
            mel_spacing = 1125 * np.log(1 + fl[-1] / 700) - 1125 * np.log(1 + fl[-2] / 700)
            fm = np.arange(1125 * np.log(1 + fl[-1] / 700),
                           1125 * np.log(1 + fl[-1] / 700) + (nbanks + 1 - 10) * mel_spacing,
                           mel_spacing)
            hertz = np.concatenate((fl[:-1], 700 * (np.exp(fm / 1125) - 1)))  # Combine linear and Mel

            # Adjust linear spacing based on the final frequency
            if hertz[-1] > fhigh:
                max_fspaced = fspaced
                fspaced = (fspaced + min_fspaced) / 2
            else:
                min_fspaced = fspaced
                fspaced = (fspaced + max_fspaced) / 2

            # Check convergence
            if abs(hertz[-1] - fhigh) <= tolerance:
                break

    else:
        raise ValueError("Invalid scale type. Use 0 for Mel or 1 for Hybrid Mel-Linear.")

    # Find indices corresponding to central frequencies
    indf = np.zeros(len(hertz), dtype=int)
    for i in range(len(hertz)):
        idx = np.where(f >= hertz[i])[0]
        indf[i] = idx[0] if len(idx) > 0 else len(f) - 1  # Assign last index if frequency exceeds range

    # Create the triangular filter bank matrix
    H = np.zeros((nbanks, k))  # Initialize filter bank matrix
    for j in range(nbanks):
        if n == 0:
            normalization = 1  # No normalization
        elif n == 1:
            normalization = 2 / (f[indf[j + 2]] - f[indf[j]])  # Area normalization
        else:
            raise ValueError("Invalid normalization type. Use 0 for none or 1 for area normalization.")

        # Define the triangular filter for each bank
        len1 = indf[j + 1] - indf[j]
        len2 = indf[j + 2] - indf[j + 1]

        H[j, indf[j]:indf[j + 1]] = normalization * np.linspace(0, 1, num=len1, endpoint=False)
        H[j, indf[j + 1]:indf[j + 2]] = normalization * np.linspace(1, 0, num=len2, endpoint=False)

    # Return indices of the filter banks (exclude the first and last frequencies)
    return H, indf[1:nbanks + 1]


def calculate_mfccs(y, nbanks, twindow, fs, maxsegments=1e10):
    """
    Calculates Mel Frequency Cepstral Coefficients (MFCCs) and their deltas.

    Parameters:
    - y: Audio signal (1D numpy array)
    - nbanks: Number of Mel filter banks
    - twindow: Window duration in seconds
    - fs: Sampling frequency in Hz
    - maxsegments: Maximum number of segments (not used here)

    Returns:
    - dctcoeff: MFCC coefficient matrix (frames x nbanks)
    - delta: Delta coefficient matrix (frames x nbanks)
    """

    # Window length in samples
    lwindow = round(twindow * fs)
    
    # Number of windows and zero-padding if necessary
    nwindow = int(np.ceil(len(y) / lwindow))
    padLength = nwindow * lwindow - len(y)
    y = np.concatenate((y, np.zeros(padLength)))

    # FFT parameters
    NFFT = 1024
    freqVector = np.linspace(0, fs / 2, NFFT // 2)  # Frequency vector

    # Create Mel filter banks
    H, indf = melfilterbanks(300, fs / 2, nbanks, freqVector, fs, 0, 1)

    # Initialize output matrices
    dctcoeff = np.zeros((nwindow, nbanks))

    ####### MFCC Calculation ########
    for frameIdx in range(nwindow):
        # Extract segment and apply Hamming window
        segment = y[frameIdx * lwindow:(frameIdx + 1) * lwindow] * np.hamming(lwindow)
        
        # Magnitude spectrum
        spectrum = np.abs(np.fft.fft(segment, NFFT))[1:NFFT//2+1]

        # Apply Mel filter banks and compute log-energy
        Efilterbank = np.log10(np.dot(H, spectrum) + 1e-10)  # Evitar log(0)

        # Apply DCT to log-energies to obtain MFCC coefficients
        dctcoeff[frameIdx, :] = scipy.fftpack.dct(Efilterbank, norm='ortho') 
        # https://docs.scipy.org/doc/scipy/reference/generated/scipy.fftpack.dct.html
        #   For a single dimension array x, dct(x, norm='ortho') is equal to MATLAB dct(x).

    ####### Delta Calculation ########
    N = 2  # Window size for finite difference
    delta = np.zeros_like(dctcoeff)
    
    # Extend matrix for edges
    dctcoeff_extended = np.vstack([dctcoeff[0:1, :]] * N + [dctcoeff] + [dctcoeff[-1:, :]] * N)

    denominator = 2 * sum(i**2 for i in range(1, N+1))
    
    for frameIdx in range(nwindow):
        delta[frameIdx, :] = sum(i * (dctcoeff_extended[frameIdx + N + i] - dctcoeff_extended[frameIdx + N - i]) for i in range(1, N+1)) / denominator

    return dctcoeff.T, delta.T


def softmax(n):
    n_max = np.max(n, axis=0, keepdims=True)
    e_n = np.exp(n - n_max)
    return e_n / np.sum(e_n, axis=0, keepdims=True)

def tansig(n):
    return 2 / (1 + np.exp(-2 * n)) - 1

def NeuralNetworkFunction(X):
    # NN constants
    b1 = np.array([[1.8916010551494097935], [2.0087474377775778045]])
    IW1_1 = np.array([
        [0.1998886754907664709, 0.57158025640225418318, 0.073053558551228664486, 0.30815336956943989444, -0.36003003489407331417, 0.35955390517506552461, -0.55180303576284406297, 0.36854512746086781627, -0.11990989497331375202, -0.12415192057333071518, -0.11036941186308167617, 0.17488461021140797036, -0.013529466284958970024, -0.14750359472676494166, 0.055395201464141327619, 0.37214938872552660865, -0.24550221656609863552, 0.26630143218921387138, -0.47940732926561213656, -0.50824973434697218178, 0.38211640719584838433, 0.62598088027513776321, 0.23283702476734549625, 0.67685222502032615921],
        [-0.58969760140591542807, 0.13612969647931399964, 0.28130343863305534713, 0.44360322873237156838, 0.025224012033459614068, -0.40590449389022426052, 1.1143112752679664723, -0.2161903374887501339, 0.23162703430932926607, -0.32741518794124441216, 0.47349665037919996813, -0.45920452643889042577, -0.14105785899327966115, 0.031379435557255155875, 0.35720107620842272977, 0.21229694142108621047, -0.20368656129300038993, 0.1738414724904205344, -0.35063776391904794005, -0.6775042296727354918, -0.17173184806768476696, 0.46230394894357262903, 0.14771323500073230139, 0.79802613525304866293]
    ])
    b2 = np.array([[-0.089268169590198245822], [0.52228923760607492977]])
    LW2_1 = np.array([[-0.44486678654834321822, 1.1688121908846942354], [1.0641647684820378927, -0.85051800764122009735]])
    
    if not isinstance(X, list):
        X = [X]
    
    TS = len(X)  # Number of timesteps
    Q = X[0].shape[1] if X else 0  # Number of samples
    Y = []
    
    for ts in range(TS):
        a1 = tansig(b1 + IW1_1 @ X[ts])
        a2 = softmax(b2 + LW2_1 @ a1)
        Y.append(a2)
    
    return np.hstack(Y) #if not isinstance(X, list) else Y

# ---------------------- PROCESSING ----------------------
def group_detections(y, duration, threshold=0.5, timeblock=3.0, operation="binary_any"):
    """
    Group binary detections into time-second blocks (ex: 3 seconds), with "any" or "sum" operation;
     or agregate output confidence with "mean" operation.
    - y:  array with values in the range [0,1]
    - threshold: binary threshold to get detections from 'y'.
    - timeblock: time interval to agregate detections.
    - operation: 
        - "binary_any": apply "any" operation to binary detections. 
        - "binary_sum": apply "sum" operation to binary detections. 
        - "mean":       apply "mean" operation to output confidence 'y'.
    """
    n_frames = duration / len(y)
    block_size_frames = round(timeblock / n_frames)   # timeblock multiple of n_frames
    timeblock_adjusted = block_size_frames * n_frames

    n_blocks = int(np.floor(len(y) / block_size_frames)) # number of frames per time block

    detections_grouped = []

    for i in range(n_blocks):
        block = y[i*block_size_frames:(i+1)*block_size_frames]
        if operation=="binary_any":
            block = block > threshold
            detections_grouped.append(1 if np.any(block) else 0) # timeblock classified as 1 if there is any detection in the block
        if operation=="binary_sum":
            block = block > threshold
            detections_grouped.append(np.sum(block)) # return sum of detections in the block
        if operation=="mean":
            detections_grouped.append(np.mean(block)) # return average of output confidences     
        #print(f"Bloque {i}: {y[i*block_size_frames:(i+1)*block_size_frames]} -> mean={np.mean(block)}")
        
    return np.array(detections_grouped), timeblock_adjusted # returns aggregated detections (using operation) and timeblock



def process_files(input_dir, output_file, threshold, plotting=False, operation="binary_any", timeblock=1.0):

    # Spectrogram parameters
    fs = 32000
    lwindow = 1024

    # MFCC calculation
    nbanks = 41
    twindow = lwindow / fs
            
    results = []
    list_files = sorted(os.listdir(input_dir))
    for filename in list_files:
        if filename.lower().endswith('.wav') or filename.lower().endswith('.mp3'):
            filepath = os.path.join(input_dir, filename)
            print(f"Processing: {filename}")
            song, FS = librosa.load(filepath, sr=None, mono=False)
            if song.ndim == 2:
                song = np.mean(song, axis=0) #song[1,:] # one-channel 
  
            
            song = resample_poly(song.astype(np.float64), fs, FS)  # Resample audio
            
            T = len(song) / fs
            t = np.arange(0, len(song)) / fs
            f = np.linspace(fs / 1024, fs / 2, lwindow // 2 + 1)
            f, tspec, spec = scipy.signal.spectrogram(song, fs, nperseg=lwindow, noverlap=lwindow // 2) # Spectrogram
            
            mfccs, delta = calculate_mfccs(song, nbanks, twindow, fs) # Calculate MFCC and deltas 
            mfccs12 = mfccs[1:13, :]
            deltas12 = delta[1:13, :]
            y = NeuralNetworkFunction(np.concatenate([mfccs12, deltas12]))[0, :] # Neural Network
          
            detect = y > threshold # Threshold
            detections = np.sum(detect) 
            
            detections_block, timeblock_adjusted = group_detections(y, T, threshold, timeblock=timeblock, operation=operation)
            #detections_block = group_detections(y, T, threshold, timeblock=1.0, operation="binary_any")
            ##detections_block = group_detections(y, T, threshold, timeblock=0.5, operation="mean")  
            if operation == "mean":
                detections_block = detections_block > threshold # in case we use "mean" -> get blocks marked as detection         
            print(detections, np.sum(detections_block))
            
            results.append(f"{filename}, {int(detections)}, {np.sum(detections_block)}")
            # Prints: 
            #     filename, total 32-ms detections, total timeblock-seg detections (aggregated using 'operation' and thresholded with 'threshold'
            
            # -- plotting figure --
            if plotting:
                nframes = int(spec.shape[1]/2)
                t1 = np.linspace(0, len(y) * twindow, len(y))
                
                if operation == "mean":
                    detections_mean = detections_block
                    block_size = int(timeblock_adjusted / twindow)  # number of frames per block
                    t_blocks = np.arange(len(detections_mean)) * timeblock_adjusted  # start time of each block  
            
                plt.figure(figsize=(15,7))
                plt.subplot(3, 1, 3)
                plt.pcolormesh(tspec[:nframes * 2], f, np.log(np.abs(spec[:, :nframes * 2]) + 1e-10), shading='auto')
                plt.ylabel('Frequency (Hz)')
                plt.xlabel('Time (s)')
                plt.ylim([0, 15000])
                
                plt.subplot(3, 1, 1)
                plt.plot(t1[:nframes], detect[:nframes], 'g-', linewidth=1.0, label=f'NN output > {threshold}')
                plt.plot(t1[:nframes], y[:nframes], 'b--', label='NN output')
                if operation == "mean":
                    for i, val in enumerate(detections_mean):
                        if val > threshold:
                            plt.axvspan(t_blocks[i], t_blocks[i]+timeblock_adjusted, color='black', alpha=0.3)
                darkpatch = mpatches.Patch(color='black', alpha=0.3, label=f'NN average ({timeblock} s) > {threshold}')
                #plt.step(t_blocks, detections_block, where='post', color='orange', label='Media por bloque (mean)')
                plt.ylim([0, 1])
                plt.xlim([0, T])
                plt.xlabel('Time (s)')
                plt.ylabel('NN Detection') 
                if operation == "mean":
                    plt.legend(handles=[plt.Line2D([], [], color='b', linestyle='--', label='NN output'),
                    			plt.Line2D([], [], color='g', label=f'NN output > {threshold}'),  
                        		darkpatch])
                else:
                    plt.legend()

                plt.subplot(3, 1, 2)
                plt.plot(t[:nframes * lwindow], song[:nframes * lwindow])
                plt.xlabel('Time (s)')
                plt.ylabel('Amplitude')
                plt.xlim([0, T])

                plt.tight_layout()
                plt.show()
                plt.savefig(f'fig-{filename}.png')
            # -----
      
    print(f"Employed th={threshold}, timeblock={timeblock_adjusted} (adjusted), operation={operation} ")        
    with open(output_file, 'w') as f:
        for line in results:
            f.write(line + '\n')
    print(f"\nResults saved in: {output_file}")
    print(f"Lines containing: filename, total 32-ms detections, total timeblock-seg detections (aggregated using '{operation}' and thresholded at {threshold}")

# ---------------------- MAIN ----------------------

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Procesar audio files and save detections.")
    parser.add_argument("--i", "--input", dest="input_dir", default='audios', help="Audio folder")
    parser.add_argument("--o", "--output", dest="output_file", default='results.txt', help="Output .txt file")
    parser.add_argument("--min_conf", dest="threshold", type=float, default=0.5, help="Minimum confidence threshold (default=0.5)")
    parser.add_argument('--plotting', action='store_true', help='Enable plotting (False by default)')
    parser.add_argument('--operation', choices=['binary_any', 'binary_sum', 'mean'], default='binary_any',
                    help='Type of operation for detection agrupation. By default: binary_any')
    parser.add_argument('--timeblock', type=float, default=0.512,
                    help='Time block duration for operation application. 0.512 s by default.')
    args = parser.parse_args()

    process_files(args.input_dir, args.output_file, args.threshold, args.plotting, args.operation, args.timeblock)


