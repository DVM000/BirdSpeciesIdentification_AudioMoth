# Falco Naumanni Presence Detection

This repository contains MATLAB code and AudioMoth firmware modifications for detecting the presence of the _Falco naumanni_ species using audio recordings collected with AudioMoth device. 
The project integrates data curation, neural network training, and embedded implementation of the trained model on an AudioMoth device.

---

## Repository Structure

### 1. MATLAB Folder
This folder contains the code for preparing the dataset, training the neural network, and testing the model. The primary files include:

- **`a_create_inputs_NN.m`**:  
  Reads audio segments from the dataset and generates feature vectors based on MFCC coefficients for training the neural network.

- **`b_trainNNmat.m`**:  
  Uses the feature vectors to train a shallow neural network.

- **`c_test.m`**:  
  Exemplifies the classification performance of the trained network with example data.

- **`finalNeuralNetworkFunction15khz.m`**:  
  Contains the trained neural network.

- **Auxiliary Scripts**:  
  Additional `.m` files provide support functions for data processing, feature extraction, and other utilities.

---

### 2. AudioMoth Folder
This folder contains the modified AudioMoth firmware based on the [Open Acoustic Devices AudioMoth Basic Firmware v1.3.0](https://github.com/OpenAcousticDevices/AudioMoth-Firmware-Basic/releases/tag/1.3.0). 
[Open Acoustic Devices code for AudioMoth Basic Firmware v1.3.0](https://github.com/OpenAcousticDevices/AudioMoth-Firmware-Basic/tree/1.3.0)

The modifications include:
- **`audiomoth.c`**: Core functionality remains unchanged.
- **`main.c`**: Updated to integrate the trained neural network for real-time classification of audio recordings.

The modified firmware enables the AudioMoth to:
1. Record audio.
2. Process audio segments using the embedded neural network.
3. Output species presence detection results.

---

## Usage

### MATLAB Workflow
1. **Generate Feature Vectors**  
   Run `a_create_inputs_NN.m` to process the dataset and create input feature vectors for training.

2. **Train the Neural Network**  
   Execute `b_trainNNmat.m` to train a shallow neural network. The trained model will be saved in `finalNeuralNetworkFunction15khz.m`.

3. **Test the Neural Network**  
   Use `c_test.m` to validate the network's performance on example audio segments.

### AudioMoth Integration
1. Replace the original `main.c` file from the AudioMoth firmware with the modified `main.c` provided in this repository.
2. Compile the firmware and flash it onto the AudioMoth device.
3. Deploy the device for field data collection and real-time species detection.

---

## Acknowledgements
- **AudioMoth Firmware**: Based on the [AudioMoth Basic Firmware v1.3.0](https://github.com/OpenAcousticDevices/AudioMoth-Firmware-Basic/releases/tag/1.3.0).
- **Open Acoustic Devices**: For their contributions to open-source acoustic monitoring technology.

