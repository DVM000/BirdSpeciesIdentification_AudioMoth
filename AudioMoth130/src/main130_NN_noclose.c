/****************************************************************************
 * main.c
 * openacousticdevices.info
 * June 2017
 *****************************************************************************/

#include <time.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
//#include <complex.h>
#include "ff.h"
//#include "ffconf.h"
#define __FPU_PRESENT       1    /*!< FPU present */
//#include "math.h"
#include "arm_math.h"

#include "audioMoth.h"

/* Sleep and LED constants */

#define DEFAULT_WAIT_INTERVAL               1

#define WAITING_LED_FLASH_INTERVAL          2
#define WAITING_LED_FLASH_DURATION          10

#define LOW_BATTERY_LED_FLASHES             10

#define SHORT_LED_FLASH_DURATION            100
#define LONG_LED_FLASH_DURATION             500

/* Useful time constants */

#define SECONDS_IN_MINUTE                   60
#define SECONDS_IN_HOUR                     (60 * SECONDS_IN_MINUTE)
#define SECONDS_IN_DAY                      (24 * SECONDS_IN_HOUR)

/* SRAM buffer constants (modified) */

#define NUMBER_OF_BUFFERS                   128 // (modified from 8)
#define EXTERNAL_SRAM_SIZE_IN_SAMPLES       (AM_EXTERNAL_SRAM_SIZE_IN_BYTES / 2)
#define NUMBER_OF_SAMPLES_IN_BUFFER         (EXTERNAL_SRAM_SIZE_IN_SAMPLES / NUMBER_OF_BUFFERS)
#define NUMBER_OF_SAMPLES_IN_DMA_TRANSFER   1024
#define NUMBER_OF_BUFFERS_TO_SKIP           1

// ---> Introduced constants:
#define NUMBER_OF_SUPERBUFFERS				8 // aggregation of 16 buffers
#define NUMBER_OF_BUFFERS_IN_SUPERBUFFER	(NUMBER_OF_BUFFERS / NUMBER_OF_SUPERBUFFERS)
#define NUMBER_OF_SAMPLES_IN_SUPERBUFFER	NUMBER_OF_SAMPLES_IN_BUFFER * NUMBER_OF_BUFFERS_IN_SUPERBUFFER

#define NUMBER_OF_BUFFERS_MFCC              5
#define NUMBER_OF_SAMPLES_IN_BUFFERS_MFCC	12
#define NBANKS								41
#define INIT_COUNTDOWN						1230
#define THRESHOLD_DETECTION					0.4f
#define DETECTION_VALUE						THRESHOLD_DETECTION * NUMBER_OF_BUFFERS_IN_SUPERBUFFER

#define MAX_INT_VALUE						32767
// <---

/* WAV header constant */

#define PCM_FORMAT                          1
#define RIFF_ID_LENGTH                      4
#define LENGTH_OF_ARTIST                    32
#define LENGTH_OF_COMMENT                   256

/* USB configuration constant */

#define MAX_START_STOP_PERIODS              5

/* DC filter constant */

#define DC_BLOCKING_FACTOR                  0.995f

/* Useful macros */

#define FLASH_LED(led, duration) { \
    AudioMoth_set ## led ## LED(true); \
    AudioMoth_delay(duration); \
    AudioMoth_set ## led ## LED(false); \
}

#define RETURN_ON_ERROR(fn) { \
    bool success = (fn); \
    if (success != true) { \
        if (configSettings->enableBatteryCheck ) { \
            AudioMoth_disableBatteryMonitor(); \
        } \
        FLASH_LED(Both, LONG_LED_FLASH_DURATION) \
        return SDCARD_WRITE_ERROR; \
    } \
}

#define SAVE_SWITCH_POSITION_AND_POWER_DOWN(duration) { \
    *previousSwitchPosition = switchPosition; \
    AudioMoth_powerDownAndWake(duration, true); \
}

#define MAX(a,b) (((a) > (b)) ? (a) : (b))

#define MIN(a,b) (((a) < (b)) ? (a) : (b))

/* Recording state enumeration */

typedef enum {RECORDING_OKAY, SWITCH_CHANGED, SDCARD_WRITE_ERROR, BATTERY_CHECK} AM_recordingState_t;

/* WAV header */

#pragma pack(push, 1)

typedef struct {
    char id[RIFF_ID_LENGTH];
    uint32_t size;
} chunk_t;

typedef struct {
    chunk_t icmt;
    char comment[LENGTH_OF_COMMENT];
} icmt_t;

typedef struct {
    chunk_t iart;
    char artist[LENGTH_OF_ARTIST];
} iart_t;

typedef struct {
    uint16_t format;
    uint16_t numberOfChannels;
    uint32_t samplesPerSecond;
    uint32_t bytesPerSecond;
    uint16_t bytesPerCapture;
    uint16_t bitsPerSample;
} wavFormat_t;

typedef struct {
    chunk_t riff;
    char format[RIFF_ID_LENGTH];
    chunk_t fmt;
    wavFormat_t wavFormat;
    chunk_t list;
    char info[RIFF_ID_LENGTH];
    icmt_t icmt;
    iart_t iart;
    chunk_t data;
} wavHeader_t;

#pragma pack(pop)

static wavHeader_t wavHeader = {
    .riff = {.id = "RIFF", .size = 0},
    .format = "WAVE",
    .fmt = {.id = "fmt ", .size = sizeof(wavFormat_t)},
    .wavFormat = {.format = PCM_FORMAT, .numberOfChannels = 1, .samplesPerSecond = 0, .bytesPerSecond = 0, .bytesPerCapture = 2, .bitsPerSample = 16},
    .list = {.id = "LIST", .size = RIFF_ID_LENGTH + sizeof(icmt_t) + sizeof(iart_t)},
    .info = "INFO",
    .icmt = {.icmt.id = "ICMT", .icmt.size = LENGTH_OF_COMMENT, .comment = ""},
    .iart = {.iart.id = "IART", .iart.size = LENGTH_OF_ARTIST, .artist = ""},
    .data = {.id = "data", .size = 0}
};


/* ---> Introduced: for Lesser Kestrel recognition */
static float32_t* buffersMFCC[NUMBER_OF_BUFFERS_MFCC];
static float32_t MK4[NUMBER_OF_SAMPLES_IN_BUFFERS_MFCC];
static float32_t MK3[NUMBER_OF_SAMPLES_IN_BUFFERS_MFCC];
static float32_t MK1[NUMBER_OF_SAMPLES_IN_BUFFERS_MFCC];
static float32_t MK0[NUMBER_OF_SAMPLES_IN_BUFFERS_MFCC];

static void MFCC(int16_t *bufferIN, float32_t *bufferOUT, uint32_t readBuffer);
static void DCTII(float32_t *in, float32_t *out);
static void deltas(float32_t **buffers);
static float32_t neuralNetwork(float32_t *bufferMFCC);

arm_rfft_fast_instance_f32 realFFTinstance;
// <---

void setHeaderDetails(uint32_t sampleRate, uint32_t numberOfSamples) {

    wavHeader.wavFormat.samplesPerSecond = sampleRate;
    wavHeader.wavFormat.bytesPerSecond = 2 * sampleRate;
    wavHeader.data.size = 2 * numberOfSamples;
    wavHeader.riff.size = 2 * numberOfSamples + sizeof(wavHeader_t) - sizeof(chunk_t);

}

void setHeaderComment(uint32_t currentTime, int8_t timezoneHours, int8_t timezoneMinutes, uint8_t *serialNumber, uint32_t gain, AM_batteryState_t batteryState, bool batteryVoltageLow, bool switchPositionChanged) {

    time_t rawtime = currentTime + timezoneHours * SECONDS_IN_HOUR + timezoneMinutes * SECONDS_IN_MINUTE;

    struct tm *time = gmtime(&rawtime);

    /* Format artist field */

    char *artist = wavHeader.iart.artist;

    sprintf(artist, "AudioMoth %08X%08X", (unsigned int)*((uint32_t*)serialNumber + 1), (unsigned int)*((uint32_t*)serialNumber));

    /* Format comment field */

    char *comment = wavHeader.icmt.comment;

    sprintf(comment, "Recorded at %02d:%02d:%02d %02d/%02d/%04d (UTC", time->tm_hour, time->tm_min, time->tm_sec, time->tm_mday, 1 + time->tm_mon, 1900 + time->tm_year);

    comment += 36;

    if (timezoneHours < 0) sprintf(comment, "%d", timezoneHours);

    if (timezoneHours > 0) sprintf(comment, "+%d", timezoneHours);

    if (timezoneHours < 0 || timezoneHours > 0) comment += 2;

    if (timezoneHours < -9 || timezoneHours > 9) comment += 1;

    if (timezoneMinutes < 0) sprintf(comment, ":%2d", -timezoneMinutes);

    if (timezoneMinutes > 0) sprintf(comment, ":%2d", timezoneMinutes);

    if (timezoneMinutes < 0 || timezoneMinutes > 0) comment += 3;

    sprintf(comment, ") by %s at gain setting %d while battery state was ", artist, (unsigned int)gain);

    comment += 74;

    if (batteryState == AM_BATTERY_LOW) {

        sprintf(comment, "less than 3.6V.");

        comment += 15;

    } else if (batteryState >= AM_BATTERY_FULL) {

        sprintf(comment, "greater than 4.9V.");

        comment += 18;

    } else {

        batteryState += 35;

        sprintf(comment, "%01d.%01dV.", batteryState / 10, batteryState % 10);

        comment += 5;

    }

    if (batteryVoltageLow || switchPositionChanged) {

        sprintf(comment, " Recording cancelled before completion due to ");

        comment += 46;

        if (batteryVoltageLow) {

            sprintf(comment, "low battery voltage.");

        } else if (switchPositionChanged) {

            sprintf(comment, "change of switch position.");

        }

    }

}

/* USB configuration data structure */

#pragma pack(push, 1)

typedef struct {
    uint16_t startMinutes;
    uint16_t stopMinutes;
} startStopPeriod_t;

typedef struct {
    uint32_t time;
    uint8_t gain;
    uint8_t clockDivider;
    uint8_t acquisitionCycles;
    uint8_t oversampleRate;
    uint32_t sampleRate;
    uint8_t sampleRateDivider;
    uint16_t sleepDuration;
    uint16_t recordDuration;
    uint8_t enableLED;
    uint8_t activeStartStopPeriods;
    startStopPeriod_t startStopPeriods[MAX_START_STOP_PERIODS];
    int8_t timezoneHours;
    uint8_t enableBatteryCheck;
    uint8_t disableBatteryLevelDisplay;
    int8_t timezoneMinutes;
} configSettings_t;

#pragma pack(pop)

static const configSettings_t defaultConfigSettings = {
    .time = 0,
    .gain = 2,
    .clockDivider = 4,
    .acquisitionCycles = 16,
    .oversampleRate = 1,
    //.sampleRate = 384000, // corresponding to 48kHz, modified
    .sampleRate = 256000,  // corresponding to 32kHz
    .sampleRateDivider = 8,
    .sleepDuration = 0,
    .recordDuration = 60, 
    .enableLED = 1,
    .activeStartStopPeriods = 0,
    .startStopPeriods = {
        {.startMinutes = 60, .stopMinutes = 120},
        {.startMinutes = 300, .stopMinutes = 420},
        {.startMinutes = 540, .stopMinutes = 600},
        {.startMinutes = 720, .stopMinutes = 780},
        {.startMinutes = 900, .stopMinutes = 960}
    },
    .timezoneHours = 0,
    .enableBatteryCheck = 0,
    .disableBatteryLevelDisplay = 0,
    .timezoneMinutes = 0
};

static uint32_t *previousSwitchPosition = (uint32_t*)AM_BACKUP_DOMAIN_START_ADDRESS;

static uint32_t *timeOfNextRecording = (uint32_t*)(AM_BACKUP_DOMAIN_START_ADDRESS + 4);

static uint32_t *durationOfNextRecording = (uint32_t*)(AM_BACKUP_DOMAIN_START_ADDRESS + 8);

static configSettings_t *configSettings = (configSettings_t*)(AM_BACKUP_DOMAIN_START_ADDRESS + 12);

/* DC filter variables */

static int8_t bitsToShift;

static int32_t previousSample;
static int32_t previousFilterOutput;

/* SRAM buffer variables */

static volatile uint32_t writeBuffer;
static volatile uint32_t writeBufferIndex;

static int16_t* buffers[NUMBER_OF_BUFFERS];

/* Recording state */

static volatile bool switchPositionChanged;

/* DMA buffers */

static int16_t primaryBuffer[NUMBER_OF_SAMPLES_IN_DMA_TRANSFER];
static int16_t secondaryBuffer[NUMBER_OF_SAMPLES_IN_DMA_TRANSFER];

/* Current recording file name */

static char fileName[20];

/* Firmware version and description */

static uint8_t firmwareVersion[AM_FIRMWARE_VERSION_LENGTH] = {1, 3, 0};

static uint8_t firmwareDescription[AM_FIRMWARE_DESCRIPTION_LENGTH] = "AudioMoth-Firmware-Basic";

/* Function prototypes */

static void flashLedToIndicateBatteryLife(void);
static void filter(int16_t *source, int16_t *dest, uint8_t sampleRateDivider, uint32_t size);
static void scheduleRecording(uint32_t currentTime, uint32_t *timeOfNextRecording, uint32_t *durationOfNextRecording);
static AM_recordingState_t makeRecording(uint32_t currentTime, uint32_t recordDuration, bool enableLED, AM_batteryState_t batteryState);

/* Functions of copy to and from the backup domain */

static void copyFromBackupDomain(uint8_t *dst, uint32_t *src, uint32_t length) {

    for (uint32_t i = 0; i < length; i += 1) {
        *(dst + i) = *((uint8_t*)src + i);
    }

}

static void copyToBackupDomain(uint32_t *dst, uint8_t *src, uint32_t length) {

    uint32_t value = 0;

    for (uint32_t i = 0; i < length / 4; i += 1) {
        *(dst + i) = *((uint32_t*)src + i);
    }

    for (uint32_t i = 0; i < length % 4; i += 1) {
        value = (value << 8) + *(src + length - 1 - i);
    }

    *(dst + length / 4) = value;

}

/* Main function */

int main(void) {

    /* Initialise device */

    AudioMoth_initialise();

    AM_switchPosition_t switchPosition = AudioMoth_getSwitchPosition();

    if (AudioMoth_isInitialPowerUp()) {

        *timeOfNextRecording = 0;

        *durationOfNextRecording = 0;

        *previousSwitchPosition = AM_SWITCH_NONE;

        copyToBackupDomain((uint32_t*)configSettings, (uint8_t*)&defaultConfigSettings, sizeof(configSettings_t));

    } else {

        /* Indicate battery state is not initial power up and switch has been moved into USB if enabled */

        if (switchPosition != *previousSwitchPosition && switchPosition == AM_SWITCH_USB && !configSettings->disableBatteryLevelDisplay) {

            flashLedToIndicateBatteryLife();

        }

    }

    /* Handle the case that the switch is in USB position  */

    if (switchPosition == AM_SWITCH_USB) {

        AudioMoth_handleUSB();

        SAVE_SWITCH_POSITION_AND_POWER_DOWN(DEFAULT_WAIT_INTERVAL);

    }

    /* Handle the case that the switch is in CUSTOM position but the time has not been set */

    if (switchPosition == AM_SWITCH_CUSTOM && (AudioMoth_hasTimeBeenSet() == false || configSettings->activeStartStopPeriods == 0)) {

        FLASH_LED(Both, SHORT_LED_FLASH_DURATION)

        SAVE_SWITCH_POSITION_AND_POWER_DOWN(DEFAULT_WAIT_INTERVAL);

    }

    /* Calculate time of next recording if switch has changed position */

    uint32_t currentTime;

    AudioMoth_getTime(&currentTime, NULL);

    if (switchPosition != *previousSwitchPosition) {

         if (switchPosition == AM_SWITCH_DEFAULT) {

             /* Set parameters to start recording now */

             *timeOfNextRecording = currentTime;

             *durationOfNextRecording = configSettings->recordDuration;

         } else {

             /* Determine starting time and duration of next recording */

             scheduleRecording(currentTime, timeOfNextRecording, durationOfNextRecording);

         }

    }

    /* Make recording if appropriate */

    bool enableLED = (switchPosition == AM_SWITCH_DEFAULT) || configSettings->enableLED;

    if (currentTime >= *timeOfNextRecording) {

        /* Make recording is battery check is disabled or enabled and okay */

        AM_recordingState_t recordingState = RECORDING_OKAY;

        AM_batteryState_t batteryState = AudioMoth_getBatteryState();

        if (!configSettings->enableBatteryCheck || batteryState > AM_BATTERY_LOW) {

            recordingState = makeRecording(currentTime, *durationOfNextRecording, enableLED, batteryState);

        } else if (enableLED) {

            FLASH_LED(Both, LONG_LED_FLASH_DURATION);

        }

        /* Schedule next recording */

		if (switchPosition == AM_SWITCH_DEFAULT) {

			/* Set parameters to start recording after sleep period */

			if (recordingState != SWITCH_CHANGED) {

				*timeOfNextRecording = currentTime + configSettings->recordDuration + configSettings->sleepDuration;

			}

		} else {

			/* Determine starting time and duration of next recording */

			scheduleRecording(currentTime, timeOfNextRecording, durationOfNextRecording);

		}

    } else if (enableLED) {

        /* Flash LED to indicate waiting */

        FLASH_LED(Green, WAITING_LED_FLASH_DURATION);

    }

    /* Determine how long to power down */

    uint32_t secondsToSleep = 0;

    if (*timeOfNextRecording > currentTime) {

        secondsToSleep = MIN(*timeOfNextRecording - currentTime, WAITING_LED_FLASH_INTERVAL);

    }

    /* Power down */

    SAVE_SWITCH_POSITION_AND_POWER_DOWN(secondsToSleep);

}

/* Time zone handler */

inline void AudioMoth_timezoneRequested(int8_t *timezoneHours, int8_t *timezoneMinutes) {

    *timezoneHours = configSettings->timezoneHours;

    *timezoneMinutes = configSettings->timezoneMinutes;

}


/* AudioMoth interrupt handlers */

inline void AudioMoth_handleSwitchInterrupt() {

    switchPositionChanged = true;

}

inline void AudioMoth_handleMicrophoneInterrupt(int16_t sample) { }

inline void AudioMoth_handleDirectMemoryAccessInterrupt(bool isPrimaryBuffer, int16_t **nextBuffer) {

    int16_t *source = secondaryBuffer;

    if (isPrimaryBuffer) source = primaryBuffer;

    /* Update the current buffer index and write buffer */

    filter(source, buffers[writeBuffer] + writeBufferIndex, configSettings->sampleRateDivider, NUMBER_OF_SAMPLES_IN_DMA_TRANSFER);

    writeBufferIndex += NUMBER_OF_SAMPLES_IN_DMA_TRANSFER / configSettings->sampleRateDivider;

    if (writeBufferIndex == NUMBER_OF_SAMPLES_IN_BUFFER) {

        writeBufferIndex = 0;

        writeBuffer = (writeBuffer + 1) & (NUMBER_OF_BUFFERS - 1);

    }

}

/* AudioMoth USB message handlers */

inline void AudioMoth_usbFirmwareVersionRequested(uint8_t **firmwareVersionPtr) {

    *firmwareVersionPtr = firmwareVersion;

}

inline void AudioMoth_usbFirmwareDescriptionRequested(uint8_t **firmwareDescriptionPtr) {

    *firmwareDescriptionPtr = firmwareDescription;

}

inline void AudioMoth_usbApplicationPacketRequested(uint32_t messageType, uint8_t *transmitBuffer, uint32_t size) {

    /* Copy the current time to the USB packet */

    uint32_t currentTime;

    AudioMoth_getTime(&currentTime, NULL);

    memcpy(transmitBuffer + 1, &currentTime, 4);

    /* Copy the unique ID to the USB packet */

    memcpy(transmitBuffer + 5, (uint8_t*)AM_UNIQUE_ID_START_ADDRESS, AM_UNIQUE_ID_SIZE_IN_BYTES);

    /* Copy the battery state to the USB packet */

    AM_batteryState_t batteryState = AudioMoth_getBatteryState();

    memcpy(transmitBuffer + 5 + AM_UNIQUE_ID_SIZE_IN_BYTES, &batteryState, 1);

    /* Copy the firmware version to the USB packet */

    memcpy(transmitBuffer + 6 + AM_UNIQUE_ID_SIZE_IN_BYTES, firmwareVersion, AM_FIRMWARE_VERSION_LENGTH);

    /* Copy the firmware description to the USB packet */

    memcpy(transmitBuffer + 6 + AM_UNIQUE_ID_SIZE_IN_BYTES + AM_FIRMWARE_VERSION_LENGTH, firmwareDescription, AM_FIRMWARE_DESCRIPTION_LENGTH);

}

inline void AudioMoth_usbApplicationPacketReceived(uint32_t messageType, uint8_t* receiveBuffer, uint8_t *transmitBuffer, uint32_t size) {

    /* Copy the USB packet contents to the back-up register data structure location */

    copyToBackupDomain((uint32_t*)configSettings,  receiveBuffer + 1, sizeof(configSettings_t));

    /* Copy the back-up register data structure to the USB packet */

    copyFromBackupDomain(transmitBuffer + 1, (uint32_t*)configSettings, sizeof(configSettings_t));

    /* Set the time */

    AudioMoth_setTime(configSettings->time, 0);

}

/* Remove DC offset from the microphone samples */

static void filter(int16_t *source, int16_t *dest, uint8_t sampleRateDivider, uint32_t size) {

    int32_t filteredOutput;
    int32_t scaledPreviousFilterOutput;

    int index = 0;

    for (int i = 0; i < size; i += sampleRateDivider) {

        int32_t sample = 0;

        for (int j = 0; j < sampleRateDivider; j += 1) {

            sample += (int32_t)source[i + j];

        }

        if (bitsToShift > 0) sample <<= bitsToShift;

        if (bitsToShift < 0) sample >>= -bitsToShift;

        scaledPreviousFilterOutput = (int32_t)(DC_BLOCKING_FACTOR * (float)previousFilterOutput);

        filteredOutput = sample - previousSample + scaledPreviousFilterOutput;

        if (filteredOutput > INT16_MAX) {

            dest[index++] = INT16_MAX;

        } else if (filteredOutput < INT16_MIN) {

            dest[index++] = INT16_MIN;

        } else {

            dest[index++] = (int16_t)filteredOutput;

        }

        previousFilterOutput = filteredOutput;

        previousSample = sample;

    }

}

/* Save recording to SD card */

static AM_recordingState_t makeRecording(uint32_t currentTime, uint32_t recordDuration, bool enableLED, AM_batteryState_t batteryState) {

    /* Initialise microphone for recording */

    AudioMoth_enableExternalSRAM();

    AudioMoth_enableMicrophone(configSettings->gain, configSettings->clockDivider, configSettings->acquisitionCycles, configSettings->oversampleRate);

    AudioMoth_initialiseDirectMemoryAccess(primaryBuffer, secondaryBuffer, NUMBER_OF_SAMPLES_IN_DMA_TRANSFER);

    AudioMoth_startMicrophoneSamples(configSettings->sampleRate);

    RETURN_ON_ERROR(AudioMoth_enableFileSystem());

    /* Initialise buffers */

    writeBuffer = 0;

    writeBufferIndex = 0;

    buffers[0] = (int16_t*)AM_EXTERNAL_SRAM_START_ADDRESS;

    for (int i = 1; i < NUMBER_OF_BUFFERS; i += 1) {
        buffers[i] = buffers[i - 1] + NUMBER_OF_SAMPLES_IN_BUFFER;
    }

    // ---> Introduced: 
    buffersMFCC[0] = (float32_t*)malloc(sizeof(float32_t) * NUMBER_OF_SAMPLES_IN_BUFFERS_MFCC);
    for (int i = 1; i < NUMBER_OF_BUFFERS_MFCC; i += 1) {
		if (i==3){
			buffersMFCC[i] = buffersMFCC[i - 1] + 2*NUMBER_OF_SAMPLES_IN_BUFFERS_MFCC;
		}
		else {
			buffersMFCC[i] = buffersMFCC[i - 1] + NUMBER_OF_SAMPLES_IN_BUFFERS_MFCC;
		}
	}
    // <---

    /* Calculate the bits to shift */

    bitsToShift = 0;

    uint16_t oversampling = configSettings->oversampleRate * configSettings->sampleRateDivider;

    while (oversampling > 16) {
        oversampling >>= 1;
        bitsToShift -= 1;
    }

    while (oversampling < 16) {
        oversampling <<= 1;
        bitsToShift += 1;
    }

    /* Calculate recording parameters */

    uint32_t numberOfSamplesInHeader = sizeof(wavHeader) >> 1;

    uint32_t numberOfSamples = configSettings->sampleRate / configSettings->sampleRateDivider * recordDuration;

    /* Enable the battery monitor */

    if (configSettings->enableBatteryCheck) {

        AudioMoth_enableBatteryMonitor();

        AudioMoth_setBatteryMonitorThreshold(AM_BATTERY_LOW);

    }


    /* Initialize file system and open a new file */
   
    if (enableLED) {

        //AudioMoth_setRedLED(true); // modified

    }


    /* Open a file with the current local time as the name */

    time_t rawtime = currentTime + configSettings->timezoneHours * SECONDS_IN_HOUR + configSettings->timezoneMinutes * SECONDS_IN_MINUTE;

    struct tm *time = gmtime(&rawtime);

    sprintf(fileName, "%04d%02d%02d_%02d%02d%02d.WAV", 1900 + time->tm_year, time->tm_mon + 1, time->tm_mday, time->tm_hour, time->tm_min, time->tm_sec);

    RETURN_ON_ERROR(AudioMoth_openFile(fileName));

    //RETURN_ON_ERROR(AudioMoth_closeFile(fileName)); // introduced

    AudioMoth_setRedLED(false);

    /* Termination conditions */

    switchPositionChanged = false;

    bool batteryVoltageLow = false;

    /* Main record loop */

    uint32_t samplesWritten = 0;

    uint32_t buffersProcessed = 0;

    uint32_t readBuffer = writeBuffer;

    float32_t keep_prob = 0;  // Introduced

    while (samplesWritten < numberOfSamples + numberOfSamplesInHeader && !switchPositionChanged /*&& !batteryVoltageLow*/) {

        while (readBuffer != writeBuffer && samplesWritten < numberOfSamples + numberOfSamplesInHeader && !switchPositionChanged /*&& !batteryVoltageLow*/) {

            /* --> Introduced code: MFCC and Neural Network */
            // Shift buffers to the left 
	    for (int j = 0; j <NUMBER_OF_SAMPLES_IN_BUFFERS_MFCC; j+= 1){
				*(buffersMFCC[0]+j) = *(buffersMFCC[1]+j);
				*(buffersMFCC[1]+j) = *(buffersMFCC[2]+j);
				*(buffersMFCC[2]+j) = *(buffersMFCC[3]+j);
				*(buffersMFCC[3]+j) = *(buffersMFCC[NUMBER_OF_BUFFERS_MFCC-1]+j);
	    }
			
	    //Calculate MFCCs corresponding buffer
	    MFCC(buffers[readBuffer], buffersMFCC[NUMBER_OF_BUFFERS_MFCC-1], readBuffer);

	    //Calculate deltas
	    deltas(buffersMFCC);

	    //Apply neural network
	    keep_prob += neuralNetwork(buffersMFCC[2]);
	    // <---


	    /* Write the appropriate number of bytes to the SD card */

            uint32_t numberOfSamplesToWrite = 0;

            //if (buffersProcessed >= NUMBER_OF_BUFFERS_TO_SKIP) {
                //numberOfSamplesToWrite = MIN(numberOfSamples + numberOfSamplesInHeader - samplesWritten, NUMBER_OF_SAMPLES_IN_BUFFER);
                // modified: write complete superbuffer
                numberOfSamplesToWrite = MIN(numberOfSamples + numberOfSamplesInHeader - samplesWritten, NUMBER_OF_BUFFERS_IN_SUPERBUFFER * NUMBER_OF_SAMPLES_IN_BUFFER);

            //}

           // --> Introduced code:
	   if ((readBuffer+1) % NUMBER_OF_BUFFERS_IN_SUPERBUFFER == 0){ // for each superbuffer

				AudioMoth_setRedLED(true);

				//RETURN_ON_ERROR(AudioMoth_appendFile(fileName));
				RETURN_ON_ERROR(AudioMoth_writeToFile(buffers[readBuffer-NUMBER_OF_BUFFERS_IN_SUPERBUFFER+1], 2 * numberOfSamplesToWrite)); // modified. write complete superbuffer
				//RETURN_ON_ERROR(AudioMoth_writeToFile(buffers[readBuffer], 2 * numberOfSamplesToWrite));
				//RETURN_ON_ERROR(AudioMoth_closeFile());

				/* Increment buffer counters */
				
				//readBuffer = (readBuffer + 1) & (NUMBER_OF_BUFFERS - 1);

				samplesWritten += numberOfSamplesToWrite;

				buffersProcessed += 1;

                                // Log detections if probability exceeds threshold
				if (keep_prob > (float32_t)(DETECTION_VALUE)){

					FIL callfile; //File to keep detections
					f_open(&callfile,"calls.txt", FA_OPEN_APPEND | FA_WRITE);
					uint32_t currentTime;
					AudioMoth_getTime(&currentTime, NULL);
					time_t rawtime = currentTime + configSettings->timezoneHours * 3600 + configSettings->timezoneMinutes * 60;
					struct tm *time = gmtime(&rawtime);
					char str[21];
					sprintf(str, "%04d/%02d/%02d %02d:%02d:%02d \n", 1900 + time->tm_year, time->tm_mon + 1, time->tm_mday, time->tm_hour, time->tm_min, time->tm_sec);
					f_puts(str,&callfile);
					f_close(&callfile);

					AudioMoth_setGreenLED(true);

				} else {
					AudioMoth_setGreenLED(false);
				}
				keep_prob = 0;

            }
            // <---
            
            /* Increment buffer counters */
            
	    readBuffer = (readBuffer + 1) & (NUMBER_OF_BUFFERS - 1);
	    
	    //samplesWritten += numberOfSamplesToWrite; // modified. Already incremented

	    //buffersProcessed += 1; // modified. Already incremented
	    

            /* Clear LED */

            AudioMoth_setRedLED(false);


        }

        /* Check the battery level */

        if (configSettings->enableBatteryCheck && !AudioMoth_isBatteryMonitorAboveThreshold()) {

            batteryVoltageLow = true;

        }

        /* Sleep until next DMA transfer is complete */

        AudioMoth_sleep();

    }

    /* Disable battery check */

    if (configSettings->enableBatteryCheck ) {

	AudioMoth_disableBatteryMonitor();

    }

    /* Initialise the WAV header */

    samplesWritten = MAX(numberOfSamplesInHeader, samplesWritten);

    setHeaderDetails(configSettings->sampleRate / configSettings->sampleRateDivider, samplesWritten - numberOfSamplesInHeader);

    setHeaderComment(currentTime, configSettings->timezoneHours, configSettings->timezoneMinutes, (uint8_t*)AM_UNIQUE_ID_START_ADDRESS, configSettings->gain, batteryState, batteryVoltageLow, switchPositionChanged);

    /* Write the header */

    if (enableLED) {

        AudioMoth_setRedLED(true);

    }


    //RETURN_ON_ERROR(AudioMoth_appendFile(fileName)); // introduced
    
    RETURN_ON_ERROR(AudioMoth_seekInFile(0));
    
    RETURN_ON_ERROR(AudioMoth_writeToFile(&wavHeader, sizeof(wavHeader)));

    /* Close the file */

    RETURN_ON_ERROR(AudioMoth_closeFile());

    AudioMoth_setRedLED(false);

    /* Return with state */

    if (batteryVoltageLow) return BATTERY_CHECK;

    if (switchPositionChanged) return SWITCH_CHANGED;

    return RECORDING_OKAY;

}

static void scheduleRecording(uint32_t currentTime, uint32_t *timeOfNextRecording, uint32_t *durationOfNextRecording) {

    /* Check number of active state stop periods */

    uint32_t activeStartStopPeriods = MIN(configSettings->activeStartStopPeriods, MAX_START_STOP_PERIODS);

    /* No active periods */

    if (activeStartStopPeriods == 0) {

        *timeOfNextRecording = UINT32_MAX;

        *durationOfNextRecording = configSettings->recordDuration;

        return;

    }

    /* Calculate the number of seconds of this day */

    time_t rawtime = currentTime;

    struct tm *time = gmtime(&rawtime);

    uint32_t currentSeconds = SECONDS_IN_HOUR * time->tm_hour + SECONDS_IN_MINUTE * time->tm_min + time->tm_sec;

    /* Check each active start stop period */

    uint32_t durationOfCycle = configSettings->recordDuration + configSettings->sleepDuration;

    for (uint32_t i = 0; i < activeStartStopPeriods; i += 1) {

        startStopPeriod_t *period = configSettings->startStopPeriods + i;

        /* Calculate the start and stop time of the current period */

        uint32_t startSeconds = SECONDS_IN_MINUTE * period->startMinutes;

        uint32_t stopSeconds = SECONDS_IN_MINUTE * period->stopMinutes;

        /* Calculate time to next period or time to next start in this period */

        if (currentSeconds < startSeconds) {

            *timeOfNextRecording = currentTime + (startSeconds - currentSeconds);

            *durationOfNextRecording = MIN(configSettings->recordDuration, stopSeconds - startSeconds);

            return;

        } else if (currentSeconds < stopSeconds) {

            uint32_t cycles = (currentSeconds - startSeconds + durationOfCycle) / durationOfCycle;

            uint32_t secondsFromStartOfPeriod = cycles * durationOfCycle;

            if (secondsFromStartOfPeriod < stopSeconds - startSeconds) {

                *timeOfNextRecording = currentTime + (startSeconds - currentSeconds) + secondsFromStartOfPeriod;

                *durationOfNextRecording = MIN(configSettings->recordDuration, stopSeconds - startSeconds - secondsFromStartOfPeriod);

                return;

            }

        }

    }

    /* Calculate time until first period tomorrow */

    startStopPeriod_t *firstPeriod = configSettings->startStopPeriods;

    uint32_t startSeconds = SECONDS_IN_MINUTE * firstPeriod->startMinutes;

    uint32_t stopSeconds = SECONDS_IN_MINUTE * firstPeriod->stopMinutes;

    *timeOfNextRecording = currentTime + (SECONDS_IN_DAY - currentSeconds) + startSeconds;

    *durationOfNextRecording = MIN(configSettings->recordDuration, stopSeconds - startSeconds);

}

static void flashLedToIndicateBatteryLife(void){

    uint32_t numberOfFlashes = LOW_BATTERY_LED_FLASHES;

    AM_batteryState_t batteryState = AudioMoth_getBatteryState();

    /* Set number of flashes according to battery state */

    if (batteryState > AM_BATTERY_LOW) {

        numberOfFlashes = (batteryState >= AM_BATTERY_4V6) ? 4 : (batteryState >= AM_BATTERY_4V4) ? 3 : (batteryState >= AM_BATTERY_4V0) ? 2 : 1;

    }

    /* Flash LED */

    for (uint32_t i = 0; i < numberOfFlashes; i += 1) {

        FLASH_LED(Red, SHORT_LED_FLASH_DURATION)

        if (numberOfFlashes == LOW_BATTERY_LED_FLASHES) {

            AudioMoth_delay(SHORT_LED_FLASH_DURATION);

        } else {

            AudioMoth_delay(LONG_LED_FLASH_DURATION);

        }

    }

}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


/* FUNCTIONS NEEDED IN makeRecording THAT PERFORM MFCC EXTRACTION AND NEURAL NETWORK PROCESSING */

/* 
 * Function: MFCC
 * Purpose: Compute Mel Frequency Cepstral Coefficients (MFCCs) from the input audio buffer.
 * 
 * Steps:
 * 1. Apply a Hamming window to the input audio samples.
 * 2. Perform an FFT to compute the frequency domain representation of the audio.
 * 3. Apply Mel filter banks to extract relevant frequency features.
 * 4. Log-transform the energy values of the filtered frequencies.
 * 5. Perform a Discrete Cosine Transform (DCT) to obtain MFCCs.

 *
 * Parameters:
 *  - bufferIN: Pointer to input audio samples.
 *  - bufferOUT: Pointer to store the calculated MFCCs.
 *  - readBuffer: Index of the buffer being processed.
 */
static void MFCC(int16_t *bufferIN, float32_t *bufferOUT, uint32_t readBuffer){

	// 1. Apply hamming window
	static const float32_t hamming_window[NUMBER_OF_SAMPLES_IN_BUFFER] = {0.080000,0.080009,0.080035,0.080078,0.080139,0.080217,0.080312,0.080425,0.080555,0.080703,0.080867,0.081049,0.081249,0.081466,0.081700,0.081951,0.082219,0.082505,0.082808,0.083129,0.083466,0.083821,0.084193,0.084582,0.084989,0.085412,0.085853,0.086311,0.086785,0.087278,0.087787,0.088313,0.088856,0.089416,0.089993,0.090588,0.091199,0.091827,0.092472,0.093134,0.093812,0.094508,0.095220,0.095950,0.096695,0.097458,0.098237,0.099033,0.099846,0.100675,0.101521,0.102383,0.103262,0.104157,0.105069,0.105997,0.106942,0.107903,0.108880,0.109873,0.110883,0.111909,0.112951,0.114009,0.115083,0.116173,0.117279,0.118402,0.119540,0.120693,0.121863,0.123049,0.124250,0.125467,0.126699,0.127947,0.129211,0.130490,0.131785,0.133095,0.134420,0.135761,0.137117,0.138488,0.139874,0.141276,0.142692,0.144123,0.145570,0.147031,0.148507,0.149998,0.151503,0.153023,0.154558,0.156107,0.157671,0.159249,0.160842,0.162449,0.164070,0.165705,0.167355,0.169018,0.170696,0.172387,0.174092,0.175811,0.177544,0.179291,0.181051,0.182824,0.184611,0.186412,0.188226,0.190053,0.191893,0.193747,0.195613,0.197493,0.199385,0.201290,0.203208,0.205139,0.207082,0.209038,0.211007,0.212988,0.214981,0.216986,0.219004,0.221033,0.223075,0.225129,0.227195,0.229272,0.231361,0.233462,0.235574,0.237698,0.239833,0.241980,0.244137,0.246306,0.248486,0.250677,0.252879,0.255092,0.257315,0.259550,0.261794,0.264050,0.266315,0.268591,0.270877,0.273174,0.275480,0.277797,0.280123,0.282459,0.284805,0.287160,0.289525,0.291900,0.294283,0.296677,0.299079,0.301490,0.303910,0.306340,0.308778,0.311224,0.313680,0.316144,0.318616,0.321097,0.323586,0.326083,0.328588,0.331101,0.333623,0.336151,0.338688,0.341232,0.343784,0.346343,0.348909,0.351483,0.354063,0.356651,0.359246,0.361847,0.364455,0.367070,0.369691,0.372319,0.374953,0.377593,0.380240,0.382892,0.385550,0.388215,0.390884,0.393560,0.396241,0.398927,0.401619,0.404316,0.407018,0.409725,0.412438,0.415154,0.417876,0.420602,0.423333,0.426068,0.428807,0.431551,0.434299,0.437050,0.439806,0.442565,0.445328,0.448095,0.450865,0.453638,0.456415,0.459195,0.461977,0.464763,0.467552,0.470343,0.473137,0.475934,0.478733,0.481534,0.484337,0.487143,0.489950,0.492760,0.495571,0.498384,0.501199,0.504014,0.506832,0.509650,0.512470,0.515291,0.518112,0.520935,0.523758,0.526582,0.529406,0.532231,0.535056,0.537881,0.540706,0.543532,0.546357,0.549182,0.552006,0.554830,0.557654,0.560477,0.563299,0.566120,0.568940,0.571759,0.574577,0.577394,0.580209,0.583023,0.585835,0.588645,0.591454,0.594260,0.597065,0.599867,0.602667,0.605465,0.608260,0.611053,0.613843,0.616630,0.619414,0.622196,0.624974,0.627749,0.630521,0.633289,0.636054,0.638815,0.641572,0.644326,0.647076,0.649821,0.652563,0.655300,0.658033,0.660762,0.663485,0.666205,0.668919,0.671629,0.674333,0.677033,0.679727,0.682416,0.685100,0.687779,0.690451,0.693118,0.695780,0.698435,0.701084,0.703728,0.706365,0.708996,0.711620,0.714238,0.716850,0.719455,0.722053,0.724644,0.727228,0.729805,0.732375,0.734938,0.737493,0.740041,0.742581,0.745114,0.747639,0.750156,0.752665,0.755167,0.757660,0.760145,0.762621,0.765089,0.767549,0.770000,0.772442,0.774876,0.777301,0.779717,0.782123,0.784521,0.786910,0.789289,0.791658,0.794019,0.796369,0.798710,0.801041,0.803363,0.805674,0.807976,0.810267,0.812548,0.814819,0.817079,0.819329,0.821569,0.823798,0.826016,0.828223,0.830420,0.832605,0.834780,0.836943,0.839095,0.841236,0.843365,0.845484,0.847590,0.849685,0.851768,0.853840,0.855899,0.857947,0.859983,0.862007,0.864018,0.866017,0.868004,0.869979,0.871941,0.873891,0.875828,0.877752,0.879664,0.881563,0.883449,0.885322,0.887182,0.889029,0.890862,0.892683,0.894490,0.896284,0.898064,0.899831,0.901584,0.903324,0.905050,0.906762,0.908461,0.910145,0.911815,0.913472,0.915114,0.916742,0.918356,0.919956,0.921542,0.923112,0.924669,0.926211,0.927739,0.929251,0.930750,0.932233,0.933702,0.935155,0.936594,0.938018,0.939427,0.940821,0.942199,0.943563,0.944911,0.946244,0.947562,0.948864,0.950151,0.951423,0.952679,0.953919,0.955144,0.956353,0.957546,0.958724,0.959885,0.961031,0.962162,0.963276,0.964374,0.965456,0.966522,0.967572,0.968606,0.969624,0.970625,0.971611,0.972580,0.973533,0.974469,0.975389,0.976292,0.977179,0.978050,0.978904,0.979742,0.980562,0.981367,0.982154,0.982925,0.983680,0.984417,0.985138,0.985842,0.986529,0.987199,0.987853,0.988489,0.989109,0.989712,0.990297,0.990866,0.991418,0.991952,0.992470,0.992971,0.993454,0.993920,0.994370,0.994802,0.995217,0.995615,0.995995,0.996359,0.996705,0.997034,0.997345,0.997640,0.997917,0.998177,0.998420,0.998645,0.998853,0.999044,0.999217,0.999373,0.999512,0.999633,0.999738,0.999824,0.999894,0.999946,0.999980,0.999998,0.999998,0.999980,0.999946,0.999894,0.999824,0.999738,0.999633,0.999512,0.999373,0.999217,0.999044,0.998853,0.998645,0.998420,0.998177,0.997917,0.997640,0.997345,0.997034,0.996705,0.996359,0.995995,0.995615,0.995217,0.994802,0.994370,0.993920,0.993454,0.992971,0.992470,0.991952,0.991418,0.990866,0.990297,0.989712,0.989109,0.988489,0.987853,0.987199,0.986529,0.985842,0.985138,0.984417,0.983680,0.982925,0.982154,0.981367,0.980562,0.979742,0.978904,0.978050,0.977179,0.976292,0.975389,0.974469,0.973533,0.972580,0.971611,0.970625,0.969624,0.968606,0.967572,0.966522,0.965456,0.964374,0.963276,0.962162,0.961031,0.959885,0.958724,0.957546,0.956353,0.955144,0.953919,0.952679,0.951423,0.950151,0.948864,0.947562,0.946244,0.944911,0.943563,0.942199,0.940821,0.939427,0.938018,0.936594,0.935155,0.933702,0.932233,0.930750,0.929251,0.927739,0.926211,0.924669,0.923112,0.921542,0.919956,0.918356,0.916742,0.915114,0.913472,0.911815,0.910145,0.908461,0.906762,0.905050,0.903324,0.901584,0.899831,0.898064,0.896284,0.894490,0.892683,0.890862,0.889029,0.887182,0.885322,0.883449,0.881563,0.879664,0.877752,0.875828,0.873891,0.871941,0.869979,0.868004,0.866017,0.864018,0.862007,0.859983,0.857947,0.855899,0.853840,0.851768,0.849685,0.847590,0.845484,0.843365,0.841236,0.839095,0.836943,0.834780,0.832605,0.830420,0.828223,0.826016,0.823798,0.821569,0.819329,0.817079,0.814819,0.812548,0.810267,0.807976,0.805674,0.803363,0.801041,0.798710,0.796369,0.794019,0.791658,0.789289,0.786910,0.784521,0.782123,0.779717,0.777301,0.774876,0.772442,0.770000,0.767549,0.765089,0.762621,0.760145,0.757660,0.755167,0.752665,0.750156,0.747639,0.745114,0.742581,0.740041,0.737493,0.734938,0.732375,0.729805,0.727228,0.724644,0.722053,0.719455,0.716850,0.714238,0.711620,0.708996,0.706365,0.703728,0.701084,0.698435,0.695780,0.693118,0.690451,0.687779,0.685100,0.682416,0.679727,0.677033,0.674333,0.671629,0.668919,0.666205,0.663485,0.660762,0.658033,0.655300,0.652563,0.649821,0.647076,0.644326,0.641572,0.638815,0.636054,0.633289,0.630521,0.627749,0.624974,0.622196,0.619414,0.616630,0.613843,0.611053,0.608260,0.605465,0.602667,0.599867,0.597065,0.594260,0.591454,0.588645,0.585835,0.583023,0.580209,0.577394,0.574577,0.571759,0.568940,0.566120,0.563299,0.560477,0.557654,0.554830,0.552006,0.549182,0.546357,0.543532,0.540706,0.537881,0.535056,0.532231,0.529406,0.526582,0.523758,0.520935,0.518112,0.515291,0.512470,0.509650,0.506832,0.504014,0.501199,0.498384,0.495571,0.492760,0.489950,0.487143,0.484337,0.481534,0.478733,0.475934,0.473137,0.470343,0.467552,0.464763,0.461977,0.459195,0.456415,0.453638,0.450865,0.448095,0.445328,0.442565,0.439806,0.437050,0.434299,0.431551,0.428807,0.426068,0.423333,0.420602,0.417876,0.415154,0.412438,0.409725,0.407018,0.404316,0.401619,0.398927,0.396241,0.393560,0.390884,0.388215,0.385550,0.382892,0.380240,0.377593,0.374953,0.372319,0.369691,0.367070,0.364455,0.361847,0.359246,0.356651,0.354063,0.351483,0.348909,0.346343,0.343784,0.341232,0.338688,0.336151,0.333623,0.331101,0.328588,0.326083,0.323586,0.321097,0.318616,0.316144,0.313680,0.311224,0.308778,0.306340,0.303910,0.301490,0.299079,0.296677,0.294283,0.291900,0.289525,0.287160,0.284805,0.282459,0.280123,0.277797,0.275480,0.273174,0.270877,0.268591,0.266315,0.264050,0.261794,0.259550,0.257315,0.255092,0.252879,0.250677,0.248486,0.246306,0.244137,0.241980,0.239833,0.237698,0.235574,0.233462,0.231361,0.229272,0.227195,0.225129,0.223075,0.221033,0.219004,0.216986,0.214981,0.212988,0.211007,0.209038,0.207082,0.205139,0.203208,0.201290,0.199385,0.197493,0.195613,0.193747,0.191893,0.190053,0.188226,0.186412,0.184611,0.182824,0.181051,0.179291,0.177544,0.175811,0.174092,0.172387,0.170696,0.169018,0.167355,0.165705,0.164070,0.162449,0.160842,0.159249,0.157671,0.156107,0.154558,0.153023,0.151503,0.149998,0.148507,0.147031,0.145570,0.144123,0.142692,0.141276,0.139874,0.138488,0.137117,0.135761,0.134420,0.133095,0.131785,0.130490,0.129211,0.127947,0.126699,0.125467,0.124250,0.123049,0.121863,0.120693,0.119540,0.118402,0.117279,0.116173,0.115083,0.114009,0.112951,0.111909,0.110883,0.109873,0.108880,0.107903,0.106942,0.105997,0.105069,0.104157,0.103262,0.102383,0.101521,0.100675,0.099846,0.099033,0.098237,0.097458,0.096695,0.095950,0.095220,0.094508,0.093812,0.093134,0.092472,0.091827,0.091199,0.090588,0.089993,0.089416,0.088856,0.088313,0.087787,0.087278,0.086785,0.086311,0.085853,0.085412,0.084989,0.084582,0.084193,0.083821,0.083466,0.083129,0.082808,0.082505,0.082219,0.081951,0.081700,0.081466,0.081249,0.081049,0.080867,0.080703,0.080555,0.080425,0.080312,0.080217,0.080139,0.080078,0.080035,0.080009,0.080000};

	float32_t hamw[NUMBER_OF_SAMPLES_IN_BUFFER] = {0};
	for (int i = 1; i < NUMBER_OF_SAMPLES_IN_BUFFER; i+= 1){
		hamw[i] = hamming_window[i] * (((float32_t)*(bufferIN+i))/(float32_t)MAX_INT_VALUE);
	}

	// 2. Perform FFT
	float32_t cplxFFT[NUMBER_OF_SAMPLES_IN_BUFFER];
	float32_t postFFT[NUMBER_OF_SAMPLES_IN_BUFFER/2];
    arm_rfft_fast_init_f32 (&realFFTinstance,NUMBER_OF_SAMPLES_IN_BUFFER);
    arm_rfft_fast_f32(&realFFTinstance, hamw, cplxFFT,0);
    arm_cmplx_mag_f32(cplxFFT, postFFT, NUMBER_OF_SAMPLES_IN_BUFFER/2); //The input array has a total of 2*numSamples values; the output array has a total of numSamples values


    // 3: Apply Mel filter banks and log-transform
	static const int16_t infoH[NBANKS][2] = {{11,4},{13,4},{16,4},{18,5},{21,5},{24,5},{27,6},{30,7},{34,7},{38,7},{42,8},{46,8},{51,9},{55,10},{61,10},{66,12},{72,13},{79,13},{86,14},{93,16},{101,17},{110,17},{119,19},{128,21},{139,22},{150,23},{162,25},{174,27},{188,29},{202,31},{218,33},{234,36},{252,38},{271,41},{291,44},{313,47},{336,50},{361,53},{387,58},{415,62},{446,65}};
	static const float32_t hbank[925] = {0.006387500000000,0.012775000000000,0.008516666666667,0.004258333333333,0.004258333333333,0.008516666666667,0.012775000000000,0.006387500000000,0.006387500000000,0.012775000000000,0.008516666666667,0.004258333333333,0.003548611111111,0.007097222222222,0.010645833333333,0.007097222222222,0.003548611111111,0.003548611111111,0.007097222222222,0.010645833333333,0.007097222222222,0.003548611111111,0.003548611111111,0.007097222222222,0.010645833333333,0.007097222222222,0.003548611111111,0.003041666666667,0.006083333333333,0.009125000000000,0.006843750000000,0.004562500000000,0.002281250000000,0.001996093750000,0.003992187500000,0.005988281250000,0.007984375000000,0.005988281250000,0.003992187500000,0.001996093750000,0.001996093750000,0.003992187500000,0.005988281250000,0.007984375000000,0.005988281250000,0.003992187500000,0.001996093750000,0.001996093750000,0.003992187500000,0.005988281250000,0.007984375000000,0.005988281250000,0.003992187500000,0.001996093750000,0.001774305555556,0.003548611111111,0.005322916666667,0.007097222222222,0.005677777777778,0.004258333333333,0.002838888888889,0.001419444444444,0.001419444444444,0.002838888888889,0.004258333333333,0.005677777777778,0.007097222222222,0.005322916666667,0.003548611111111,0.001774305555556,0.001596875000000,0.003193750000000,0.004790625000000,0.006387500000000,0.005322916666667,0.004258333333333,0.003193750000000,0.002129166666667,0.001064583333333,0.000967803030303,0.001935606060606,0.002903409090909,0.003871212121212,0.004839015151515,0.005806818181818,0.004645454545455,0.003484090909091,0.002322727272727,0.001161363636364,0.001161363636364,0.002322727272727,0.003484090909091,0.004645454545455,0.005806818181818,0.004839015151515,0.003871212121212,0.002903409090909,0.001935606060606,0.000967803030303,0.000818910256410,0.001637820512821,0.002456730769231,0.003275641025641,0.004094551282051,0.004913461538462,0.004211538461538,0.003509615384615,0.002807692307692,0.002105769230769,0.001403846153846,0.000701923076923,0.000651785714286,0.001303571428571,0.001955357142857,0.002607142857143,0.003258928571429,0.003910714285714,0.004562500000000,0.003910714285714,0.003258928571429,0.002607142857143,0.001955357142857,0.001303571428571,0.000651785714286,0.000651785714286,0.001303571428571,0.001955357142857,0.002607142857143,0.003258928571429,0.003910714285714,0.004562500000000,0.003910714285714,0.003258928571429,0.002607142857143,0.001955357142857,0.001303571428571,0.000651785714286,0.000608333333333,0.001216666666667,0.001825000000000,0.002433333333333,0.003041666666667,0.003650000000000,0.004258333333333,0.003726041666667,0.003193750000000,0.002661458333333,0.002129166666667,0.001596875000000,0.001064583333333,0.000532291666667,0.000469669117647,0.000939338235294,0.001409007352941,0.001878676470588,0.002348345588235,0.002818014705882,0.003287683823529,0.003757352941176,0.003339869281046,0.002922385620915,0.002504901960784,0.002087418300654,0.001669934640523,0.001252450980392,0.000834967320261,0.000417483660131,0.000394290123457,0.000788580246914,0.001182870370370,0.001577160493827,0.001971450617284,0.002365740740741,0.002760030864198,0.003154320987654,0.003548611111111,0.003154320987654,0.002760030864198,0.002365740740741,0.001971450617284,0.001577160493827,0.001182870370370,0.000788580246914,0.000394290123457,0.000394290123457,0.000788580246914,0.001182870370370,0.001577160493827,0.001971450617284,0.002365740740741,0.002760030864198,0.003154320987654,0.003548611111111,0.003154320987654,0.002760030864198,0.002365740740741,0.001971450617284,0.001577160493827,0.001182870370370,0.000788580246914,0.000394290123457,0.000354861111111,0.000709722222222,0.001064583333333,0.001419444444444,0.001774305555556,0.002129166666667,0.002484027777778,0.002838888888889,0.003193750000000,0.002903409090909,0.002613068181818,0.002322727272727,0.002032386363636,0.001742045454545,0.001451704545455,0.001161363636364,0.000871022727273,0.000580681818182,0.000290340909091,0.000263946280992,0.000527892561983,0.000791838842975,0.001055785123967,0.001319731404959,0.001583677685950,0.001847623966942,0.002111570247934,0.002375516528926,0.002639462809917,0.002903409090909,0.002639462809917,0.002375516528926,0.002111570247934,0.001847623966942,0.001583677685950,0.001319731404959,0.001055785123967,0.000791838842975,0.000527892561983,0.000263946280992,0.000252470355731,0.000504940711462,0.000757411067194,0.001009881422925,0.001262351778656,0.001514822134387,0.001767292490119,0.002019762845850,0.002272233201581,0.002524703557312,0.002777173913043,0.002545742753623,0.002314311594203,0.002082880434783,0.001851449275362,0.001620018115942,0.001388586956522,0.001157155797101,0.000925724637681,0.000694293478261,0.000462862318841,0.000231431159420,0.000221788194444,0.000443576388889,0.000665364583333,0.000887152777778,0.001108940972222,0.001330729166667,0.001552517361111,0.001774305555556,0.001996093750000,0.002217881944444,0.002439670138889,0.002661458333333,0.002439670138889,0.002217881944444,0.001996093750000,0.001774305555556,0.001552517361111,0.001330729166667,0.001108940972222,0.000887152777778,0.000665364583333,0.000443576388889,0.000221788194444,0.000204727564103,0.000409455128205,0.000614182692308,0.000818910256410,0.001023637820513,0.001228365384615,0.001433092948718,0.001637820512821,0.001842548076923,0.002047275641026,0.002252003205128,0.002456730769231,0.002281250000000,0.002105769230769,0.001930288461538,0.001754807692308,0.001579326923077,0.001403846153846,0.001228365384615,0.001052884615385,0.000877403846154,0.000701923076923,0.000526442307692,0.000350961538462,0.000175480769231,0.000162946428571,0.000325892857143,0.000488839285714,0.000651785714286,0.000814732142857,0.000977678571429,0.001140625000000,0.001303571428571,0.001466517857143,0.001629464285714,0.001792410714286,0.001955357142857,0.002118303571429,0.002281250000000,0.002118303571429,0.001955357142857,0.001792410714286,0.001629464285714,0.001466517857143,0.001303571428571,0.001140625000000,0.000977678571429,0.000814732142857,0.000651785714286,0.000488839285714,0.000325892857143,0.000162946428571,0.000152083333333,0.000304166666667,0.000456250000000,0.000608333333333,0.000760416666667,0.000912500000000,0.001064583333333,0.001216666666667,0.001368750000000,0.001520833333333,0.001672916666667,0.001825000000000,0.001977083333333,0.002129166666667,0.001996093750000,0.001863020833333,0.001729947916667,0.001596875000000,0.001463802083333,0.001330729166667,0.001197656250000,0.001064583333333,0.000931510416667,0.000798437500000,0.000665364583333,0.000532291666667,0.000399218750000,0.000266145833333,0.000133072916667,0.000124755859375,0.000249511718750,0.000374267578125,0.000499023437500,0.000623779296875,0.000748535156250,0.000873291015625,0.000998046875000,0.001122802734375,0.001247558593750,0.001372314453125,0.001497070312500,0.001621826171875,0.001746582031250,0.001871337890625,0.001996093750000,0.001871337890625,0.001746582031250,0.001621826171875,0.001497070312500,0.001372314453125,0.001247558593750,0.001122802734375,0.000998046875000,0.000873291015625,0.000748535156250,0.000623779296875,0.000499023437500,0.000374267578125,0.000249511718750,0.000124755859375,0.000117417279412,0.000234834558824,0.000352251838235,0.000469669117647,0.000587086397059,0.000704503676471,0.000821920955882,0.000939338235294,0.001056755514706,0.001174172794118,0.001291590073529,0.001409007352941,0.001526424632353,0.001643841911765,0.001761259191176,0.001878676470588,0.001774305555556,0.001669934640523,0.001565563725490,0.001461192810458,0.001356821895425,0.001252450980392,0.001148080065359,0.001043709150327,0.000939338235294,0.000834967320261,0.000730596405229,0.000626225490196,0.000521854575163,0.000417483660131,0.000313112745098,0.000208741830065,0.000104370915033,0.000095908408408,0.000191816816817,0.000287725225225,0.000383633633634,0.000479542042042,0.000575450450450,0.000671358858859,0.000767267267267,0.000863175675676,0.000959084084084,0.001054992492492,0.001150900900901,0.001246809309309,0.001342717717718,0.001438626126126,0.001534534534535,0.001630442942943,0.001726351351351,0.001635490753912,0.001544630156472,0.001453769559033,0.001362908961593,0.001272048364154,0.001181187766714,0.001090327169275,0.000999466571835,0.000908605974395,0.000817745376956,0.000726884779516,0.000636024182077,0.000545163584637,0.000454302987198,0.000363442389758,0.000272581792319,0.000181721194879,0.000090860597440,0.000086201079622,0.000172402159244,0.000258603238866,0.000344804318489,0.000431005398111,0.000517206477733,0.000603407557355,0.000689608636977,0.000775809716599,0.000862010796221,0.000948211875843,0.001034412955466,0.001120614035088,0.001206815114710,0.001293016194332,0.001379217273954,0.001465418353576,0.001551619433198,0.001637820512821,0.001555929487179,0.001474038461538,0.001392147435897,0.001310256410256,0.001228365384615,0.001146474358974,0.001064583333333,0.000982692307692,0.000900801282051,0.000818910256410,0.000737019230769,0.000655128205128,0.000573237179487,0.000491346153846,0.000409455128205,0.000327564102564,0.000245673076923,0.000163782051282,0.000081891025641,0.000076041666667,0.000152083333333,0.000228125000000,0.000304166666667,0.000380208333333,0.000456250000000,0.000532291666667,0.000608333333333,0.000684375000000,0.000760416666667,0.000836458333333,0.000912500000000,0.000988541666667,0.001064583333333,0.001140625000000,0.001216666666667,0.001292708333333,0.001368750000000,0.001444791666667,0.001520833333333,0.001451704545455,0.001382575757576,0.001313446969697,0.001244318181818,0.001175189393939,0.001106060606061,0.001036931818182,0.000967803030303,0.000898674242424,0.000829545454545,0.000760416666667,0.000691287878788,0.000622159090909,0.000553030303030,0.000483901515152,0.000414772727273,0.000345643939394,0.000276515151515,0.000207386363636,0.000138257575758,0.000069128787879,0.000064520202020,0.000129040404040,0.000193560606061,0.000258080808081,0.000322601010101,0.000387121212121,0.000451641414141,0.000516161616162,0.000580681818182,0.000645202020202,0.000709722222222,0.000774242424242,0.000838762626263,0.000903282828283,0.000967803030303,0.001032323232323,0.001096843434343,0.001161363636364,0.001225883838384,0.001290404040404,0.001354924242424,0.001419444444444,0.001357729468599,0.001296014492754,0.001234299516908,0.001172584541063,0.001110869565217,0.001049154589372,0.000987439613527,0.000925724637681,0.000864009661836,0.000802294685990,0.000740579710145,0.000678864734300,0.000617149758454,0.000555434782609,0.000493719806763,0.000432004830918,0.000370289855072,0.000308574879227,0.000246859903382,0.000185144927536,0.000123429951691,0.000061714975845,0.000057857789855,0.000115715579710,0.000173573369565,0.000231431159420,0.000289288949275,0.000347146739130,0.000405004528986,0.000462862318841,0.000520720108696,0.000578577898551,0.000636435688406,0.000694293478261,0.000752151268116,0.000810009057971,0.000867866847826,0.000925724637681,0.000983582427536,0.001041440217391,0.001099298007246,0.001157155797101,0.001215013586957,0.001272871376812,0.001330729166667,0.001277500000000,0.001224270833333,0.001171041666667,0.001117812500000,0.001064583333333,0.001011354166667,0.000958125000000,0.000904895833333,0.000851666666667,0.000798437500000,0.000745208333333,0.000691979166667,0.000638750000000,0.000585520833333,0.000532291666667,0.000479062500000,0.000425833333333,0.000372604166667,0.000319375000000,0.000266145833333,0.000212916666667,0.000159687500000,0.000106458333333,0.000053229166667,0.000050098039216,0.000100196078431,0.000150294117647,0.000200392156863,0.000250490196078,0.000300588235294,0.000350686274510,0.000400784313725,0.000450882352941,0.000500980392157,0.000551078431373,0.000601176470588,0.000651274509804,0.000701372549020,0.000751470588235,0.000801568627451,0.000851666666667,0.000901764705882,0.000951862745098,0.001001960784314,0.001052058823529,0.001102156862745,0.001152254901961,0.001202352941176,0.001252450980392,0.001204279788839,0.001156108597285,0.001107937405732,0.001059766214178,0.001011595022624,0.000963423831071,0.000915252639517,0.000867081447964,0.000818910256410,0.000770739064857,0.000722567873303,0.000674396681750,0.000626225490196,0.000578054298643,0.000529883107089,0.000481711915535,0.000433540723982,0.000385369532428,0.000337198340875,0.000289027149321,0.000240855957768,0.000192684766214,0.000144513574661,0.000096342383107,0.000048171191554,0.000045495014245,0.000090990028490,0.000136485042735,0.000181980056980,0.000227475071225,0.000272970085470,0.000318465099715,0.000363960113960,0.000409455128205,0.000454950142450,0.000500445156695,0.000545940170940,0.000591435185185,0.000636930199430,0.000682425213675,0.000727920227920,0.000773415242165,0.000818910256410,0.000864405270655,0.000909900284900,0.000955395299145,0.001000890313390,0.001046385327635,0.001091880341880,0.001137375356125,0.001182870370370,0.001140625000000,0.001098379629630,0.001056134259259,0.001013888888889,0.000971643518519,0.000929398148148,0.000887152777778,0.000844907407407,0.000802662037037,0.000760416666667,0.000718171296296,0.000675925925926,0.000633680555556,0.000591435185185,0.000549189814815,0.000506944444444,0.000464699074074,0.000422453703704,0.000380208333333,0.000337962962963,0.000295717592593,0.000253472222222,0.000211226851852,0.000168981481481,0.000126736111111,0.000084490740741,0.000042245370370,0.000038665254237,0.000077330508475,0.000115995762712,0.000154661016949,0.000193326271186,0.000231991525424,0.000270656779661,0.000309322033898,0.000347987288136,0.000386652542373,0.000425317796610,0.000463983050847,0.000502648305085,0.000541313559322,0.000579978813559,0.000618644067797,0.000657309322034,0.000695974576271,0.000734639830508,0.000773305084746,0.000811970338983,0.000850635593220,0.000889300847458,0.000927966101695,0.000966631355932,0.001005296610169,0.001043961864407,0.001082627118644,0.001047703663204,0.001012780207764,0.000977856752324,0.000942933296884,0.000908009841443,0.000873086386003,0.000838162930563,0.000803239475123,0.000768316019683,0.000733392564243,0.000698469108803,0.000663545653362,0.000628622197922,0.000593698742482,0.000558775287042,0.000523851831602,0.000488928376162,0.000454004920722,0.000419081465282,0.000384158009841,0.000349234554401,0.000314311098961,0.000279387643521,0.000244464188081,0.000209540732641,0.000174617277201,0.000139693821761,0.000104770366320,0.000069846910880,0.000034923455440,0.000032706093190,0.000065412186380,0.000098118279570,0.000130824372760,0.000163530465950,0.000196236559140,0.000228942652330,0.000261648745520,0.000294354838710,0.000327060931900,0.000359767025090,0.000392473118280,0.000425179211470,0.000457885304659,0.000490591397849,0.000523297491039,0.000556003584229,0.000588709677419,0.000621415770609,0.000654121863799,0.000686827956989,0.000719534050179,0.000752240143369,0.000784946236559,0.000817652329749,0.000850358422939,0.000883064516129,0.000915770609319,0.000948476702509,0.000981182795699,0.001013888888889,0.000982204861111,0.000950520833333,0.000918836805556,0.000887152777778,0.000855468750000,0.000823784722222,0.000792100694444,0.000760416666667,0.000728732638889,0.000697048611111,0.000665364583333,0.000633680555556,0.000601996527778,0.000570312500000,0.000538628472222,0.000506944444444,0.000475260416667,0.000443576388889,0.000411892361111,0.000380208333333,0.000348524305556,0.000316840277778,0.000285156250000,0.000253472222222,0.000221788194444,0.000190104166667,0.000158420138889,0.000126736111111,0.000095052083333,0.000063368055556,0.000031684027778,0.000030243844697,0.000060487689394,0.000090731534091,0.000120975378788,0.000151219223485,0.000181463068182,0.000211706912879,0.000241950757576,0.000272194602273,0.000302438446970,0.000332682291667,0.000362926136364,0.000393169981061,0.000423413825758,0.000453657670455,0.000483901515152,0.000514145359848,0.000544389204545,0.000574633049242,0.000604876893939,0.000635120738636,0.000665364583333,0.000695608428030,0.000725852272727,0.000756096117424,0.000786339962121,0.000816583806818,0.000846827651515,0.000877071496212,0.000907315340909,0.000937559185606,0.000967803030303,0.000939338235294,0.000910873440285,0.000882408645276,0.000853943850267,0.000825479055258,0.000797014260250,0.000768549465241,0.000740084670232,0.000711619875223,0.000683155080214,0.000654690285205,0.000626225490196,0.000597760695187,0.000569295900178,0.000540831105169,0.000512366310160,0.000483901515152,0.000455436720143,0.000426971925134,0.000398507130125,0.000370042335116,0.000341577540107,0.000313112745098,0.000284647950089,0.000256183155080,0.000227718360071,0.000199253565062,0.000170788770053,0.000142323975045,0.000113859180036,0.000085394385027,0.000056929590018,0.000028464795009};

	static float32_t preDCT[NBANKS-1] = {0};
    int16_t punt = 0;
    for (int ibank = 0; ibank < NBANKS-1; ibank += 1){
    	float32_t sum = 0;
    	for (int i = 0; i < infoH[ibank][1]; i += 1){
    		sum = sum + (*(postFFT+infoH[ibank][0]+i)) * hbank[i+punt];
    	}
    	punt = punt + infoH[ibank][1];
    	preDCT[ibank] = (float32_t)log10((double)sum);
    }

    // 4: Perform DCT to obtain MFCCs
    DCTII(preDCT, bufferOUT);
}


/* 
 * Function: DCTII

 * Purpose: Perform Discrete Cosine Transform (DCT-II) on the input data.
 * 
 * Steps:
 * 1. Iterate through each DCT coefficient (k).

 * 2. For each coefficient, compute the weighted sum of input values.
 * 
 * Parameters:
 *  - indct: Pointer to input data for DCT.
 *  - outdct: Pointer to output data for the transformed values.
 */
static void DCTII(float32_t *indct, float32_t *outdct){
	static const float32_t W = 0.223606797749979;
	static const float32_t t = 0.039269908169872;

	for (int k = 1; k < NUMBER_OF_SAMPLES_IN_BUFFERS_MFCC + 1; k += 1){
		float32_t sum = 0;
		for (int n = 0; n < NBANKS-1; n+= 1){
		    sum = sum + *(indct+n)*arm_cos_f32((2*n+1)*k*t);
		}

		*(outdct+k-1) = W * sum;
	}
}

/* 
 * Function: deltas

 * Purpose: Calculate the delta (time-derivative) features from the MFCCs.
 * 
 * Steps:
 * 1. Initialize matrix instances for MFCCs and weighted matrices.

 * 2. Scale and add matrices to compute the deltas.
 * 
 * Parameters:
 *  - Dbuffers: Pointer to the MFCC buffers (2D array).

 */
static void deltas(float32_t **Dbuffers){

	// Initialize matrices for MFCCs and weighted values
	uint8_t srcRows = NUMBER_OF_SAMPLES_IN_BUFFERS_MFCC;
	uint8_t srcColumns = 1;
	arm_matrix_instance_f32 M0;
	arm_matrix_instance_f32 M1;
	arm_matrix_instance_f32 M2;
	arm_matrix_instance_f32 M3;
	arm_matrix_instance_f32 M4;
	arm_matrix_instance_f32 M0K;
	arm_matrix_instance_f32 M1K;
	arm_matrix_instance_f32 M3K;
	arm_matrix_instance_f32 M4K;
	arm_matrix_instance_f32 MDELT;

	arm_mat_init_f32(&M0, srcRows, srcColumns, *Dbuffers);
	arm_mat_init_f32(&M1, srcRows, srcColumns, *(Dbuffers + 1));
	arm_mat_init_f32(&M2, srcRows, srcColumns, *(Dbuffers + 2));
	arm_mat_init_f32(&M3, srcRows, srcColumns, *(Dbuffers + 3));
	arm_mat_init_f32(&M4, srcRows, srcColumns, *(Dbuffers + 4));
	arm_mat_init_f32(&MDELT, srcRows, srcColumns, *(Dbuffers + 2) + NUMBER_OF_SAMPLES_IN_BUFFERS_MFCC);

	arm_mat_init_f32(&M4K, srcRows, srcColumns, MK4);
	arm_mat_init_f32(&M3K, srcRows, srcColumns, MK3);
	arm_mat_init_f32(&M1K, srcRows, srcColumns, MK1);
	arm_mat_init_f32(&M0K, srcRows, srcColumns, MK0);

    // Define scaling factors for deltas
	const float32_t scalep1 = 0.1;
	const float32_t scalep2 = 0.2;
	const float32_t scalen1 = -0.1;
	const float32_t scalen2 = -0.2;

    // Scale the matrices
	arm_mat_scale_f32 (&M0,scalen2, &M0K);
	arm_mat_scale_f32 (&M1,scalen1, &M1K);
	arm_mat_scale_f32 (&M3,scalep1, &M3K);
	arm_mat_scale_f32 (&M4,scalep2, &M4K);

    // Add the scaled matrices to compute the deltas
	arm_mat_add_f32 (&M0K,&M4K,&M0K);
	arm_mat_add_f32 (&M1K,&M3K,&M1K);
	arm_mat_add_f32 (&M0K,&M1K,&MDELT);

}

/* 
 * Function: neuralNetwork
 * Purpose: Compute the output probability of the neural network given MFCCs as input.
 * 
 * Steps:
 * 1. Perform matrix multiplication for the input and hidden layer.
 * 2. Add biases and apply activation (tansig function).
 * 3. Perform matrix multiplication for the output layer.
 * 4. Add biases and apply softmax to compute probabilities.
 * 
 * Parameters:
 *  - bufferMFCC: Pointer to the input MFCC data.
 * 
 * Returns:
 *  - Output probability for the target class.
 */
static float32_t neuralNetwork(float32_t *bufferMFCC){

    // Define weights and biases for the neural network (extracted from MATLAB)
	static const float32_t b1[2] = {-0.80761591265985332999,0.70817827130235644351};
	static const float32_t b2[2] = {-2.9522122406295876473,3.1464177655874263628};
	static const float32_t A1[48] = {-0.200636658389935,-0.491093372135600,0.257896622560145,0.472462088911258,-1.629308584860063,0.991262953888194,1.052882670001120,-1.011546126598006,0.183028236169138,0.451996808211366,-0.559336443354897,0.447844079532080,0.209119774765904,-1.006003354442111,1.214402688241593,0.322894458349409,-1.374734024759847,0.107924508383444,-0.571046024039248,-0.073493046741152,-0.171147898388103,-0.025146114075661,-0.234303800556554,-0.321337287816772,0.070967814786528,-0.388798047984194,-0.864365181456682,-0.075849029641188,0.900608172165006,-0.459210515854226,-0.655127014731351,0.694567236929201,-0.043727119290212,0.424118461825914,0.237008977612013,-0.268814248741889,-0.734323562165055,0.629388825091410,-0.645229160317088,-0.348760228496328,0.397508290970482,0.837171909872718,-0.276476212519036,-0.924267190567635,-0.174003225436541,-0.340519350527777,0.355185996151635,0.075633771364941};
	static const float32_t A2[4] = {3.1790411477338480495, 2.4568203259422229934,-2.8553231719653346943, -3.3896470973358105994};

	float32_t L1[2]; // Hidden layer outputs
	float32_t L2[2]; // Output layer outputs

    // Initialize matrices for the neural network
	arm_matrix_instance_f32 Minput;
	arm_matrix_instance_f32 MA1;
	arm_matrix_instance_f32 Mb1;
	arm_matrix_instance_f32 MA2;
	arm_matrix_instance_f32 Mb2;
	arm_matrix_instance_f32 ML1;
	arm_matrix_instance_f32 ML2;


	uint8_t rows = 2;
	uint8_t columns = 2*NUMBER_OF_SAMPLES_IN_BUFFERS_MFCC;
	uint8_t srcRows = NUMBER_OF_SAMPLES_IN_BUFFERS_MFCC;
	uint8_t srcColumns = 1;

	arm_status status;
	arm_mat_init_f32(&Minput, 2*srcRows, srcColumns, bufferMFCC);
	arm_mat_init_f32(&MA1, rows, columns, A1);
	arm_mat_init_f32(&Mb1, rows, srcColumns, b1);
	arm_mat_init_f32(&MA2, rows, 2*srcColumns, A2);
	arm_mat_init_f32(&Mb2, rows, srcColumns, b2);
	arm_mat_init_f32(&ML1, rows, srcColumns, L1);
	arm_mat_init_f32(&ML2, rows, srcColumns, L2);

	//Input and hidden layer computations
	status = arm_mat_mult_f32 (&MA1,&Minput,&ML1);

	if(ARM_MATH_SUCCESS == 1){
		FLASH_LED(Both, LONG_LED_FLASH_DURATION);
	}

	arm_mat_add_f32 (&Mb1,&ML1,&ML1);

	// Apply tansig activation
	for (int i = 0; i < 2; i += 1 ){
		L1[i] = 2 / (1 + exp(-2*L1[i])) - 1;
	}

	//Output layer computation
	arm_mat_mult_f32 (&MA2,&ML1,&ML2);

	arm_mat_add_f32 (&Mb2,&ML2,&ML2);

	// Apply Softmax activation
	float32_t expo;
	expo = exp(L2[0]);

	return expo/(exp(L2[1])+expo);

}
