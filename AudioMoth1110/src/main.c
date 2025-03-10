/****************************************************************************
 * main.c
 * openacousticdevices.info
 * June 2017
 *****************************************************************************/

// Changes with respect to original code are marked as 'Introduced' or 'Modified'

#include <time.h>
#include <math.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>

#include "gps.h"
#include "sunrise.h"
#include "audiomoth.h"
#include "audioconfig.h"
#include "digitalfilter.h"

//#include <complex.h>
#include "ff.h"
//#include "ffconf.h"
#define __FPU_PRESENT       1    /*!< FPU present */
//#include "math.h"
#include "arm_math.h"

/* Useful time constants */

#define MILLISECONDS_IN_SECOND                  1000

#define SECONDS_IN_MINUTE                       60
#define SECONDS_IN_HOUR                         (60 * SECONDS_IN_MINUTE)
#define SECONDS_IN_DAY                          (24 * SECONDS_IN_HOUR)

#define MINUTES_IN_HOUR                         60
#define MINUTES_IN_DAY                          1440
#define YEAR_OFFSET                             1900
#define MONTH_OFFSET                            1              

#define START_OF_CENTURY                        946684800
#define MIDPOINT_OF_CENTURY                     2524608000

/* Useful coordinate constant */

#define MINUTES_IN_DEGREE                       60

/* Useful type constants */

#define BITS_PER_BYTE                           8
#define UINT32_SIZE_IN_BITS                     32
#define UINT32_SIZE_IN_BYTES                    4
#define UINT16_SIZE_IN_BYTES                    2

/* Sleep and LED constants */

#define LOW_BATTERY_LED_FLASHES                 10

#define SHORT_LED_FLASH_DURATION                100
#define LONG_LED_FLASH_DURATION                 500

#define WAITING_LED_FLASH_DURATION              10
#define WAITING_LED_FLASH_INTERVAL              2000

#define MINIMUM_LED_FLASH_INTERVAL              500

#define SHORT_WAIT_INTERVAL                     100
#define DEFAULT_WAIT_INTERVAL                   1000

#define USB_CONFIGURATION_BLINK                 400

/* SRAM buffer constants (modified) */

#define NUMBER_OF_BUFFERS                       128 // (modified from 8)
#define NUMBER_OF_BYTES_IN_SAMPLE               2
#define EXTERNAL_SRAM_SIZE_IN_SAMPLES           (AM_EXTERNAL_SRAM_SIZE_IN_BYTES / NUMBER_OF_BYTES_IN_SAMPLE)
#define NUMBER_OF_SAMPLES_IN_BUFFER             (EXTERNAL_SRAM_SIZE_IN_SAMPLES / NUMBER_OF_BUFFERS)

// ---> Introduced constants:
#define NUMBER_OF_SUPERBUFFERS				8 // aggregation of 16 buffers
#define NUMBER_OF_BUFFERS_IN_SUPERBUFFER	(NUMBER_OF_BUFFERS / NUMBER_OF_SUPERBUFFERS)
#define NUMBER_OF_SAMPLES_IN_SUPERBUFFER	NUMBER_OF_SAMPLES_IN_BUFFER * NUMBER_OF_BUFFERS_IN_SUPERBUFFER

#define NUMBER_OF_BUFFERS_MFCC              5
#define NUMBER_OF_SAMPLES_IN_BUFFERS_MFCC	12
#define NBANKS								41
#define INIT_COUNTDOWN						1230
#define THRESHOLD_DETECTION					0.5f
//#define DETECTION_VALUE						THRESHOLD_DETECTION * NUMBER_OF_BUFFERS_IN_SUPERBUFFER

#define MAX_INT_VALUE						32767
// <---

/* DMA transfer constant */

#define MAXIMUM_SAMPLES_IN_DMA_TRANSFER         1024

/* Compression constants */

#define COMPRESSION_BUFFER_SIZE_IN_BYTES        512

/* File size constants */

#define MAXIMUM_FILE_NAME_LENGTH                64

#define MAXIMUM_WAV_FILE_SIZE                   (UINT32_MAX - 1024 * 1024)

/* Configuration file constants */

#define CONFIG_BUFFER_LENGTH                    512
#define CONFIG_TIMEZONE_LENGTH                  8

/* WAV header constant */

#define PCM_FORMAT                              1
#define RIFF_ID_LENGTH                          4
#define LENGTH_OF_ARTIST                        32
#define LENGTH_OF_COMMENT                       384

/* USB configuration constant */

#define MAX_RECORDING_PERIODS                   5

/* Digital filter constant */

#define FILTER_FREQ_MULTIPLIER                  100

/* DC filter constants */

#define LOW_DC_BLOCKING_FREQ                    8
#define DEFAULT_DC_BLOCKING_FREQ                48

/* Supply voltage constant */

#define MINIMUM_SUPPLY_VOLTAGE                  2800

/* Recording error constant */

#define MAXIMUM_NUMBER_OF_RECORDING_ERRORS      5

/* Deployment ID constant */

#define DEPLOYMENT_ID_LENGTH                    8

/* Acoustic location constant */

#define ACOUSTIC_LOCATION_SIZE_IN_BYTES         7

/* Audio configuration constants */

#define AUDIO_CONFIG_PULSE_INTERVAL             10
#define AUDIO_CONFIG_TIME_CORRECTION            134
#define AUDIO_CONFIG_TONE_TIMEOUT               250
#define AUDIO_CONFIG_PACKETS_TIMEOUT            30000

/* GPS time setting constants */

#define GPS_MAXIMUM_MS_DIFFERENCE               (SECONDS_IN_HOUR * MILLISECONDS_IN_SECOND)
#define GPS_INITIAL_TIME_SETTING_PERIOD         300
#define GPS_DEFAULT_TIME_SETTING_PERIOD         300
#define GPS_MIN_TIME_SETTING_PERIOD             30
#define GPS_TIME_SETTING_MARGIN                 2
#define GPS_FREQUENCY_PRECISION                 1000
#define GPS_FILENAME                            "GPS.TXT"

/* Magnetic switch constants */

#define MAGNETIC_SWITCH_WAIT_MULTIPLIER         2
#define MAGNETIC_SWITCH_CHANGE_FLASHES          10

/* USB configuration constant */

#define USB_CONFIG_TIME_CORRECTION              26

/* Recording preparation constants */

#define PREPARATION_PERIOD_INCREMENT            250
#define MINIMUM_PREPARATION_PERIOD              750
#define INITIAL_PREPARATION_PERIOD              2000
#define MAXIMUM_PREPARATION_PERIOD              30000

/* Energy saver mode constant */

#define ENERGY_SAVER_SAMPLE_RATE_THRESHOLD      48000

/* Frequency trigger constants */

#define FREQUENCY_TRIGGER_WINDOW_MINIMUM        16
#define FREQUENCY_TRIGGER_WINDOW_MAXIMUM        1024

/* Sunrise and sunset recording constants */

#define MINIMUM_SUN_RECORDING_GAP               60
#define SUN_RECORDING_GAP_MULTIPLIER            4

/* Location constants */

#define ACOUSTIC_LONGITUDE_MULTIPLIER           2

#define CONFIG_LOCATION_PRECISION               100
#define ACOUSTIC_LOCATION_PRECISION             1000000
#define GPS_LOCATION_PRECISION                  1000000

/* Useful macros */

#define FLASH_LED(led, duration) { \
    AudioMoth_set ## led ## LED(true); \
    AudioMoth_delay(duration); \
    AudioMoth_set ## led ## LED(false); \
}

#define FLASH_REPEAT_LED(led, repeats, duration) { \
    for (uint32_t i = 0; i < repeats; i += 1) { \
        AudioMoth_set ## led ## LED(true); \
        AudioMoth_delay(duration); \
        AudioMoth_set ## led ## LED(false); \
        AudioMoth_delay(duration); \
    } \
}

#define FLASH_LED_AND_RETURN_ON_ERROR(fn) { \
    bool success = (fn); \
    if (success != true) { \
        AudioMoth_setBothLED(false); \
        AudioMoth_delay(LONG_LED_FLASH_DURATION); \
        FLASH_LED(Both, LONG_LED_FLASH_DURATION) \
        return SDCARD_WRITE_ERROR; \
    } \
}

#define RETURN_BOOL_ON_ERROR(fn) { \
    bool success = (fn); \
    if (success != true) { \
        return success; \
    } \
}

#define SAVE_SWITCH_POSITION_AND_POWER_DOWN(milliseconds) { \
    *previousSwitchPosition = switchPosition; \
    AudioMoth_powerDownAndWakeMilliseconds(milliseconds); \
}

#define SERIAL_NUMBER                           "%08X%08X"

#define FORMAT_SERIAL_NUMBER(src)               (unsigned int)*((uint32_t*)src + 1),  (unsigned int)*((uint32_t*)src)

#define ABS(a)                                  ((a) < (0) ? (-a) : (a))

#define MIN(a, b)                               ((a) < (b) ? (a) : (b))

#define MAX(a, b)                               ((a) > (b) ? (a) : (b))

#define ROUNDED_DIV(a, b)                       (((a) + (b/2)) / (b))

#define ROUNDED_UP_DIV(a, b)                    (((a) + (b) - 1) / (b))

#define ROUND_UP_TO_MULTIPLE(a, b)              (((a) + (b) - 1) & ~((b)-1))

#define UNSIGNED_ROUND(n, d)                    ((d) * (((n) + (d) / 2) / (d)))

/* Recording state enumeration */

typedef enum {RECORDING_OKAY, FILE_SIZE_LIMITED, SUPPLY_VOLTAGE_LOW, SWITCH_CHANGED, MICROPHONE_CHANGED, MAGNETIC_SWITCH, SDCARD_WRITE_ERROR} AM_recordingState_t;

/* Filter type enumeration */

typedef enum {NO_FILTER, LOW_PASS_FILTER, BAND_PASS_FILTER, HIGH_PASS_FILTER} AM_filterType_t;

/* Battery level display type */

typedef enum {BATTERY_LEVEL, NIMH_LIPO_BATTERY_VOLTAGE} AM_batteryLevelDisplayType_t;

/* Sun recording mode enumeration */

typedef enum {SUNRISE_RECORDING, SUNSET_RECORDING, SUNRISE_AND_SUNSET_RECORDING, SUNSET_TO_SUNRISE_RECORDING, SUNRISE_TO_SUNSET_RECORDING} AM_sunRecordingMode_t;

/* Sun recording mode enumeration */

typedef enum {INITIAL_GPS_FIX, GPS_FIX_BEFORE_RECORDING_PERIOD, GPS_FIX_BETWEEN_RECORDING_PERIODS, GPS_FIX_AFTER_RECORDING_PERIOD, GPS_FIX_BEFORE_INDIVIDUAL_RECORDING, GPS_FIX_BETWEEN_INDIVIDUAL_RECORDINGS, GPS_FIX_AFTER_INDIVIDUAL_RECORDING} AM_gpsFixMode_t;

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

/* USB configuration data structure */

#pragma pack(push, 1)

typedef struct {
    uint16_t startMinutes;
    uint16_t endMinutes;
} recordingPeriod_t;

typedef struct {
    uint32_t time;
    AM_gainSetting_t gain;
    uint8_t clockDivider;
    uint8_t acquisitionCycles;
    uint8_t oversampleRate;
    uint32_t sampleRate;
    uint8_t sampleRateDivider;
    uint16_t sleepDuration;
    uint16_t recordDuration;
    uint8_t enableLED;
    union {
        struct {
            uint8_t activeRecordingPeriods;
            recordingPeriod_t recordingPeriods[MAX_RECORDING_PERIODS];
        };
        struct {
            uint8_t sunRecordingMode : 3;
            uint8_t sunRecordingEvent : 2;
            int16_t latitude;
            int16_t longitude;
            uint8_t sunRoundingMinutes;
            uint16_t beforeSunriseMinutes : 10;
            uint16_t afterSunriseMinutes : 10;
            uint16_t beforeSunsetMinutes : 10;
            uint16_t afterSunsetMinutes : 10;
        };
    };
    int8_t timezoneHours;
    uint8_t enableLowVoltageCutoff;
    uint8_t disableBatteryLevelDisplay;
    int8_t timezoneMinutes;
    uint8_t disableSleepRecordCycle : 1;
    uint8_t enableFilenameWithDeviceID : 1;
    uint8_t enableTimeSettingBeforeAndAfterRecordings: 1;
    uint8_t gpsTimeSettingPeriod: 4;
    uint32_t earliestRecordingTime;
    uint32_t latestRecordingTime;
    uint16_t lowerFilterFreq;
    uint16_t higherFilterFreq;
    union {
        uint16_t amplitudeThreshold;
        uint16_t frequencyTriggerCentreFrequency;
    };
    uint8_t requireAcousticConfiguration : 1;
    AM_batteryLevelDisplayType_t batteryLevelDisplayType : 1;
    uint8_t minimumTriggerDuration : 6;
    union {
        struct {
            uint8_t frequencyTriggerWindowLengthShift : 4; 
            uint8_t frequencyTriggerThresholdPercentageMantissa : 4; 
            int8_t frequencyTriggerThresholdPercentageExponent : 3; 
        };
        struct {
            uint8_t enableAmplitudeThresholdDecibelScale : 1;
            uint8_t amplitudeThresholdDecibels : 7; 
            uint8_t enableAmplitudeThresholdPercentageScale : 1;
            uint8_t amplitudeThresholdPercentageMantissa : 4; 
            int8_t amplitudeThresholdPercentageExponent : 3; 
        };
    };
    uint8_t enableEnergySaverMode : 1; 
    uint8_t disable48HzDCBlockingFilter : 1;
    uint8_t enableTimeSettingFromGPS : 1;
    uint8_t enableMagneticSwitch : 1;
    uint8_t enableLowGainRange : 1;
    uint8_t enableFrequencyTrigger : 1;
    uint8_t enableDailyFolders : 1;
    uint8_t enableSunRecording : 1;
} configSettings_t;

#pragma pack(pop)

static const configSettings_t defaultConfigSettings = {
    .time = 0,
    .gain = AM_GAIN_MEDIUM,
    .clockDivider = 4,
    .acquisitionCycles = 16,
    .oversampleRate = 1,
    .sampleRate = 384000,
    .sampleRateDivider = 8,
    .sleepDuration = 5,
    .recordDuration = 55,
    .enableLED = 1,
    .activeRecordingPeriods = 1,
    .recordingPeriods = {
        {.startMinutes = 0, .endMinutes = 0},
        {.startMinutes = 0, .endMinutes = 0},
        {.startMinutes = 0, .endMinutes = 0},
        {.startMinutes = 0, .endMinutes = 0},
        {.startMinutes = 0, .endMinutes = 0}
    },
    .timezoneHours = 0,
    .enableLowVoltageCutoff = 1,
    .disableBatteryLevelDisplay = 0,
    .timezoneMinutes = 0,
    .disableSleepRecordCycle = 0,
    .enableFilenameWithDeviceID = 0,
    .enableTimeSettingBeforeAndAfterRecordings = 0,
    .gpsTimeSettingPeriod = 0,
    .earliestRecordingTime = 0,
    .latestRecordingTime = 0,
    .lowerFilterFreq = 0,
    .higherFilterFreq = 0,
    .amplitudeThreshold = 0,
    .requireAcousticConfiguration = 0,
    .batteryLevelDisplayType = BATTERY_LEVEL,
    .minimumTriggerDuration = 0,
    .enableAmplitudeThresholdDecibelScale = 0,
    .amplitudeThresholdDecibels = 0,
    .enableAmplitudeThresholdPercentageScale = 0,
    .amplitudeThresholdPercentageMantissa = 0,
    .amplitudeThresholdPercentageExponent = 0,
    .enableEnergySaverMode = 0,
    .disable48HzDCBlockingFilter = 0,
    .enableTimeSettingFromGPS = 0,
    .enableMagneticSwitch = 0,
    .enableLowGainRange = 0,
    .enableFrequencyTrigger = 0,
    .enableDailyFolders = 0,
    .enableSunRecording = 0
};

/* Persistent configuration data structure */

#pragma pack(push, 1)

typedef struct {
    uint8_t firmwareVersion[AM_FIRMWARE_VERSION_LENGTH];
    uint8_t firmwareDescription[AM_FIRMWARE_DESCRIPTION_LENGTH];
    configSettings_t configSettings;
} persistentConfigSettings_t;

#pragma pack(pop)

/* Acoustic location data structure */

#pragma pack(push, 1)

typedef struct {
    int32_t latitude: 28;
    int32_t longitude: 28;
} acousticLocation_t;

#pragma pack(pop)

/* Function to select energy saver mode */

static bool isEnergySaverMode(configSettings_t *configSettings) {

    return configSettings->enableEnergySaverMode && configSettings->sampleRate / configSettings->sampleRateDivider <= ENERGY_SAVER_SAMPLE_RATE_THRESHOLD;

}

/* Functions to format header and configuration components */

static uint32_t formatDecibels(char *dest, uint32_t value, bool space) {

    if (value > 0) return sprintf(dest, space ? "-%lu dB" : "-%ludB", value);

    return sprintf(dest, space ? "0 dB" : "0dB");

}

static uint32_t formatPercentage(char *dest, uint32_t mantissa, int32_t exponent) {

    uint32_t length = exponent < 0 ? 1 - exponent : 0;

    memcpy(dest, "0.0000", length);

    length += sprintf(dest + length, "%lu", mantissa);

    while (exponent-- > 0) dest[length++] = '0';

    dest[length++] = '%';

    return length;

}

/* Functions to set WAV header details and comment */

static void setHeaderDetails(wavHeader_t *wavHeader, uint32_t sampleRate, uint32_t numberOfSamples, uint32_t guanoHeaderSize) {

    wavHeader->wavFormat.samplesPerSecond = sampleRate;
    wavHeader->wavFormat.bytesPerSecond = NUMBER_OF_BYTES_IN_SAMPLE * sampleRate;
    wavHeader->data.size = NUMBER_OF_BYTES_IN_SAMPLE * numberOfSamples;
    wavHeader->riff.size = NUMBER_OF_BYTES_IN_SAMPLE * numberOfSamples + sizeof(wavHeader_t) + guanoHeaderSize - sizeof(chunk_t);

}

static void setHeaderComment(wavHeader_t *wavHeader, configSettings_t *configSettings, uint32_t currentTime, uint8_t *serialNumber, uint8_t *deploymentID, uint8_t *defaultDeploymentID, AM_extendedBatteryState_t extendedBatteryState, int32_t temperature, bool externalMicrophone, AM_recordingState_t recordingState, AM_filterType_t filterType) {

    struct tm time;

    time_t rawTime = currentTime + configSettings->timezoneHours * SECONDS_IN_HOUR + configSettings->timezoneMinutes * SECONDS_IN_MINUTE;

    gmtime_r(&rawTime, &time);

    /* Format artist field */

    char *artist = wavHeader->iart.artist;

    sprintf(artist, "AudioMoth " SERIAL_NUMBER, FORMAT_SERIAL_NUMBER(serialNumber));

    /* Clear comment field */

    char *comment = wavHeader->icmt.comment;

    memset(comment, 0, LENGTH_OF_COMMENT);

    /* Format comment field */

    comment += sprintf(comment, "Recorded at %02d:%02d:%02d %02d/%02d/%04d (UTC", time.tm_hour, time.tm_min, time.tm_sec, time.tm_mday, MONTH_OFFSET + time.tm_mon, YEAR_OFFSET + time.tm_year);

    int8_t timezoneHours = configSettings->timezoneHours;

    int8_t timezoneMinutes = configSettings->timezoneMinutes;

    if (timezoneHours < 0) {

        comment += sprintf(comment, "%d", timezoneHours);

    } else if (timezoneHours > 0) {

        comment += sprintf(comment, "+%d", timezoneHours);

    } else {

        if (timezoneMinutes < 0) comment += sprintf(comment, "-%d", timezoneHours);

        if (timezoneMinutes > 0) comment += sprintf(comment, "+%d", timezoneHours);

    }

    if (timezoneMinutes < 0) comment += sprintf(comment, ":%02d", -timezoneMinutes);

    if (timezoneMinutes > 0) comment += sprintf(comment, ":%02d", timezoneMinutes);

    if (memcmp(deploymentID, defaultDeploymentID, DEPLOYMENT_ID_LENGTH)) {

        comment += sprintf(comment, ") during deployment " SERIAL_NUMBER " ", FORMAT_SERIAL_NUMBER(deploymentID));

    } else {

        comment += sprintf(comment, ") by %s ", artist);

    }

    if (externalMicrophone) {

        comment += sprintf(comment, "using external microphone ");

    }

    static char *gainSettings[5] = {"low", "low-medium", "medium", "medium-high", "high"};

    comment += sprintf(comment, "at %s gain while battery was ", gainSettings[configSettings->gain]);

    if (extendedBatteryState == AM_EXT_BAT_LOW) {

        comment += sprintf(comment, "less than 2.5V");

    } else if (extendedBatteryState >= AM_EXT_BAT_FULL) {

        comment += sprintf(comment, "greater than 4.9V");

    } else {

        uint32_t batteryVoltage =  extendedBatteryState + AM_EXT_BAT_STATE_OFFSET / AM_BATTERY_STATE_INCREMENT;

        comment += sprintf(comment, "%01lu.%01luV", batteryVoltage / 10, batteryVoltage % 10);

    }

    char *temperatureSign = temperature < 0 ? "-" : "";

    uint32_t temperatureInDecidegrees = ROUNDED_DIV(ABS(temperature), 100);

    comment += sprintf(comment, " and temperature was %s%lu.%luC.", temperatureSign, temperatureInDecidegrees / 10, temperatureInDecidegrees % 10);
    
    bool frequencyTriggerEnabled = configSettings->enableFrequencyTrigger;

    bool amplitudeThresholdEnabled = frequencyTriggerEnabled ? false : configSettings->amplitudeThreshold > 0 || configSettings->enableAmplitudeThresholdDecibelScale || configSettings->enableAmplitudeThresholdPercentageScale;

    if (frequencyTriggerEnabled) {

        comment += sprintf(comment, " Frequency trigger (%u.%ukHz and window length of %u samples) threshold was ", configSettings->frequencyTriggerCentreFrequency / 10, configSettings->frequencyTriggerCentreFrequency % 10, (0x01 << configSettings->frequencyTriggerWindowLengthShift));

        comment += formatPercentage(comment, configSettings->frequencyTriggerThresholdPercentageMantissa, configSettings->frequencyTriggerThresholdPercentageExponent);

        comment += sprintf(comment, " with %us minimum trigger duration.", configSettings->minimumTriggerDuration);

    }

    uint16_t lowerFilterFreq = configSettings->lowerFilterFreq;

    uint16_t higherFilterFreq = configSettings->higherFilterFreq;

    if (filterType == LOW_PASS_FILTER) {

        comment += sprintf(comment, " Low-pass filter with frequency of %01u.%01ukHz applied.", higherFilterFreq / 10, higherFilterFreq % 10);

    } else if (filterType == BAND_PASS_FILTER) {

        comment += sprintf(comment, " Band-pass filter with frequencies of %01u.%01ukHz and %01u.%01ukHz applied.", lowerFilterFreq / 10, lowerFilterFreq % 10, higherFilterFreq / 10, higherFilterFreq % 10);

    } else if (filterType == HIGH_PASS_FILTER) {

        comment += sprintf(comment, " High-pass filter with frequency of %01u.%01ukHz applied.", lowerFilterFreq / 10, lowerFilterFreq % 10);

    }

    if (amplitudeThresholdEnabled) {
        
        comment += sprintf(comment, " Amplitude threshold was ");

        if (configSettings->enableAmplitudeThresholdDecibelScale && configSettings->enableAmplitudeThresholdPercentageScale == false) {

            comment += formatDecibels(comment, configSettings->amplitudeThresholdDecibels, true);

        } else if (configSettings->enableAmplitudeThresholdPercentageScale && configSettings->enableAmplitudeThresholdDecibelScale == false) {

            comment += formatPercentage(comment, configSettings->amplitudeThresholdPercentageMantissa, configSettings->amplitudeThresholdPercentageExponent);

        } else {

            comment += sprintf(comment, "%u", configSettings->amplitudeThreshold);

        }

        comment += sprintf(comment, " with %us minimum trigger duration.", configSettings->minimumTriggerDuration);

    }

    if (recordingState != RECORDING_OKAY) {

        comment += sprintf(comment, " Recording stopped");

        if (recordingState == MICROPHONE_CHANGED) {

            comment += sprintf(comment, " due to microphone change.");

        } else if (recordingState == SWITCH_CHANGED) {

            comment += sprintf(comment, " due to switch position change.");

        } else if (recordingState == MAGNETIC_SWITCH) {
        
            comment += sprintf(comment, " by magnetic switch.");

        } else if (recordingState == SUPPLY_VOLTAGE_LOW) {

            comment += sprintf(comment, " due to low voltage.");

        } else if (recordingState == FILE_SIZE_LIMITED) {

            comment += sprintf(comment, " due to file size limit.");

        } else if (recordingState == SDCARD_WRITE_ERROR) {

            comment += sprintf(comment, " due to SD card write error.");

        }

    }

}

/* Function to write the GUANO data */

static uint32_t writeGuanoData(char *buffer, configSettings_t *configSettings, uint32_t currentTime, bool gpsLocationReceived, int32_t *gpsLastFixLatitude, int32_t *gpsLastFixLongitude, bool acousticLocationReceived, int32_t *acousticLatitude, int32_t *acousticLongitude, uint8_t *firmwareDescription, uint8_t *firmwareVersion, uint8_t *serialNumber, uint8_t *deploymentID, uint8_t *defaultDeploymentID, char *filename, AM_extendedBatteryState_t extendedBatteryState, int32_t temperature, AM_filterType_t filterType) {

    uint32_t length = sprintf(buffer, "guan") + UINT32_SIZE_IN_BYTES;

    /* General information */
    
    length += sprintf(buffer + length, "GUANO|Version:1.0\nMake:Open Acoustic Devices\nModel:AudioMoth\nSerial:" SERIAL_NUMBER "\n", FORMAT_SERIAL_NUMBER(serialNumber));

    if (memcmp(deploymentID, defaultDeploymentID, DEPLOYMENT_ID_LENGTH)) {

        length += sprintf(buffer + length, "OAD|Deployment ID:" SERIAL_NUMBER "\n", FORMAT_SERIAL_NUMBER(deploymentID));

    }

    length += sprintf(buffer + length, "Firmware Version:%s (%u.%u.%u)\n", firmwareDescription, firmwareVersion[0], firmwareVersion[1], firmwareVersion[2]);

    /* Timestamp */

    int32_t timezoneOffset = configSettings->timezoneHours * SECONDS_IN_HOUR + configSettings->timezoneMinutes * SECONDS_IN_MINUTE;

    time_t rawTime = currentTime + timezoneOffset;

    struct tm time;

    gmtime_r(&rawTime, &time);

    length += sprintf(buffer + length, "Timestamp:%04d-%02d-%02dT%02d:%02d:%02d", YEAR_OFFSET + time.tm_year, MONTH_OFFSET + time.tm_mon, time.tm_mday, time.tm_hour, time.tm_min, time.tm_sec);

    if (timezoneOffset == 0) {

        length += sprintf(buffer + length, "Z\n");
        
    } else if (timezoneOffset < 0) {

        length += sprintf(buffer + length, "-%02d:%02d\n", ABS(configSettings->timezoneHours), ABS(configSettings->timezoneMinutes));

    } else {

        length += sprintf(buffer + length, "+%02d:%02d\n", configSettings->timezoneHours, configSettings->timezoneMinutes);

    }

    /* Location position and source */

    if (gpsLocationReceived || acousticLocationReceived || configSettings->enableSunRecording) {

        int32_t latitude = gpsLocationReceived ? *gpsLastFixLatitude : acousticLocationReceived ? *acousticLatitude : configSettings->latitude;

        int32_t longitude = gpsLocationReceived ? *gpsLastFixLongitude : acousticLocationReceived ? *acousticLongitude : configSettings->longitude;

        char *latitudeSign = latitude < 0 ? "-" : "";

        char *longitudeSign = longitude < 0 ? "-" : "";

        if (gpsLocationReceived) {

            length += sprintf(buffer + length, "Loc Position:%s%ld.%06ld %s%ld.%06ld\nOAD|Loc Source:GPS\n", latitudeSign, ABS(latitude) / GPS_LOCATION_PRECISION, ABS(latitude) % GPS_LOCATION_PRECISION, longitudeSign, ABS(longitude) / GPS_LOCATION_PRECISION, ABS(longitude) % GPS_LOCATION_PRECISION);

        } else if (acousticLocationReceived) {

            length += sprintf(buffer + length, "Loc Position:%s%ld.%06ld %s%ld.%06ld\nOAD|Loc Source:Acoustic chime\n", latitudeSign, ABS(latitude) / ACOUSTIC_LOCATION_PRECISION, ABS(latitude) % ACOUSTIC_LOCATION_PRECISION, longitudeSign, ABS(longitude) / ACOUSTIC_LOCATION_PRECISION, ABS(longitude) % ACOUSTIC_LOCATION_PRECISION);

        } else {

            length += sprintf(buffer + length, "Loc Position:%s%ld.%02ld %s%ld.%02ld\nOAD|Loc Source:Configuration app\n", latitudeSign, ABS(latitude) / CONFIG_LOCATION_PRECISION, ABS(latitude) % CONFIG_LOCATION_PRECISION, longitudeSign, ABS(longitude) / CONFIG_LOCATION_PRECISION, ABS(longitude) % CONFIG_LOCATION_PRECISION);

        }

    }

    /* Filename */

    char *start = strchr(filename, '/');

    length += sprintf(buffer + length, "Original Filename:%s\n", start ? start + 1 : filename);

    /* Recording settings */

    length += sprintf(buffer + length, "OAD|Recording Settings:%lu GAIN %u", configSettings->sampleRate / configSettings->sampleRateDivider, configSettings->gain);

    bool frequencyTriggerEnabled = configSettings->enableFrequencyTrigger;

    bool amplitudeThresholdEnabled = frequencyTriggerEnabled ? false : configSettings->amplitudeThreshold > 0 || configSettings->enableAmplitudeThresholdDecibelScale || configSettings->enableAmplitudeThresholdPercentageScale;

    if (frequencyTriggerEnabled) {

        length += sprintf(buffer + length, " FREQ %u %u ", FILTER_FREQ_MULTIPLIER * configSettings->frequencyTriggerCentreFrequency, 0x01 << configSettings->frequencyTriggerWindowLengthShift);

        length += formatPercentage(buffer + length, configSettings->frequencyTriggerThresholdPercentageMantissa, configSettings->frequencyTriggerThresholdPercentageExponent);

        length += sprintf(buffer + length, " %u", configSettings->minimumTriggerDuration);

    }

    uint32_t lowerFilterFreq = FILTER_FREQ_MULTIPLIER * configSettings->lowerFilterFreq;

    uint32_t higherFilterFreq = FILTER_FREQ_MULTIPLIER * configSettings->higherFilterFreq;

    if (filterType == LOW_PASS_FILTER) {

        length += sprintf(buffer + length, " LPF %lu", higherFilterFreq);

    } else if (filterType == BAND_PASS_FILTER) {

        length += sprintf(buffer + length, " BPF %lu %lu", lowerFilterFreq, higherFilterFreq);

    } else if (filterType == HIGH_PASS_FILTER) {

        length += sprintf(buffer + length, " HPF %lu", lowerFilterFreq);

    }

    if (amplitudeThresholdEnabled) {

        length += sprintf(buffer + length, " AMP ");

        if (configSettings->enableAmplitudeThresholdDecibelScale && configSettings->enableAmplitudeThresholdPercentageScale == false) {

            length += formatDecibels(buffer + length, configSettings->amplitudeThresholdDecibels, false);

        } else if (configSettings->enableAmplitudeThresholdPercentageScale && configSettings->enableAmplitudeThresholdDecibelScale == false) {

            length += formatPercentage(buffer + length, configSettings->amplitudeThresholdPercentageMantissa, configSettings->amplitudeThresholdPercentageExponent);

        } else {

            length += sprintf(buffer + length, "%u", configSettings->amplitudeThreshold);

        }
        
        length += sprintf(buffer + length, " %u", configSettings->minimumTriggerDuration);

    }

    if (configSettings->enableLowGainRange) length += sprintf(buffer + length, " LGR");

    if (configSettings->disable48HzDCBlockingFilter) length += sprintf(buffer + length, " D48");

    if (isEnergySaverMode(configSettings)) length += sprintf(buffer + length, " ESM");

    /* Battery and temperature */

    uint32_t batteryVoltage = extendedBatteryState == AM_EXT_BAT_LOW ? 24 : extendedBatteryState >= AM_EXT_BAT_FULL ? 50 : extendedBatteryState + AM_EXT_BAT_STATE_OFFSET / AM_BATTERY_STATE_INCREMENT;

    length += sprintf(buffer + length, "\nOAD|Battery Voltage:%01lu.%01lu\n", batteryVoltage / 10, batteryVoltage % 10);
    
    char *temperatureSign = temperature < 0 ? "-" : "";

    uint32_t temperatureInDecidegrees = ROUNDED_DIV(ABS(temperature), 100);

    length += sprintf(buffer + length, "Temperature Int:%s%lu.%lu", temperatureSign, temperatureInDecidegrees / 10, temperatureInDecidegrees % 10);

    /* Set GUANO chunk size */

    *(uint32_t*)(buffer + RIFF_ID_LENGTH) = length - sizeof(chunk_t);;

    return length;

}

/* Function to write configuration to file */

static bool writeConfigurationToFile(configSettings_t *configSettings, uint32_t currentTime, bool gpsLocationReceived, int32_t *gpsLatitude, int32_t *gpsLongitude, bool acousticLocationReceived, int32_t *acousticLatitude, int32_t *acousticLongitude, uint8_t *firmwareDescription, uint8_t *firmwareVersion, uint8_t *serialNumber, uint8_t *deploymentID, uint8_t *defaultDeploymentID) {

    static char configBuffer[CONFIG_BUFFER_LENGTH];

    static char timezoneBuffer[CONFIG_TIMEZONE_LENGTH];

    int32_t timezoneOffset = configSettings->timezoneHours * SECONDS_IN_HOUR + configSettings->timezoneMinutes * SECONDS_IN_MINUTE;

    RETURN_BOOL_ON_ERROR(AudioMoth_openFile("CONFIG.TXT"));

    uint32_t length = sprintf(configBuffer, "Device ID                       : " SERIAL_NUMBER "\r\n", FORMAT_SERIAL_NUMBER(serialNumber));

    length += sprintf(configBuffer + length, "Firmware                        : %s (%u.%u.%u)\r\n\r\n", firmwareDescription, firmwareVersion[0], firmwareVersion[1], firmwareVersion[2]);

    if (memcmp(deploymentID, defaultDeploymentID, DEPLOYMENT_ID_LENGTH)) {

        length += sprintf(configBuffer + length, "Deployment ID                   : " SERIAL_NUMBER "\r\n\r\n", FORMAT_SERIAL_NUMBER(deploymentID));

    }

    uint32_t timezoneLength = sprintf(timezoneBuffer, "UTC");

    if (configSettings->timezoneHours < 0) {

        timezoneLength += sprintf(timezoneBuffer + timezoneLength, "%d", configSettings->timezoneHours);

    } else if (configSettings->timezoneHours > 0) {

        timezoneLength += sprintf(timezoneBuffer + timezoneLength, "+%d", configSettings->timezoneHours);

    } else {

        if (configSettings->timezoneMinutes < 0) timezoneLength += sprintf(timezoneBuffer + timezoneLength, "-%d", configSettings->timezoneHours);

        if (configSettings->timezoneMinutes > 0) timezoneLength += sprintf(timezoneBuffer + timezoneLength, "+%d", configSettings->timezoneHours);

    }

    if (configSettings->timezoneMinutes < 0) timezoneLength += sprintf(timezoneBuffer + timezoneLength, ":%02d", -configSettings->timezoneMinutes);

    if (configSettings->timezoneMinutes > 0) timezoneLength += sprintf(timezoneBuffer + timezoneLength, ":%02d", configSettings->timezoneMinutes);

    time_t rawTime = currentTime + timezoneOffset;

    struct tm time;

    gmtime_r(&rawTime, &time);

    length += sprintf(configBuffer + length, "Device time                     : %04d-%02d-%02d %02d:%02d:%02d (%s)", YEAR_OFFSET + time.tm_year, MONTH_OFFSET + time.tm_mon, time.tm_mday, time.tm_hour, time.tm_min, time.tm_sec, timezoneBuffer);

    RETURN_BOOL_ON_ERROR(AudioMoth_writeToFile(configBuffer, length));

    length = sprintf(configBuffer, "\r\n\r\nSample rate (Hz)                : %lu\r\n", configSettings->sampleRate / configSettings->sampleRateDivider);

    static char *gainSettings[5] = {"Low", "Low-Medium", "Medium", "Medium-High", "High"};

    length += sprintf(configBuffer + length, "Gain                            : %s\r\n\r\n", gainSettings[configSettings->gain]);

    length += sprintf(configBuffer + length, "Sleep duration (s)              : ");

    if (configSettings->disableSleepRecordCycle) {

        length += sprintf(configBuffer + length, "-");

    } else {

        length += sprintf(configBuffer + length, "%u", configSettings->sleepDuration);

    }

    length += sprintf(configBuffer + length, "\r\nRecording duration (s)          : ");

    if (configSettings->disableSleepRecordCycle) {

        length += sprintf(configBuffer + length, "-");

    } else {

        length += sprintf(configBuffer + length, "%u", configSettings->recordDuration);

    }

    RETURN_BOOL_ON_ERROR(AudioMoth_writeToFile(configBuffer, length));

    if (configSettings->enableSunRecording) {

        int32_t latitude = gpsLocationReceived ? *gpsLatitude : acousticLocationReceived ? *acousticLatitude : configSettings->latitude;

        int32_t longitude = gpsLocationReceived ? *gpsLongitude : acousticLocationReceived ? *acousticLongitude : configSettings->longitude;

        char latitudeDirection = latitude < 0 ? 'S' : 'N';

        char longitudeDirection = longitude < 0 ? 'W' : 'E';

        if (gpsLocationReceived) {

            length = sprintf(configBuffer, "\r\n\r\nLocation                        : %ld.%06ld°%c %ld.%06ld°%c (GPS)", ABS(latitude) / GPS_LOCATION_PRECISION, ABS(latitude) % GPS_LOCATION_PRECISION, latitudeDirection, ABS(longitude) / GPS_LOCATION_PRECISION, ABS(longitude) % GPS_LOCATION_PRECISION, longitudeDirection);

        } else if (acousticLocationReceived) {

            length = sprintf(configBuffer, "\r\n\r\nLocation                        : %ld.%06ld°%c %ld.%06ld°%c (Acoustic chime)", ABS(latitude) / ACOUSTIC_LOCATION_PRECISION, ABS(latitude) % ACOUSTIC_LOCATION_PRECISION, latitudeDirection, ABS(longitude) / ACOUSTIC_LOCATION_PRECISION, ABS(longitude) % ACOUSTIC_LOCATION_PRECISION, longitudeDirection);

        } else {

            length = sprintf(configBuffer, "\r\n\r\nLocation                        : %ld.%02ld°%c %ld.%02ld°%c (Configuration app)", ABS(latitude) / CONFIG_LOCATION_PRECISION, ABS(latitude) % CONFIG_LOCATION_PRECISION, latitudeDirection, ABS(longitude) / CONFIG_LOCATION_PRECISION, ABS(longitude) % CONFIG_LOCATION_PRECISION, longitudeDirection);

        }

        static char* twilightTypes[3] = {"Civil", "Nautical", "Astronomical"};

        static char* dawnDuskModes[5] = {"dawn", "dusk", "dawn and dusk", "dusk to dawn", "dawn to dusk"};

        static char* sunriseSunsetModes[5] = {"Sunrise", "Sunset", "Sunrise and sunset", "Sunset to sunrise", "Sunrise to sunset"};

        length += sprintf(configBuffer + length, "\r\nSun recording mode              : ");
        
        if (configSettings->sunRecordingEvent == 0) {

            length += sprintf(configBuffer + length, "%s", sunriseSunsetModes[configSettings->sunRecordingMode]);

        } else {

            length += sprintf(configBuffer + length, "%s %s", twilightTypes[configSettings->sunRecordingEvent - 1], dawnDuskModes[configSettings->sunRecordingMode]);

        }

        char *sunriseText = configSettings->sunRecordingEvent == 0 ? "\r\nSunrise - before, after (mins)  " : "\r\nDawn - before, after (mins)     ";

        char *sunsetText = configSettings->sunRecordingEvent == 0 ? "\r\nSunset - before, after (mins)   " : "\r\nDusk - before, after (mins)     ";

        if (configSettings->sunRecordingMode == SUNRISE_RECORDING) {

            length += sprintf(configBuffer + length, "%s: %u, %u", sunriseText, configSettings->beforeSunriseMinutes, configSettings->afterSunriseMinutes);
            length += sprintf(configBuffer + length, "%s: -, -", sunsetText);
            
        } else if (configSettings->sunRecordingMode == SUNSET_RECORDING) {

            length += sprintf(configBuffer + length, "%s: -, -", sunriseText);
            length += sprintf(configBuffer + length, "%s: %u, %u", sunsetText, configSettings->beforeSunsetMinutes, configSettings->afterSunsetMinutes);

        } else if (configSettings->sunRecordingMode == SUNRISE_AND_SUNSET_RECORDING) {

            length += sprintf(configBuffer + length, "%s: %u, %u", sunriseText, configSettings->beforeSunriseMinutes, configSettings->afterSunriseMinutes);
            length += sprintf(configBuffer + length, "%s: %u, %u", sunsetText, configSettings->beforeSunsetMinutes, configSettings->afterSunsetMinutes);

        } else if (configSettings->sunRecordingMode == SUNSET_TO_SUNRISE_RECORDING) {

            length += sprintf(configBuffer + length, "%s: -, %u", sunriseText, configSettings->afterSunriseMinutes);
            length += sprintf(configBuffer + length, "%s: %u, -", sunsetText, configSettings->beforeSunsetMinutes);

        } else if (configSettings->sunRecordingMode == SUNRISE_TO_SUNSET_RECORDING) {

            length += sprintf(configBuffer + length, "%s: %u, -", sunriseText, configSettings->beforeSunriseMinutes);
            length += sprintf(configBuffer + length, "%s: -, %u", sunsetText, configSettings->afterSunsetMinutes);

        }

        char *roundingText = configSettings->sunRecordingEvent == 0 ? "\r\nSunrise/sunset rounding (mins)  : %u" : "\r\nDawn/dusk rounding (mins)       : %u";

        length += sprintf(configBuffer + length, roundingText, configSettings->sunRoundingMinutes);

    } else {

        length = sprintf(configBuffer, "\r\n\r\nActive recording periods        : %u\r\n", configSettings->activeRecordingPeriods);

        /* Find the first recording period */

        uint32_t minimumIndex = 0;

        uint32_t minimumStartMinutes = UINT32_MAX;

        for (uint32_t i = 0; i < configSettings->activeRecordingPeriods; i += 1) {

            uint32_t startMinutes = (MINUTES_IN_DAY + configSettings->recordingPeriods[i].startMinutes + timezoneOffset / SECONDS_IN_MINUTE) % MINUTES_IN_DAY;

            if (startMinutes < minimumStartMinutes) {

                minimumStartMinutes = startMinutes;

                minimumIndex = i;

            }

        }
    
        /* Display the recording periods */

        for (uint32_t i = 0; i < configSettings->activeRecordingPeriods; i += 1) {

            uint32_t index = (minimumIndex + i) % configSettings->activeRecordingPeriods;

            uint32_t startMinutes = (MINUTES_IN_DAY + configSettings->recordingPeriods[index].startMinutes + timezoneOffset / SECONDS_IN_MINUTE) % MINUTES_IN_DAY;

            uint32_t endMinutes = (MINUTES_IN_DAY + configSettings->recordingPeriods[index].endMinutes + timezoneOffset / SECONDS_IN_MINUTE) % MINUTES_IN_DAY;

            length += sprintf(configBuffer + length, "\r\nRecording period %lu              : %02lu:%02lu - %02lu:%02lu (%s)", i + 1, startMinutes / MINUTES_IN_HOUR, startMinutes % MINUTES_IN_HOUR, endMinutes / MINUTES_IN_HOUR, endMinutes % MINUTES_IN_HOUR, timezoneBuffer);

        }

    }

    RETURN_BOOL_ON_ERROR(AudioMoth_writeToFile(configBuffer, length));

    if (configSettings->earliestRecordingTime == 0) {

        length = sprintf(configBuffer, "\r\n\r\nFirst recording date            : ----------");

    } else {

        time_t rawTime = configSettings->earliestRecordingTime + timezoneOffset;

        gmtime_r(&rawTime, &time);

        if (time.tm_hour == 0 && time.tm_min == 0 && time.tm_sec == 0) {

            length = sprintf(configBuffer, "\r\n\r\nFirst recording date            : ");

            length += sprintf(configBuffer + length, "%04d-%02d-%02d (%s)", YEAR_OFFSET + time.tm_year, MONTH_OFFSET + time.tm_mon, time.tm_mday, timezoneBuffer);

        } else {

            length = sprintf(configBuffer, "\r\n\r\nFirst recording time            : ");

            length += sprintf(configBuffer + length, "%04d-%02d-%02d %02d:%02d:%02d (%s)", YEAR_OFFSET + time.tm_year, MONTH_OFFSET + time.tm_mon, time.tm_mday, time.tm_hour, time.tm_min, time.tm_sec, timezoneBuffer);

        } 

    }
                                              
    if (configSettings->latestRecordingTime == 0) {

        length += sprintf(configBuffer + length, "\r\nLast recording date             : ----------");

    } else {

        time_t rawTime = configSettings->latestRecordingTime + timezoneOffset;

        gmtime_r(&rawTime, &time);

        if (time.tm_hour == 0 && time.tm_min == 0 && time.tm_sec == 0) {

            rawTime -= SECONDS_IN_DAY;

            gmtime_r(&rawTime, &time);

            length += sprintf(configBuffer + length, "\r\nLast recording date             : ");

            length += sprintf(configBuffer + length, "%04d-%02d-%02d (%s)", YEAR_OFFSET + time.tm_year, MONTH_OFFSET + time.tm_mon, time.tm_mday, timezoneBuffer);

        } else {

            length += sprintf(configBuffer + length, "\r\nLast recording time             : ");

            length += sprintf(configBuffer + length, "%04d-%02d-%02d %02d:%02d:%02d (%s)", YEAR_OFFSET + time.tm_year, MONTH_OFFSET + time.tm_mon, time.tm_mday, time.tm_hour, time.tm_min, time.tm_sec, timezoneBuffer);

        }

    }

    RETURN_BOOL_ON_ERROR(AudioMoth_writeToFile(configBuffer, length));

    length = sprintf(configBuffer, "\r\n\r\nFilter                          : ");

    if (configSettings->lowerFilterFreq == 0 && configSettings->higherFilterFreq == 0) {

        length += sprintf(configBuffer + length, "-");

    } else if (configSettings->lowerFilterFreq == UINT16_MAX) {

        length += sprintf(configBuffer + length, "Low-pass (%u.%ukHz)", configSettings->higherFilterFreq / 10, configSettings->higherFilterFreq % 10);

    } else if (configSettings->higherFilterFreq == UINT16_MAX) {

        length += sprintf(configBuffer + length, "High-pass (%u.%ukHz)", configSettings->lowerFilterFreq / 10, configSettings->lowerFilterFreq % 10);

    } else {

        length += sprintf(configBuffer + length, "Band-pass (%u.%ukHz - %u.%ukHz)", configSettings->lowerFilterFreq / 10, configSettings->lowerFilterFreq % 10, configSettings->higherFilterFreq / 10, configSettings->higherFilterFreq % 10);

    }

    bool frequencyTriggerEnabled = configSettings->enableFrequencyTrigger;

    bool amplitudeThresholdEnabled = frequencyTriggerEnabled ? false : configSettings->amplitudeThreshold > 0 || configSettings->enableAmplitudeThresholdDecibelScale || configSettings->enableAmplitudeThresholdPercentageScale;

    length += sprintf(configBuffer + length, "\r\n\r\nTrigger type                    : ");

    if (frequencyTriggerEnabled) {

        length += sprintf(configBuffer + length, "Frequency (%u.%ukHz and window length of %u samples)", configSettings->frequencyTriggerCentreFrequency / 10, configSettings->frequencyTriggerCentreFrequency % 10, (0x01 << configSettings->frequencyTriggerWindowLengthShift));

        length += sprintf(configBuffer + length, "\r\nThreshold setting               : ");

        length += formatPercentage(configBuffer + length, configSettings->frequencyTriggerThresholdPercentageMantissa, configSettings->frequencyTriggerThresholdPercentageExponent);

    } else if (amplitudeThresholdEnabled) {

        length += sprintf(configBuffer + length, "Amplitude");

        length += sprintf(configBuffer + length, "\r\nThreshold setting               : ");

        if (configSettings->enableAmplitudeThresholdDecibelScale && configSettings->enableAmplitudeThresholdPercentageScale == false) {

            length += formatDecibels(configBuffer + length, configSettings->amplitudeThresholdDecibels, true);

        } else if (configSettings->enableAmplitudeThresholdPercentageScale && configSettings->enableAmplitudeThresholdDecibelScale == false) {

            length += formatPercentage(configBuffer + length, configSettings->amplitudeThresholdPercentageMantissa, configSettings->amplitudeThresholdPercentageExponent);

        } else {

            length += sprintf(configBuffer + length, "%u", configSettings->amplitudeThreshold);

        }

    } else {

        length += sprintf(configBuffer + length, "-");

        length += sprintf(configBuffer + length, "\r\nThreshold setting               : -");

    }

    length += sprintf(configBuffer + length, "\r\nMinimum trigger duration (s)    : ");

    if (frequencyTriggerEnabled || amplitudeThresholdEnabled) {

        length += sprintf(configBuffer + length, "%u", configSettings->minimumTriggerDuration);

    } else {

        length += sprintf(configBuffer + length, "-");

    }

    RETURN_BOOL_ON_ERROR(AudioMoth_writeToFile(configBuffer, length));

    length = sprintf(configBuffer, "\r\n\r\nEnable LED                      : %s\r\n", configSettings->enableLED ? "Yes" : "No");

    length += sprintf(configBuffer + length, "Enable low-voltage cut-off      : %s\r\n", configSettings->enableLowVoltageCutoff ? "Yes" : "No");

    length += sprintf(configBuffer + length, "Enable battery level indication : %s\r\n\r\n", configSettings->disableBatteryLevelDisplay ? "No" : configSettings->batteryLevelDisplayType == NIMH_LIPO_BATTERY_VOLTAGE ? "Yes (NiMH/LiPo voltage range)" : "Yes");

    length += sprintf(configBuffer + length, "Always require acoustic chime   : %s\r\n", configSettings->requireAcousticConfiguration ? "Yes" : "No");

    length += sprintf(configBuffer + length, "Use device ID in WAV file name  : %s\r\n", configSettings->enableFilenameWithDeviceID ? "Yes" : "No");

    length += sprintf(configBuffer + length, "Use daily folder for WAV files  : %s\r\n\r\n", configSettings->enableDailyFolders ? "Yes" : "No");

    length += sprintf(configBuffer + length, "Disable 48Hz DC blocking filter : %s\r\n", configSettings->disable48HzDCBlockingFilter ? "Yes" : "No");

    length += sprintf(configBuffer + length, "Enable energy saver mode        : %s\r\n", configSettings->enableEnergySaverMode ? "Yes" : "No");

    length += sprintf(configBuffer + length, "Enable low gain range           : %s\r\n\r\n", configSettings->enableLowGainRange ? "Yes" : "No");

    length += sprintf(configBuffer + length, "Enable magnetic switch          : %s\r\n\r\n", configSettings->enableMagneticSwitch ? "Yes" : "No");

    RETURN_BOOL_ON_ERROR(AudioMoth_writeToFile(configBuffer, length));

    length = sprintf(configBuffer, "Enable GPS time setting         : %s\r\n", configSettings->enableTimeSettingFromGPS ? "Yes" : "No");

    length += sprintf(configBuffer + length, "GPS fix before and after        : %s\r\n", configSettings->enableTimeSettingFromGPS == false ? "-" : configSettings->enableTimeSettingBeforeAndAfterRecordings ? "Individual recordings" : "Recording periods");

    length += sprintf(configBuffer + length, "GPS fix time (mins)             : ");

    if (configSettings->enableTimeSettingFromGPS) {

        uint32_t gpsTimeSettingPeriod = configSettings->gpsTimeSettingPeriod == 0 ? GPS_DEFAULT_TIME_SETTING_PERIOD / SECONDS_IN_MINUTE : configSettings->gpsTimeSettingPeriod;

        length += sprintf(configBuffer + length, "%ld\r\n", gpsTimeSettingPeriod);

    } else {

        length += sprintf(configBuffer + length, "-\r\n");

    }

    RETURN_BOOL_ON_ERROR(AudioMoth_writeToFile(configBuffer, length));

    RETURN_BOOL_ON_ERROR(AudioMoth_closeFile());

    return true;

}

/* Backup domain variables */

static uint32_t *backupDomainFlags = (uint32_t*)AM_BACKUP_DOMAIN_START_ADDRESS;

static uint32_t *previousSwitchPosition = (uint32_t*)(AM_BACKUP_DOMAIN_START_ADDRESS + 4);

static uint32_t *startOfRecordingPeriod = (uint32_t*)(AM_BACKUP_DOMAIN_START_ADDRESS + 8);

static uint32_t *timeOfNextRecording = (uint32_t*)(AM_BACKUP_DOMAIN_START_ADDRESS + 12);

static uint32_t *indexOfNextRecording = (uint32_t*)(AM_BACKUP_DOMAIN_START_ADDRESS + 16);

static uint32_t *durationOfNextRecording = (uint32_t*)(AM_BACKUP_DOMAIN_START_ADDRESS + 20);

static uint32_t *timeOfNextGPSTimeSetting = (uint32_t*)(AM_BACKUP_DOMAIN_START_ADDRESS + 24);

static uint8_t *deploymentID = (uint8_t*)(AM_BACKUP_DOMAIN_START_ADDRESS + 28);

static uint32_t *numberOfRecordingErrors = (uint32_t*)(AM_BACKUP_DOMAIN_START_ADDRESS + 36);

static uint32_t *recordingPreparationPeriod = (uint32_t*)(AM_BACKUP_DOMAIN_START_ADDRESS + 40);

static int32_t *gpsLatitude = (int32_t*)(AM_BACKUP_DOMAIN_START_ADDRESS + 44);

static int32_t *gpsLongitude = (int32_t*)(AM_BACKUP_DOMAIN_START_ADDRESS + 48);

static int32_t *gpsLastFixLatitude = (int32_t*)(AM_BACKUP_DOMAIN_START_ADDRESS + 52);

static int32_t *gpsLastFixLongitude = (int32_t*)(AM_BACKUP_DOMAIN_START_ADDRESS + 56);

static int32_t *acousticLatitude = (int32_t*)(AM_BACKUP_DOMAIN_START_ADDRESS + 60);

static int32_t *acousticLongitude = (int32_t*)(AM_BACKUP_DOMAIN_START_ADDRESS + 64);

static uint32_t *numberOfSunRecordingPeriods = (uint32_t*)(AM_BACKUP_DOMAIN_START_ADDRESS + 68);

static recordingPeriod_t *firstSunRecordingPeriod = (recordingPeriod_t*)(AM_BACKUP_DOMAIN_START_ADDRESS + 72);

static recordingPeriod_t *secondSunRecordingPeriod = (recordingPeriod_t*)(AM_BACKUP_DOMAIN_START_ADDRESS + 76);

static uint32_t *timeOfNextSunriseSunsetCalculation = (uint32_t*)(AM_BACKUP_DOMAIN_START_ADDRESS + 80);

static configSettings_t *configSettings = (configSettings_t*)(AM_BACKUP_DOMAIN_START_ADDRESS + 84);

/* Functions to query, set and clear backup domain flags */

typedef enum {
    BACKUP_WRITTEN_CONFIGURATION_TO_FILE,
    BACKUP_READY_TO_MAKE_RECORDING, 
    BACKUP_MUST_SET_TIME_FROM_GPS, 
    BACKUP_SHOULD_SET_TIME_FROM_GPS, 
    BACKUP_WAITING_FOR_MAGNETIC_SWITCH,
    BACKUP_POWERED_DOWN_WITH_SHORT_WAIT_INTERVAL,
    BACKUP_GPS_LOCATION_RECEIVED,
    BACKUP_ACOUSTIC_LOCATION_RECEIVED
} AM_backupDomainFlag_t;

static inline bool getBackupFlag(AM_backupDomainFlag_t flag) {
    uint32_t mask = (1 << flag);
    return (*backupDomainFlags & mask) == mask;
}

static inline void setBackupFlag(AM_backupDomainFlag_t flag, bool state) {
    if (state) {
        *backupDomainFlags |= (1 << flag);
    } else {
        *backupDomainFlags &= ~(1 << flag);
    }
}

/* Filter variables */

static AM_filterType_t requestedFilterType;

/* DMA transfer variable */

static uint32_t numberOfRawSamplesInDMATransfer;

/* SRAM buffer variables */

static volatile uint32_t writeBuffer;

static volatile uint32_t writeBufferIndex;

static int16_t* buffers[NUMBER_OF_BUFFERS];

/* Flag to start processing DMA transfers */

static volatile uint32_t numberOfDMATransfers;

static volatile uint32_t numberOfDMATransfersToWait;

/* Write indicator buffer */

static bool writeIndicator[NUMBER_OF_BUFFERS];

/* Compression buffer */

static int16_t compressionBuffer[COMPRESSION_BUFFER_SIZE_IN_BYTES / NUMBER_OF_BYTES_IN_SAMPLE];

/* GPS fix variables */

static bool gpsEnableLED;

static bool gpsPPSEvent;

static bool gpsFixEvent;

static bool gpsMessageEvent;

static bool gpsFirstMessageReceived;

static uint32_t gpsTickEventCount = 1;

static uint32_t gpsTickEventModulo = GPS_TICK_EVENTS_PER_SECOND;

/* Audio configuration variables */

static bool audioConfigStateLED;

static bool audioConfigToggleLED;

static uint32_t audioConfigPulseCounter;

static bool acousticConfigurationPerformed;

static uint32_t secondsOfAcousticSignalStart;

static uint32_t millisecondsOfAcousticSignalStart;

/* Deployment ID variable */

static uint8_t defaultDeploymentID[DEPLOYMENT_ID_LENGTH];

/* Recording state */

static volatile bool magneticSwitch;

static volatile bool microphoneChanged;

static volatile bool switchPositionChanged;

/* DMA buffers */

static int16_t primaryBuffer[MAXIMUM_SAMPLES_IN_DMA_TRANSFER];

static int16_t secondaryBuffer[MAXIMUM_SAMPLES_IN_DMA_TRANSFER];

/* Firmware version and description */

static uint8_t firmwareVersion[AM_FIRMWARE_VERSION_LENGTH] = {1, 11, 0};

static uint8_t firmwareDescription[AM_FIRMWARE_DESCRIPTION_LENGTH] = "AudioMoth-Firmware-Basic";

/* Function prototypes */

static AM_recordingState_t makeRecording(uint32_t timeOfNextRecording, uint32_t recordDuration, bool enableLED, AM_extendedBatteryState_t extendedBatteryState, int32_t temperature, uint32_t *fileOpenTime, uint32_t *fileOpenMilliseconds);

static void scheduleRecording(uint32_t currentTime, uint32_t *timeOfNextRecording, uint32_t *indexOfNextRecording, uint32_t *durationOfNextRecording, uint32_t *startOfRecordingPeriod, uint32_t *endOfRecordingPeriod);

static void determineTimeOfNextSunriseSunsetCalculation(uint32_t currentTime, uint32_t *timeOfNextSunriseSunsetCalculation);

static void determineSunriseAndSunsetTimesAndScheduleRecording(uint32_t currentTime);

static void determineSunriseAndSunsetTimes(uint32_t currentTime);

static void flashLedToIndicateBatteryLife(void);

/* Functions of copy to and from the backup domain */

static void copyFromBackupDomain(uint8_t *dst, uint32_t *src, uint32_t length) {

    for (uint32_t i = 0; i < length; i += 1) {
        *(dst + i) = *((uint8_t*)src + i);
    }

}

static void copyToBackupDomain(uint32_t *dst, uint8_t *src, uint32_t length) {

    uint32_t value = 0;

    for (uint32_t i = 0; i < length / UINT32_SIZE_IN_BYTES; i += 1) {
        *(dst + i) = *((uint32_t*)src + i);
    }

    for (uint32_t i = 0; i < length % UINT32_SIZE_IN_BYTES; i += 1) {
        value = (value << BITS_PER_BYTE) + *(src + length - 1 - i);
    }

    if (length % UINT32_SIZE_IN_BYTES) *(dst + length / UINT32_SIZE_IN_BYTES) = value;

}

/* GPS time setting functions */

static void writeGPSLogMessage(uint32_t currentTime, uint32_t currentMilliseconds, char *message) {

    static char logBuffer[256];

    struct tm time;

    time_t rawTime = currentTime;

    gmtime_r(&rawTime, &time);

    uint32_t length = sprintf(logBuffer, "%02d/%02d/%04d %02d:%02d:%02d.%03lu UTC: %s\r\n", time.tm_mday, MONTH_OFFSET + time.tm_mon, YEAR_OFFSET + time.tm_year, time.tm_hour, time.tm_min, time.tm_sec, currentMilliseconds, message);

    AudioMoth_writeToFile(logBuffer, length);

}

static GPS_fixResult_t setTimeFromGPS(bool enableLED, AM_gpsFixMode_t fixMode, uint32_t timeout) {

    /* Enable GPS and get the current time */

    GPS_powerUpGPS();

    GPS_enableGPSInterface();

    uint32_t currentTime, currentMilliseconds;

    /* Open the GPS log file */

    bool success = AudioMoth_appendFile(GPS_FILENAME);

    /* Add power up message */

    static char *messages[] = {
        "GPS switched on for initial fix.",
        "GPS switched on for fix before recording period.",
        "GPS switched on for fix between recording periods.",
        "GPS switched on for fix after recording period.",
        "GPS switched on for fix before recording.",
        "GPS switched on for fix between recordings.",
        "GPS switched on for fix after recording."
    };

    char *message = messages[fixMode];

    AudioMoth_getTime(&currentTime, &currentMilliseconds);

    writeGPSLogMessage(currentTime, currentMilliseconds, message);

    /* Set green LED and enter routine */

    gpsEnableLED = enableLED;

    if (gpsEnableLED) AudioMoth_setGreenLED(true);

    GPS_fixResult_t result = GPS_setTimeFromGPS(timeout);

    AudioMoth_setGreenLED(false);

    AudioMoth_setRedLED(false);

    /* Disable the GPS */

    GPS_disableGPSInterface();

    GPS_powerDownGPS();

    /* Add result message */

    AudioMoth_getTime(&currentTime, &currentMilliseconds);

    if (result == GPS_CANCELLED_BY_MAGNETIC_SWITCH) writeGPSLogMessage(currentTime, currentMilliseconds, "Time was not updated. Cancelled by magnetic switch.");
 
    if (result == GPS_CANCELLED_BY_SWITCH) writeGPSLogMessage(currentTime, currentMilliseconds, "Time was not updated. Cancelled by switch position change.");

    if (result == GPS_TIMEOUT) writeGPSLogMessage(currentTime, currentMilliseconds, "Time was not updated. Timed out.");

    writeGPSLogMessage(currentTime, currentMilliseconds, "GPS switched off.\r\n");

    if (success) AudioMoth_closeFile();

    return result;

}

/* Magnetic switch wait functions */

static void startWaitingForMagneticSwitch() {

    /* Flash LED to indicate start of waiting for magnetic switch */

    FLASH_REPEAT_LED(Red, MAGNETIC_SWITCH_CHANGE_FLASHES, SHORT_LED_FLASH_DURATION);

    /* Cancel any scheduled recording */

    *indexOfNextRecording = 0;

    *timeOfNextRecording = UINT32_MAX;

    *startOfRecordingPeriod = UINT32_MAX;

    *durationOfNextRecording = UINT32_MAX;

    *timeOfNextGPSTimeSetting = UINT32_MAX;

    setBackupFlag(BACKUP_SHOULD_SET_TIME_FROM_GPS, false);

    setBackupFlag(BACKUP_WAITING_FOR_MAGNETIC_SWITCH, true);

}

static void stopWaitingForMagneticSwitch(uint32_t *currentTime, uint32_t *currentMilliseconds) {

    /* Flash LED to indicate end of waiting for magnetic switch */

    FLASH_REPEAT_LED(Green, MAGNETIC_SWITCH_CHANGE_FLASHES, SHORT_LED_FLASH_DURATION);

    /* Schedule next recording */

    AudioMoth_getTime(currentTime, currentMilliseconds);

    uint32_t scheduleTime = *currentTime + ROUNDED_UP_DIV(*currentMilliseconds + *recordingPreparationPeriod, MILLISECONDS_IN_SECOND);

    determineSunriseAndSunsetTimesAndScheduleRecording(scheduleTime);

    setBackupFlag(BACKUP_WAITING_FOR_MAGNETIC_SWITCH, false);

}

/* Function to calculate the time to the next event */

static void calculateTimeToNextEvent(uint32_t currentTime, uint32_t currentMilliseconds, int64_t *timeUntilPreparationStart, int64_t *timeUntilNextGPSTimeSetting) {

    *timeUntilPreparationStart = (int64_t)*timeOfNextRecording * MILLISECONDS_IN_SECOND - (int64_t)*recordingPreparationPeriod - (int64_t)currentTime * MILLISECONDS_IN_SECOND - (int64_t)currentMilliseconds;

    *timeUntilNextGPSTimeSetting = (int64_t)*timeOfNextGPSTimeSetting * MILLISECONDS_IN_SECOND - (int64_t)currentTime * MILLISECONDS_IN_SECOND - (int64_t)currentMilliseconds;

}

/* Main function */

int main(void) {

    /* Initialise device */

    AudioMoth_initialise();

    AM_switchPosition_t switchPosition = AudioMoth_getSwitchPosition();

    if (AudioMoth_isInitialPowerUp()) {
        
        /* Initialise recording schedule variables */

        *indexOfNextRecording = 0;

        *timeOfNextRecording = UINT32_MAX;

        *startOfRecordingPeriod = UINT32_MAX;

        *durationOfNextRecording = UINT32_MAX;

        *timeOfNextGPSTimeSetting = UINT32_MAX;

        /* Initialise configuration writing variable */

        setBackupFlag(BACKUP_WRITTEN_CONFIGURATION_TO_FILE, false);

        /* Initialise recording state variables */

        *previousSwitchPosition = AM_SWITCH_NONE;

        setBackupFlag(BACKUP_MUST_SET_TIME_FROM_GPS, false);

        setBackupFlag(BACKUP_SHOULD_SET_TIME_FROM_GPS, false);

        setBackupFlag(BACKUP_READY_TO_MAKE_RECORDING, false);

        *numberOfRecordingErrors = 0;

        *recordingPreparationPeriod = INITIAL_PREPARATION_PERIOD;

        /* Initialise magnetic switch state variables */

        setBackupFlag(BACKUP_WAITING_FOR_MAGNETIC_SWITCH, false);

        /* Initialise the power down interval flag */

        setBackupFlag(BACKUP_POWERED_DOWN_WITH_SHORT_WAIT_INTERVAL, false);

        /* Initial GPS and sunrise and sunset variables */

        *gpsLastFixLatitude = 0;

        *gpsLastFixLongitude = 0;

        setBackupFlag(BACKUP_GPS_LOCATION_RECEIVED, false);

        setBackupFlag(BACKUP_ACOUSTIC_LOCATION_RECEIVED, false);

        *timeOfNextSunriseSunsetCalculation = 0;

        /* Copy default deployment ID */

        copyToBackupDomain((uint32_t*)deploymentID, (uint8_t*)defaultDeploymentID, DEPLOYMENT_ID_LENGTH);

        /* Check the persistent configuration */

        persistentConfigSettings_t *persistentConfigSettings = (persistentConfigSettings_t*)AM_FLASH_USER_DATA_ADDRESS;

        if (memcmp(persistentConfigSettings->firmwareVersion, firmwareVersion, AM_FIRMWARE_VERSION_LENGTH) == 0 && memcmp(persistentConfigSettings->firmwareDescription, firmwareDescription, AM_FIRMWARE_DESCRIPTION_LENGTH) == 0) {

            copyToBackupDomain((uint32_t*)configSettings, (uint8_t*)&persistentConfigSettings->configSettings, sizeof(configSettings_t));

        } else {

            copyToBackupDomain((uint32_t*)configSettings, (uint8_t*)&defaultConfigSettings, sizeof(configSettings_t));

        }

    }

    /* Handle the case that the switch is in USB position  */

    if (switchPosition == AM_SWITCH_USB) {

        if (configSettings->disableBatteryLevelDisplay == false && (*previousSwitchPosition == AM_SWITCH_DEFAULT || *previousSwitchPosition == AM_SWITCH_CUSTOM)) {

            flashLedToIndicateBatteryLife();

        }

        AudioMoth_handleUSB();

        SAVE_SWITCH_POSITION_AND_POWER_DOWN(DEFAULT_WAIT_INTERVAL);

    }

    /* Read the time */

    uint32_t currentTime;

    uint32_t currentMilliseconds;

    AudioMoth_getTime(&currentTime, &currentMilliseconds);

    /* Check if switch has just been moved to CUSTOM or DEFAULT */

    bool fileSystemEnabled = false;

    if (switchPosition != *previousSwitchPosition) {

        /* Reset the GPS flags */

        setBackupFlag(BACKUP_MUST_SET_TIME_FROM_GPS, false);

        setBackupFlag(BACKUP_SHOULD_SET_TIME_FROM_GPS, false);

        /* Reset the power down interval flag */

        setBackupFlag(BACKUP_POWERED_DOWN_WITH_SHORT_WAIT_INTERVAL, false);

        /* Check there are active recording periods if the switch is in CUSTOM position */

        setBackupFlag(BACKUP_READY_TO_MAKE_RECORDING, switchPosition == AM_SWITCH_DEFAULT || (switchPosition == AM_SWITCH_CUSTOM && (configSettings->activeRecordingPeriods > 0 || configSettings->enableSunRecording)));

        /* Check if acoustic configuration is required */

        if (getBackupFlag(BACKUP_READY_TO_MAKE_RECORDING)) {

            /* Determine if acoustic configuration is required */

            bool shouldPerformAcousticConfiguration = switchPosition == AM_SWITCH_CUSTOM && (AudioMoth_hasTimeBeenSet() == false || configSettings->requireAcousticConfiguration);

            /* Overrule this decision if setting of time from GPS is enabled and acoustic configuration not enforced. Also set GPS time setting flag */

            if (switchPosition == AM_SWITCH_CUSTOM && configSettings->enableTimeSettingFromGPS) {

                if (configSettings->requireAcousticConfiguration == false) shouldPerformAcousticConfiguration = false;

                setBackupFlag(BACKUP_MUST_SET_TIME_FROM_GPS, true);

            }

            /* Determine whether to listen for the acoustic tone */

            bool listenForAcousticTone = switchPosition == AM_SWITCH_CUSTOM && shouldPerformAcousticConfiguration == false;

            if (listenForAcousticTone) {

                AudioConfig_enableAudioConfiguration();

                shouldPerformAcousticConfiguration = AudioConfig_listenForAudioConfigurationTone(AUDIO_CONFIG_TONE_TIMEOUT);

            }

            if (shouldPerformAcousticConfiguration) {

                AudioMoth_setRedLED(true);

                AudioMoth_setGreenLED(false);

                audioConfigPulseCounter = 0;

                audioConfigStateLED = false;

                audioConfigToggleLED = false;

                acousticConfigurationPerformed = false;

                if (listenForAcousticTone == false) {

                    AudioConfig_enableAudioConfiguration();

                }

                AudioConfig_listenForAudioConfigurationPackets(listenForAcousticTone, AUDIO_CONFIG_PACKETS_TIMEOUT);

                AudioConfig_disableAudioConfiguration();

                if (acousticConfigurationPerformed) {

                    /* Indicate success with LED flashes */

                    AudioMoth_setRedLED(false);

                    AudioMoth_setGreenLED(true);

                    AudioMoth_delay(1000);
                    
                    AudioMoth_delay(1000);

                    AudioMoth_setGreenLED(false);

                    AudioMoth_delay(500);

                } else {

                    /* Turn off LED */

                    AudioMoth_setBothLED(false);

                    /* Determine if it is possible to still make a recording */

                    if (configSettings->requireAcousticConfiguration) {

                        setBackupFlag(BACKUP_READY_TO_MAKE_RECORDING, false);

                    } else {

                        setBackupFlag(BACKUP_READY_TO_MAKE_RECORDING, AudioMoth_hasTimeBeenSet() || getBackupFlag(BACKUP_MUST_SET_TIME_FROM_GPS));

                    }

                    /* Power down */

                    SAVE_SWITCH_POSITION_AND_POWER_DOWN(DEFAULT_WAIT_INTERVAL);

                }

            } else if (listenForAcousticTone) {

                AudioConfig_disableAudioConfiguration();

            }

        }

        /* Calculate time of next recording if ready to make a recording */

        if (getBackupFlag(BACKUP_READY_TO_MAKE_RECORDING)) {

            /* Enable energy saver mode */

            if (isEnergySaverMode(configSettings)) AudioMoth_setClockDivider(AM_HF_CLK_DIV2);

            /* Reset the recording error counter */

            *numberOfRecordingErrors = 0;

            /* Reset the recording preparation period to default */

            *recordingPreparationPeriod = INITIAL_PREPARATION_PERIOD;

            /* Reset persistent configuration write flag */

            setBackupFlag(BACKUP_WRITTEN_CONFIGURATION_TO_FILE, false);

            /* Reset GPS and sunrise and sunset variables */

            *gpsLastFixLatitude = 0;

            *gpsLastFixLongitude = 0;

            setBackupFlag(BACKUP_GPS_LOCATION_RECEIVED, false);

            *timeOfNextSunriseSunsetCalculation = 0;

            /* Try to write configuration now if it will not be written later when time is set */

            if (configSettings->enableSunRecording == false || getBackupFlag(BACKUP_MUST_SET_TIME_FROM_GPS) == false) {

                fileSystemEnabled = AudioMoth_enableFileSystem(configSettings->sampleRateDivider == 1 ? AM_SD_CARD_HIGH_SPEED : AM_SD_CARD_NORMAL_SPEED);

                if (fileSystemEnabled) {
                    
                    AudioMoth_getTime(&currentTime, &currentMilliseconds);

                    bool gpsLocationReceived = getBackupFlag(BACKUP_GPS_LOCATION_RECEIVED);

                    bool acousticLocationReceived = getBackupFlag(BACKUP_ACOUSTIC_LOCATION_RECEIVED);

                    bool success = writeConfigurationToFile(configSettings, currentTime, gpsLocationReceived, gpsLatitude, gpsLongitude, acousticLocationReceived, acousticLatitude, acousticLongitude, firmwareDescription, firmwareVersion, (uint8_t*)AM_UNIQUE_ID_START_ADDRESS, deploymentID, defaultDeploymentID);

                    setBackupFlag(BACKUP_WRITTEN_CONFIGURATION_TO_FILE, success);

                }

            }

            /* Update the time and calculate earliest schedule start time */

            AudioMoth_getTime(&currentTime, &currentMilliseconds);

            uint32_t scheduleTime = currentTime + ROUNDED_UP_DIV(currentMilliseconds + *recordingPreparationPeriod, MILLISECONDS_IN_SECOND);

            /* Schedule the next recording */

            if (switchPosition == AM_SWITCH_CUSTOM) {

                setBackupFlag(BACKUP_WAITING_FOR_MAGNETIC_SWITCH, configSettings->enableMagneticSwitch);

                if (configSettings->enableMagneticSwitch || getBackupFlag(BACKUP_MUST_SET_TIME_FROM_GPS)) {

                    *indexOfNextRecording = 0;

                    *timeOfNextRecording = UINT32_MAX;

                    *startOfRecordingPeriod = UINT32_MAX;

                    *durationOfNextRecording = UINT32_MAX;

                    *timeOfNextGPSTimeSetting = UINT32_MAX;

                } else {

                    *timeOfNextRecording = UINT32_MAX;

                    *startOfRecordingPeriod = UINT32_MAX;

                    determineSunriseAndSunsetTimesAndScheduleRecording(scheduleTime);

                }

            }

            /* Set parameters to start recording now */

            if (switchPosition == AM_SWITCH_DEFAULT) {

                *indexOfNextRecording = 0; 

                *timeOfNextRecording = scheduleTime;

                *startOfRecordingPeriod = scheduleTime;

                *durationOfNextRecording = UINT32_MAX;

                *timeOfNextGPSTimeSetting = UINT32_MAX;

            }

        }

    }
    
    /* If not ready to make a recording then flash LED and power down */

    if (getBackupFlag(BACKUP_READY_TO_MAKE_RECORDING) == false) {

        FLASH_LED(Both, SHORT_LED_FLASH_DURATION)

        SAVE_SWITCH_POSITION_AND_POWER_DOWN(DEFAULT_WAIT_INTERVAL);

    }

    /* Enable the magnetic switch */

    bool magneticSwitchEnabled = configSettings->enableMagneticSwitch && switchPosition == AM_SWITCH_CUSTOM;

    if (magneticSwitchEnabled) GPS_enableMagneticSwitch();

    /* Reset LED flags */

    bool enableLED = (switchPosition == AM_SWITCH_DEFAULT) || configSettings->enableLED;

    bool shouldSuppressLED = getBackupFlag(BACKUP_POWERED_DOWN_WITH_SHORT_WAIT_INTERVAL);

    setBackupFlag(BACKUP_POWERED_DOWN_WITH_SHORT_WAIT_INTERVAL, false);

    /* Calculate time until next activity */

    int64_t timeUntilPreparationStart;
    
    int64_t timeUntilNextGPSTimeSetting;

    calculateTimeToNextEvent(currentTime, currentMilliseconds, &timeUntilPreparationStart, &timeUntilNextGPSTimeSetting);

    /* If the GPS synchronisation window has passed then cancel it */

    int64_t timeSinceScheduledGPSTimeSetting = -timeUntilNextGPSTimeSetting;

    uint32_t gpsTimeSettingPeriod = configSettings->gpsTimeSettingPeriod == 0 ? GPS_DEFAULT_TIME_SETTING_PERIOD : configSettings->gpsTimeSettingPeriod * SECONDS_IN_MINUTE;

    if (timeSinceScheduledGPSTimeSetting > gpsTimeSettingPeriod * MILLISECONDS_IN_SECOND) {

        *timeOfNextGPSTimeSetting = UINT32_MAX;

        calculateTimeToNextEvent(currentTime, currentMilliseconds, &timeUntilPreparationStart, &timeUntilNextGPSTimeSetting);

    }

    /* Set the time from the GPS */

    if (getBackupFlag(BACKUP_MUST_SET_TIME_FROM_GPS) && getBackupFlag(BACKUP_WAITING_FOR_MAGNETIC_SWITCH) == false) {

        /* Enable the file system and set the time from the GPS */

        if (!fileSystemEnabled) fileSystemEnabled = AudioMoth_enableFileSystem(AM_SD_CARD_NORMAL_SPEED);
            
        GPS_fixResult_t fixResult = setTimeFromGPS(true, INITIAL_GPS_FIX, currentTime + GPS_INITIAL_TIME_SETTING_PERIOD);

        /* Update the schedule if successful */

        if (fixResult == GPS_SUCCESS) {

            /* Reset the flag */

            setBackupFlag(BACKUP_MUST_SET_TIME_FROM_GPS, false);

            /* Update the GPS location */

            setBackupFlag(BACKUP_GPS_LOCATION_RECEIVED, true);

            *gpsLatitude = *gpsLastFixLatitude;

            *gpsLongitude = *gpsLastFixLongitude;

            /* Write the configuration file */

            if (configSettings->enableSunRecording && fileSystemEnabled) {

                AudioMoth_getTime(&currentTime, &currentMilliseconds);

                bool gpsLocationReceived = getBackupFlag(BACKUP_GPS_LOCATION_RECEIVED);

                bool acousticLocationReceived = getBackupFlag(BACKUP_ACOUSTIC_LOCATION_RECEIVED);

                bool success = writeConfigurationToFile(configSettings, currentTime, gpsLocationReceived, gpsLatitude, gpsLongitude, acousticLocationReceived, acousticLatitude, acousticLongitude, firmwareDescription, firmwareVersion, (uint8_t*)AM_UNIQUE_ID_START_ADDRESS, deploymentID, defaultDeploymentID);

                setBackupFlag(BACKUP_WRITTEN_CONFIGURATION_TO_FILE, success);

            }

            /* Schedule the next recording */

            AudioMoth_getTime(&currentTime, &currentMilliseconds);

            uint32_t scheduleTime = currentTime + ROUNDED_UP_DIV(currentMilliseconds + *recordingPreparationPeriod, MILLISECONDS_IN_SECOND);

            determineSunriseAndSunsetTimesAndScheduleRecording(scheduleTime);

        }
        
        /* If time setting was cancelled with the magnet switch then start waiting for the magnetic switch */

        if (fixResult == GPS_CANCELLED_BY_MAGNETIC_SWITCH) startWaitingForMagneticSwitch();

        /* Power down */

        SAVE_SWITCH_POSITION_AND_POWER_DOWN(DEFAULT_WAIT_INTERVAL);

    }

    /* Make a recording */
    
    if (timeUntilPreparationStart <= 0) {

        /* Enable energy saver mode */

        if (isEnergySaverMode(configSettings)) AudioMoth_setClockDivider(AM_HF_CLK_DIV2);

        /* Write configuration if not already done so */

        if (getBackupFlag(BACKUP_WRITTEN_CONFIGURATION_TO_FILE) == false) {

            if (!fileSystemEnabled) fileSystemEnabled = AudioMoth_enableFileSystem(configSettings->sampleRateDivider == 1 ? AM_SD_CARD_HIGH_SPEED : AM_SD_CARD_NORMAL_SPEED);

            if (fileSystemEnabled) {

                bool gpsLocationReceived = getBackupFlag(BACKUP_GPS_LOCATION_RECEIVED);

                bool acousticLocationReceived = getBackupFlag(BACKUP_ACOUSTIC_LOCATION_RECEIVED);

                bool success = writeConfigurationToFile(configSettings, currentTime, gpsLocationReceived, gpsLatitude, gpsLongitude, acousticLocationReceived, acousticLatitude, acousticLongitude, firmwareDescription, firmwareVersion, (uint8_t*)AM_UNIQUE_ID_START_ADDRESS, deploymentID, defaultDeploymentID);

                setBackupFlag(BACKUP_WRITTEN_CONFIGURATION_TO_FILE, success);

            }

        }

        /* Make the recording */

        uint32_t fileOpenTime;

        uint32_t fileOpenMilliseconds;

        AM_recordingState_t recordingState = RECORDING_OKAY;

        /* Measure battery voltage */

        uint32_t supplyVoltage = AudioMoth_getSupplyVoltage();

        AM_extendedBatteryState_t extendedBatteryState = AudioMoth_getExtendedBatteryState(supplyVoltage);

        /* Check if low voltage check is enabled and that the voltage is okay */

        bool okayToMakeRecording = true;

        if (configSettings->enableLowVoltageCutoff) {

            AudioMoth_enableSupplyMonitor();

            AudioMoth_setSupplyMonitorThreshold(MINIMUM_SUPPLY_VOLTAGE);

            okayToMakeRecording = AudioMoth_isSupplyAboveThreshold();

        }

        /* Make recording if okay */

        if (okayToMakeRecording) {

            AudioMoth_enableTemperature();

            int32_t temperature = AudioMoth_getTemperature();

            AudioMoth_disableTemperature();

            if (!fileSystemEnabled) fileSystemEnabled = AudioMoth_enableFileSystem(configSettings->sampleRateDivider == 1 ? AM_SD_CARD_HIGH_SPEED : AM_SD_CARD_NORMAL_SPEED);

            if (fileSystemEnabled)  {

                recordingState = makeRecording(*timeOfNextRecording, *durationOfNextRecording, enableLED, extendedBatteryState, temperature, &fileOpenTime, &fileOpenMilliseconds);

            } else {

                FLASH_LED(Both, LONG_LED_FLASH_DURATION);

                recordingState = SDCARD_WRITE_ERROR;

            }

        } else {

            recordingState = SUPPLY_VOLTAGE_LOW;

        }

        /* Disable low voltage monitor if it was used */

        if (configSettings->enableLowVoltageCutoff) AudioMoth_disableSupplyMonitor();

        /* Enable the error warning flashes */

        if (recordingState == SUPPLY_VOLTAGE_LOW) {

            AudioMoth_delay(LONG_LED_FLASH_DURATION);

            FLASH_LED(Both, LONG_LED_FLASH_DURATION);

            *numberOfRecordingErrors += 1;

        }

        if (recordingState == SDCARD_WRITE_ERROR) {

            *numberOfRecordingErrors += 1;

        }

        /* Update the preparation period */

        if (recordingState != SDCARD_WRITE_ERROR) {

            int64_t measuredPreparationPeriod = (int64_t)fileOpenTime * MILLISECONDS_IN_SECOND + (int64_t)fileOpenMilliseconds - (int64_t)currentTime * MILLISECONDS_IN_SECOND - (int64_t)currentMilliseconds;

            *recordingPreparationPeriod = MIN(MAXIMUM_PREPARATION_PERIOD, MAX(MINIMUM_PREPARATION_PERIOD, measuredPreparationPeriod + PREPARATION_PERIOD_INCREMENT));

        }

        /* Update the time and calculate earliest schedule start time */

        AudioMoth_getTime(&currentTime, &currentMilliseconds);

        uint32_t scheduleTime = currentTime + ROUNDED_UP_DIV(currentMilliseconds + *recordingPreparationPeriod, MILLISECONDS_IN_SECOND);

        /* Schedule the next recording */

        if (*numberOfRecordingErrors >= MAXIMUM_NUMBER_OF_RECORDING_ERRORS) {

            /* Cancel the schedule */

            *indexOfNextRecording = 0; 

            *timeOfNextRecording = UINT32_MAX;

            *startOfRecordingPeriod = UINT32_MAX;

            *durationOfNextRecording = UINT32_MAX;

            *timeOfNextGPSTimeSetting = UINT32_MAX;

        } else if (switchPosition == AM_SWITCH_CUSTOM) {

            /* Update schedule time as if the recording has ended correctly */

            if (recordingState == RECORDING_OKAY || recordingState == SUPPLY_VOLTAGE_LOW || recordingState == SDCARD_WRITE_ERROR) {

                scheduleTime = MAX(scheduleTime, *timeOfNextRecording + *durationOfNextRecording);

            }

            /* Calculate the next recording schedule */

            determineSunriseAndSunsetTimesAndScheduleRecording(scheduleTime);

        } else {

            /* Set parameters to start recording now */

            *indexOfNextRecording = 0; 

            *timeOfNextRecording = scheduleTime;

            *startOfRecordingPeriod = scheduleTime;

            *durationOfNextRecording = UINT32_MAX;

            *timeOfNextGPSTimeSetting = UINT32_MAX;

        }

        /* If recording was cancelled with the magnetic switch then start waiting for the magnetic switch */

        if (recordingState == MAGNETIC_SWITCH) {
            
            startWaitingForMagneticSwitch();

            setBackupFlag(BACKUP_SHOULD_SET_TIME_FROM_GPS, false);

            SAVE_SWITCH_POSITION_AND_POWER_DOWN(DEFAULT_WAIT_INTERVAL);

        }

         /* Power down with short interval if the next recording is due */

        if (switchPosition == AM_SWITCH_CUSTOM) {

            calculateTimeToNextEvent(currentTime, currentMilliseconds, &timeUntilPreparationStart, &timeUntilNextGPSTimeSetting);

            if (timeUntilPreparationStart < DEFAULT_WAIT_INTERVAL) {

                setBackupFlag(BACKUP_POWERED_DOWN_WITH_SHORT_WAIT_INTERVAL, true);

                SAVE_SWITCH_POSITION_AND_POWER_DOWN(SHORT_WAIT_INTERVAL);

            }
        
        }

        /* Power down */

        SAVE_SWITCH_POSITION_AND_POWER_DOWN(DEFAULT_WAIT_INTERVAL);

    }

    /* Update the time from the GPS */
    
    if ((timeUntilNextGPSTimeSetting <= 0 || getBackupFlag(BACKUP_SHOULD_SET_TIME_FROM_GPS)) && timeUntilPreparationStart > GPS_MIN_TIME_SETTING_PERIOD * MILLISECONDS_IN_SECOND) {

        /* Set the time from the GPS */

        AudioMoth_enableFileSystem(AM_SD_CARD_NORMAL_SPEED);

        AM_gpsFixMode_t fixMode = configSettings->enableTimeSettingBeforeAndAfterRecordings ? GPS_FIX_BEFORE_INDIVIDUAL_RECORDING : GPS_FIX_BEFORE_RECORDING_PERIOD;
        
        if (getBackupFlag(BACKUP_SHOULD_SET_TIME_FROM_GPS)) fixMode += 1;

        if (timeUntilNextGPSTimeSetting > 0) fixMode += 1;

        uint32_t gpsTimeSettingPeriod = configSettings->gpsTimeSettingPeriod == 0 ? GPS_DEFAULT_TIME_SETTING_PERIOD : configSettings->gpsTimeSettingPeriod * SECONDS_IN_MINUTE;

        uint32_t timeOut = MIN(*timeOfNextRecording - ROUNDED_UP_DIV(*recordingPreparationPeriod, MILLISECONDS_IN_SECOND) - GPS_TIME_SETTING_MARGIN, currentTime + gpsTimeSettingPeriod);

        GPS_fixResult_t fixResult = setTimeFromGPS(enableLED, fixMode, timeOut);

        /* Update flag and next scheduled GPS fix time */

        if (timeUntilNextGPSTimeSetting <= 0) *timeOfNextGPSTimeSetting = UINT32_MAX;

        setBackupFlag(BACKUP_SHOULD_SET_TIME_FROM_GPS, false);

        /* If time setting was cancelled with the magnet switch then start waiting for the magnetic switch */

        if (fixResult == GPS_CANCELLED_BY_MAGNETIC_SWITCH) startWaitingForMagneticSwitch();

        /* Power down */

        SAVE_SWITCH_POSITION_AND_POWER_DOWN(DEFAULT_WAIT_INTERVAL);

    } 

    /* Update the post-recording GPS flag if the previous condition was not met */

    setBackupFlag(BACKUP_SHOULD_SET_TIME_FROM_GPS, false);

    /* Power down if switch position has changed */

    if (switchPosition != *previousSwitchPosition) SAVE_SWITCH_POSITION_AND_POWER_DOWN(DEFAULT_WAIT_INTERVAL);

    /* Calculate the wait intervals */

    int64_t waitIntervalMilliseconds = WAITING_LED_FLASH_INTERVAL;

    uint32_t waitIntervalSeconds = WAITING_LED_FLASH_INTERVAL / MILLISECONDS_IN_SECOND;

    if (getBackupFlag(BACKUP_WAITING_FOR_MAGNETIC_SWITCH)) {
        
        waitIntervalMilliseconds *= MAGNETIC_SWITCH_WAIT_MULTIPLIER;

        waitIntervalSeconds *= MAGNETIC_SWITCH_WAIT_MULTIPLIER;

    }

    /* Wait for the next event whilst flashing the LED */

    bool startedRealTimeClock = false;

    while (true) {

        /* Update the time */

        AudioMoth_getTime(&currentTime, &currentMilliseconds);

        /* Handle magnetic switch event */

        bool magneticSwitchEvent = magneticSwitch || (magneticSwitchEnabled && GPS_isMagneticSwitchClosed());

        if (magneticSwitchEvent) {

            if (getBackupFlag(BACKUP_WAITING_FOR_MAGNETIC_SWITCH)) {

                stopWaitingForMagneticSwitch(&currentTime, &currentMilliseconds);

                if (switchPosition == AM_SWITCH_CUSTOM && configSettings->enableTimeSettingFromGPS) setBackupFlag(BACKUP_MUST_SET_TIME_FROM_GPS, true);

            } else {

                startWaitingForMagneticSwitch();

            }
            
            SAVE_SWITCH_POSITION_AND_POWER_DOWN(DEFAULT_WAIT_INTERVAL);

        }

        /* Handle switch position change */

        bool switchEvent = switchPositionChanged || AudioMoth_getSwitchPosition() != *previousSwitchPosition;
       
        if (switchEvent) SAVE_SWITCH_POSITION_AND_POWER_DOWN(DEFAULT_WAIT_INTERVAL);
        
        /* Calculate the time to the next event */
        
        calculateTimeToNextEvent(currentTime, currentMilliseconds, &timeUntilPreparationStart, &timeUntilNextGPSTimeSetting);

        int64_t timeToEarliestEvent = MIN(timeUntilPreparationStart, timeUntilNextGPSTimeSetting);

        /* Flash LED */

        bool shouldFlashLED = getBackupFlag(BACKUP_WAITING_FOR_MAGNETIC_SWITCH) || (enableLED && shouldSuppressLED == false && timeToEarliestEvent > MINIMUM_LED_FLASH_INTERVAL);

        if (shouldFlashLED) {

            if (*numberOfRecordingErrors > 0) {

                FLASH_LED(Both, WAITING_LED_FLASH_DURATION);

            } else {

                FLASH_LED(Green, WAITING_LED_FLASH_DURATION);

            }

        }

        /* Check there is time to sleep */

        if (timeToEarliestEvent < waitIntervalMilliseconds) {
            
            /* Calculate the remaining time to power down */

            uint32_t timeToWait = timeToEarliestEvent < 0 ? 0 : timeToEarliestEvent;

            SAVE_SWITCH_POSITION_AND_POWER_DOWN(timeToWait);

        }

        /* Start the real time clock if it isn't running */

        if (startedRealTimeClock == false) {

            AudioMoth_startRealTimeClock(waitIntervalSeconds);

            startedRealTimeClock = true;

        }

        /* Enter deep sleep */

        AudioMoth_deepSleep();

        /* Handle time overflow on awakening */

        AudioMoth_checkAndHandleTimeOverflow();

    }

}

/* Time zone handler */

inline void AudioMoth_timezoneRequested(int8_t *timezoneHours, int8_t *timezoneMinutes) {

    *timezoneHours = configSettings->timezoneHours;

    *timezoneMinutes = configSettings->timezoneMinutes;

}

/* GPS time handlers */

inline void GPS_handleSetTime(uint32_t time, uint32_t milliseconds, int64_t timeDifference, uint32_t measuredClockFrequency) {

    char setTimeBuffer[64];

    /* Update the time if appropriate */

    if (!AudioMoth_hasTimeBeenSet()) {

        AudioMoth_setTime(time, milliseconds);

        writeGPSLogMessage(time, milliseconds, "Time was set from GPS.");

    } else {

        if (timeDifference == 0) {

            writeGPSLogMessage(time, milliseconds, "Time was not updated. The internal clock was correct.");

        } else if (timeDifference < -GPS_MAXIMUM_MS_DIFFERENCE || timeDifference > GPS_MAXIMUM_MS_DIFFERENCE) {

            writeGPSLogMessage(time, milliseconds, "Time was not updated. The discrepancy between the internal clock and the GPS was too large.");

        } else {

            AudioMoth_setTime(time, milliseconds);
            
            sprintf(setTimeBuffer, "Time was updated. The internal clock was %ldms %s.", timeDifference > 0 ? (int32_t)timeDifference : -(int32_t)timeDifference, timeDifference > 0 ? "fast" : "slow");

            writeGPSLogMessage(time, milliseconds, setTimeBuffer);

        }

    }

    /* Calculate the actual sampling rate */

    uint32_t intendedClockFrequency = AudioMoth_getClockFrequency();

    uint32_t intendedSamplingRate = configSettings->sampleRate / configSettings->sampleRateDivider;

    uint32_t clockTicksPerSample = intendedClockFrequency / intendedSamplingRate;

    uint64_t actualSamplingRate = ROUNDED_DIV(GPS_FREQUENCY_PRECISION * (uint64_t)measuredClockFrequency, (uint64_t)clockTicksPerSample);

    uint32_t integerPart = actualSamplingRate / GPS_FREQUENCY_PRECISION;

    uint32_t fractionalPart = actualSamplingRate % GPS_FREQUENCY_PRECISION;

    sprintf(setTimeBuffer, "Actual sample rate will be %lu.%03lu Hz.", integerPart, fractionalPart);

    writeGPSLogMessage(time, milliseconds, setTimeBuffer);

}

inline void GPS_handleGetTime(uint32_t *time, uint32_t *milliseconds) {

    AudioMoth_getTime(time, milliseconds);

}

/* GPS format conversion */

static int32_t convertToDecimalDegrees(uint32_t degrees, uint32_t minutes, uint32_t tenThousandths, char direction) {

    int32_t value = degrees * GPS_LOCATION_PRECISION;

    value += ROUNDED_DIV(minutes * GPS_LOCATION_PRECISION, MINUTES_IN_DEGREE);

    value += ROUNDED_DIV(tenThousandths * (GPS_LOCATION_PRECISION / 10000), MINUTES_IN_DEGREE); 

    if (direction == 'S' || direction == 'W') value *= -1;

    return value;

}

/* GPS interrupt handlers */

inline void GPS_handleTickEvent() {

    if (gpsTickEventCount == 0) {

        gpsTickEventModulo = GPS_TICK_EVENTS_PER_SECOND;
    
        if (gpsPPSEvent || gpsFixEvent) {

            gpsTickEventModulo /= 3;

        } else if (gpsMessageEvent) {

            gpsTickEventModulo /= 2;

        }

        gpsMessageEvent = false;

        gpsFixEvent = false;

        gpsPPSEvent = false;

    }

    if (gpsEnableLED) AudioMoth_setRedLED(gpsTickEventCount % gpsTickEventModulo == 0);

    gpsTickEventCount = (gpsTickEventCount + 1) % GPS_TICK_EVENTS_PER_SECOND;

}

inline void GPS_handlePPSEvent(uint32_t time, uint32_t milliseconds) {

    writeGPSLogMessage(time, milliseconds, "Received pulse per second signal.");

    gpsPPSEvent = true;

}

inline void GPS_handleFixEvent(uint32_t time, uint32_t milliseconds, GPS_fixTime_t *fixTime, GPS_fixPosition_t *fixPosition, char *message) {

    static char fixBuffer[128];

    *gpsLastFixLatitude = convertToDecimalDegrees(fixPosition->latitudeDegrees, fixPosition->latitudeMinutes, fixPosition->latitudeTenThousandths, fixPosition->latitudeDirection);

    *gpsLastFixLongitude = convertToDecimalDegrees(fixPosition->longitudeDegrees, fixPosition->longitudeMinutes, fixPosition->longitudeTenThousandths, fixPosition->longitudeDirection);

    uint32_t length = sprintf(fixBuffer, "Received GPS fix - %ld.%06ld°%c %ld.%06ld°%c ", ABS(*gpsLastFixLatitude) / GPS_LOCATION_PRECISION, ABS(*gpsLastFixLatitude) % GPS_LOCATION_PRECISION, fixPosition->latitudeDirection, ABS(*gpsLastFixLongitude) / GPS_LOCATION_PRECISION, ABS(*gpsLastFixLongitude) % GPS_LOCATION_PRECISION, fixPosition->longitudeDirection);
    
    sprintf(fixBuffer + length, "at %02u/%02u/%04u %02u:%02u:%02u.%03u UTC.", fixTime->day, fixTime->month, fixTime->year, fixTime->hours, fixTime->minutes, fixTime->seconds, fixTime->milliseconds);

    writeGPSLogMessage(time, milliseconds, fixBuffer);

    gpsFixEvent = true;

}

inline void GPS_handleMessageEvent(uint32_t time, uint32_t milliseconds, char *message) {

    if (!gpsFirstMessageReceived) {

        writeGPSLogMessage(time, milliseconds, "Received first GPS message.");

        gpsFirstMessageReceived = true;

    }

    gpsMessageEvent = true;

}

inline void GPS_handleMagneticSwitchInterrupt() {

    magneticSwitch = true;

    GPS_cancelTimeSetting(GPS_CANCEL_BY_MAGNETIC_SWITCH);

}

/* AudioMoth interrupt handlers */

inline void AudioMoth_handleMicrophoneChangeInterrupt() {

    microphoneChanged = true;

}

inline void AudioMoth_handleSwitchInterrupt() {

    switchPositionChanged = true;

    AudioConfig_cancelAudioConfiguration();

    GPS_cancelTimeSetting(GPS_CANCEL_BY_SWITCH);

}

inline void AudioMoth_handleDirectMemoryAccessInterrupt(bool isPrimaryBuffer, int16_t **nextBuffer) {

    int16_t *source = secondaryBuffer;

    if (isPrimaryBuffer) source = primaryBuffer;

    /* Apply filter to samples */

    bool thresholdExceeded = DigitalFilter_applyFilter(source, buffers[writeBuffer] + writeBufferIndex, configSettings->sampleRateDivider, numberOfRawSamplesInDMATransfer);

    numberOfDMATransfers += 1;

    /* Update the current buffer index and write buffer if wait period is over */

    if (numberOfDMATransfers > numberOfDMATransfersToWait) {

        writeIndicator[writeBuffer] |= thresholdExceeded;

        writeBufferIndex += numberOfRawSamplesInDMATransfer / configSettings->sampleRateDivider;

        if (writeBufferIndex == NUMBER_OF_SAMPLES_IN_BUFFER) {

            writeBufferIndex = 0;

            writeBuffer = (writeBuffer + 1) & (NUMBER_OF_BUFFERS - 1);

            writeIndicator[writeBuffer] = false;

        }

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

    memcpy(transmitBuffer + 1, &currentTime, UINT32_SIZE_IN_BYTES);

    /* Copy the unique ID to the USB packet */

    memcpy(transmitBuffer + 5, (uint8_t*)AM_UNIQUE_ID_START_ADDRESS, AM_UNIQUE_ID_SIZE_IN_BYTES);

    /* Copy the battery state to the USB packet */

    uint32_t supplyVoltage = AudioMoth_getSupplyVoltage();

    AM_batteryState_t batteryState = AudioMoth_getBatteryState(supplyVoltage);

    memcpy(transmitBuffer + 5 + AM_UNIQUE_ID_SIZE_IN_BYTES, &batteryState, 1);

    /* Copy the firmware version to the USB packet */

    memcpy(transmitBuffer + 6 + AM_UNIQUE_ID_SIZE_IN_BYTES, firmwareVersion, AM_FIRMWARE_VERSION_LENGTH);

    /* Copy the firmware description to the USB packet */

    memcpy(transmitBuffer + 6 + AM_UNIQUE_ID_SIZE_IN_BYTES + AM_FIRMWARE_VERSION_LENGTH, firmwareDescription, AM_FIRMWARE_DESCRIPTION_LENGTH);

}

inline void AudioMoth_usbApplicationPacketReceived(uint32_t messageType, uint8_t* receiveBuffer, uint8_t *transmitBuffer, uint32_t size) {

    /* Make persistent configuration settings data structure */

    static persistentConfigSettings_t persistentConfigSettings __attribute__ ((aligned(UINT32_SIZE_IN_BYTES)));

    memcpy(&persistentConfigSettings.firmwareVersion, &firmwareVersion, AM_FIRMWARE_VERSION_LENGTH);

    memcpy(&persistentConfigSettings.firmwareDescription, &firmwareDescription, AM_FIRMWARE_DESCRIPTION_LENGTH);

    memcpy(&persistentConfigSettings.configSettings, receiveBuffer + 1,  sizeof(configSettings_t));

    /* Implement energy saver mode changes */

    if (isEnergySaverMode(&persistentConfigSettings.configSettings)) {

        persistentConfigSettings.configSettings.sampleRate /= 2;
        persistentConfigSettings.configSettings.clockDivider /= 2;
        persistentConfigSettings.configSettings.sampleRateDivider /= 2;

    }

    /* Copy persistent configuration settings to flash */

    uint32_t numberOfBytes = ROUND_UP_TO_MULTIPLE(sizeof(persistentConfigSettings_t), UINT32_SIZE_IN_BYTES);

    bool success = AudioMoth_writeToFlashUserDataPage((uint8_t*)&persistentConfigSettings, numberOfBytes);

    if (success) {

        /* Copy the USB packet contents to the back-up register data structure location */

        copyToBackupDomain((uint32_t*)configSettings, (uint8_t*)&persistentConfigSettings.configSettings, sizeof(configSettings_t));

        /* Copy the back-up register data structure to the USB packet */

        copyFromBackupDomain(transmitBuffer + 1, (uint32_t*)configSettings, sizeof(configSettings_t));

        /* Revert energy saver mode changes */

        configSettings_t *tempConfigSettings = (configSettings_t*)(transmitBuffer + 1);

        if (isEnergySaverMode(tempConfigSettings)) {

            tempConfigSettings->sampleRate *= 2;
            tempConfigSettings->clockDivider *= 2;
            tempConfigSettings->sampleRateDivider *= 2;

        }    

        /* Set the time */

        AudioMoth_setTime(configSettings->time, USB_CONFIG_TIME_CORRECTION);

        /* Blink the green LED */

        AudioMoth_blinkDuringUSB(USB_CONFIGURATION_BLINK);

    } else {

        /* Return blank configuration as error indicator */

        memset(transmitBuffer + 1, 0, sizeof(configSettings_t));

    }

}

/* Audio configuration handlers */

inline void AudioConfig_handleAudioConfigurationEvent(AC_audioConfigurationEvent_t event) {

    if (event == AC_EVENT_PULSE) {

        audioConfigPulseCounter = (audioConfigPulseCounter + 1) % AUDIO_CONFIG_PULSE_INTERVAL;

    } else if (event == AC_EVENT_START) {

        audioConfigStateLED = true;

        audioConfigToggleLED = true;

        AudioMoth_getTime(&secondsOfAcousticSignalStart, &millisecondsOfAcousticSignalStart);

    } else if (event == AC_EVENT_BYTE) {

        audioConfigToggleLED = !audioConfigToggleLED;

    } else if (event == AC_EVENT_BIT_ERROR || event == AC_EVENT_CRC_ERROR) {

        audioConfigStateLED = false;

    }

    AudioMoth_setGreenLED((audioConfigStateLED && audioConfigToggleLED) || (!audioConfigStateLED && !audioConfigPulseCounter));

}

inline void AudioConfig_handleAudioConfigurationPacket(uint8_t *receiveBuffer, uint32_t size) {

    uint32_t standardPacketSize = UINT32_SIZE_IN_BYTES + UINT16_SIZE_IN_BYTES;

    bool standardPacket = size == standardPacketSize;

    bool hasLocation = size == (standardPacketSize + ACOUSTIC_LOCATION_SIZE_IN_BYTES) || size == (standardPacketSize + DEPLOYMENT_ID_LENGTH + ACOUSTIC_LOCATION_SIZE_IN_BYTES);

    bool hasDeploymentID = size == (standardPacketSize + DEPLOYMENT_ID_LENGTH) || size == (standardPacketSize + DEPLOYMENT_ID_LENGTH + ACOUSTIC_LOCATION_SIZE_IN_BYTES);

    if (standardPacket || hasLocation || hasDeploymentID) {

        /* Copy time from the packet */

        uint32_t time;

        memcpy(&time, receiveBuffer, UINT32_SIZE_IN_BYTES);

        /* Calculate the time correction */

        uint32_t secondsOfAcousticSignalEnd;

        uint32_t millisecondsOfAcousticSignalEnd;

        AudioMoth_getTime(&secondsOfAcousticSignalEnd, &millisecondsOfAcousticSignalEnd);

        uint32_t millisecondTimeOffset = (secondsOfAcousticSignalEnd - secondsOfAcousticSignalStart) * MILLISECONDS_IN_SECOND + millisecondsOfAcousticSignalEnd - millisecondsOfAcousticSignalStart + AUDIO_CONFIG_TIME_CORRECTION;

        /* Set the time */

        AudioMoth_setTime(time + millisecondTimeOffset / MILLISECONDS_IN_SECOND, millisecondTimeOffset % MILLISECONDS_IN_SECOND);

        /* Set acoustic location */

        if (hasLocation) {

            acousticLocation_t location;

            memcpy(&location, receiveBuffer + standardPacketSize, ACOUSTIC_LOCATION_SIZE_IN_BYTES);

            setBackupFlag(BACKUP_ACOUSTIC_LOCATION_RECEIVED, true);

            *acousticLatitude = location.latitude;
            
            *acousticLongitude = location.longitude * ACOUSTIC_LONGITUDE_MULTIPLIER;

        }

        /* Set deployment ID */

        if (hasDeploymentID) {

            copyToBackupDomain((uint32_t*)deploymentID, receiveBuffer + standardPacketSize + (hasLocation ? ACOUSTIC_LOCATION_SIZE_IN_BYTES : 0), DEPLOYMENT_ID_LENGTH);

        }

        /* Indicate success */

        AudioConfig_cancelAudioConfiguration();

        acousticConfigurationPerformed = true;

    }

    /* Reset receive state */

    audioConfigStateLED = false;

}

/* Clear and encode the compression buffer */

static void clearCompressionBuffer() {

    for (uint32_t i = 0; i < COMPRESSION_BUFFER_SIZE_IN_BYTES / NUMBER_OF_BYTES_IN_SAMPLE; i += 1) {

        compressionBuffer[i] = 0;

    }

}

static void encodeCompressionBuffer(uint32_t numberOfCompressedBuffers) {

    for (uint32_t i = 0; i < UINT32_SIZE_IN_BITS; i += 1) {

        compressionBuffer[i] = numberOfCompressedBuffers & 0x01 ? 1 : -1;

        numberOfCompressedBuffers >>= 1;

    }

    for (uint32_t i = UINT32_SIZE_IN_BITS; i < COMPRESSION_BUFFER_SIZE_IN_BYTES / NUMBER_OF_BYTES_IN_SAMPLE; i += 1) {

        compressionBuffer[i] = 0;

    }

}

/* Generate foldername and filename from time */

static void generateFolderAndFilename(char *foldername, char *filename, uint32_t timestamp, bool triggeredRecording) {

    struct tm time;

    time_t rawTime = timestamp + configSettings->timezoneHours * SECONDS_IN_HOUR + configSettings->timezoneMinutes * SECONDS_IN_MINUTE;

    gmtime_r(&rawTime, &time);

    sprintf(foldername, "%04d%02d%02d", YEAR_OFFSET + time.tm_year, MONTH_OFFSET + time.tm_mon, time.tm_mday);

    uint32_t length = 0;
    
    if (configSettings->enableDailyFolders) {

        length += sprintf(filename, "%s/", foldername);

    } 
    
    if (configSettings->enableFilenameWithDeviceID) {

        uint8_t *source = memcmp(deploymentID, defaultDeploymentID, DEPLOYMENT_ID_LENGTH) ? deploymentID : (uint8_t*)AM_UNIQUE_ID_START_ADDRESS;

        length += sprintf(filename + length, SERIAL_NUMBER "_", FORMAT_SERIAL_NUMBER(source));

    }
    
    length += sprintf(filename + length, "%s_%02d%02d%02d", foldername, time.tm_hour, time.tm_min, time.tm_sec);

    char *extension = triggeredRecording ? "T.WAV" : ".WAV";

    strcpy(filename + length, extension);

}

/* Save recording to SD card */

static AM_recordingState_t makeRecording(uint32_t timeOfNextRecording, uint32_t recordDuration, bool enableLED, AM_extendedBatteryState_t extendedBatteryState, int32_t temperature, uint32_t *fileOpenTime, uint32_t *fileOpenMilliseconds) {

    /* Initialise buffers */

    writeBuffer = 0;

    writeBufferIndex = 0;

    buffers[0] = (int16_t*)AM_EXTERNAL_SRAM_START_ADDRESS;

    for (uint32_t i = 1; i < NUMBER_OF_BUFFERS; i += 1) {
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

    /* Calculate effective sample rate */

    uint32_t effectiveSampleRate = configSettings->sampleRate / configSettings->sampleRateDivider;

    /* Set up the digital filter */

    uint32_t blockingFilterFrequency = configSettings->disable48HzDCBlockingFilter ? LOW_DC_BLOCKING_FREQ : DEFAULT_DC_BLOCKING_FREQ;

    if (configSettings->lowerFilterFreq == 0 && configSettings->higherFilterFreq == 0) {

        requestedFilterType = NO_FILTER;

        DigitalFilter_designHighPassFilter(effectiveSampleRate, blockingFilterFrequency);

    } else if (configSettings->lowerFilterFreq == UINT16_MAX) {

        requestedFilterType = LOW_PASS_FILTER;

        DigitalFilter_designBandPassFilter(effectiveSampleRate, blockingFilterFrequency, FILTER_FREQ_MULTIPLIER * configSettings->higherFilterFreq);

    } else if (configSettings->higherFilterFreq == UINT16_MAX) {

        requestedFilterType = HIGH_PASS_FILTER;

        DigitalFilter_designHighPassFilter(effectiveSampleRate, MAX(blockingFilterFrequency, FILTER_FREQ_MULTIPLIER * configSettings->lowerFilterFreq));

    } else {

        requestedFilterType = BAND_PASS_FILTER;

        DigitalFilter_designBandPassFilter(effectiveSampleRate, MAX(blockingFilterFrequency, FILTER_FREQ_MULTIPLIER * configSettings->lowerFilterFreq), FILTER_FREQ_MULTIPLIER * configSettings->higherFilterFreq);

    }

    /* Calculate the number of samples in each DMA transfer (while ensuring that number of samples written to the SRAM buffer on each DMA transfer is a power of two so each SRAM buffer is filled after an integer number of DMA transfers) */

    numberOfRawSamplesInDMATransfer = MAXIMUM_SAMPLES_IN_DMA_TRANSFER / configSettings->sampleRateDivider;

    while (numberOfRawSamplesInDMATransfer & (numberOfRawSamplesInDMATransfer - 1)) {

        numberOfRawSamplesInDMATransfer = numberOfRawSamplesInDMATransfer & (numberOfRawSamplesInDMATransfer - 1);

    }

    numberOfRawSamplesInDMATransfer *= configSettings->sampleRateDivider;

    /* Calculate the minimum amplitude threshold duration */

    uint32_t minimumNumberOfTriggeredBuffersToWrite = ROUNDED_UP_DIV(configSettings->minimumTriggerDuration * effectiveSampleRate, NUMBER_OF_SAMPLES_IN_BUFFER);

    /* Initialise termination conditions */

    microphoneChanged = false;

    bool supplyVoltageLow = false;

    /* Enable the external SRAM and microphone and initialise direct memory access */

    AudioMoth_enableExternalSRAM();

    AM_gainRange_t gainRange = configSettings->enableLowGainRange ? AM_LOW_GAIN_RANGE : AM_NORMAL_GAIN_RANGE;

    bool externalMicrophone = AudioMoth_enableMicrophone(gainRange, configSettings->gain, configSettings->clockDivider, configSettings->acquisitionCycles, configSettings->oversampleRate);

    AudioMoth_initialiseDirectMemoryAccess(primaryBuffer, secondaryBuffer, numberOfRawSamplesInDMATransfer);

    /* Calculate the sample multiplier */

    float sampleMultiplier = 16.0f / (float)(configSettings->oversampleRate * configSettings->sampleRateDivider);

    if (AudioMoth_hasInvertedOutput()) sampleMultiplier = -sampleMultiplier;

    if (externalMicrophone) sampleMultiplier = -sampleMultiplier;

    DigitalFilter_setAdditionalGain(sampleMultiplier);

    /* Determine if amplitude threshold is enabled */

    bool frequencyTriggerEnabled = configSettings->enableFrequencyTrigger;

    bool amplitudeThresholdEnabled = frequencyTriggerEnabled ? false : configSettings->amplitudeThreshold > 0 || configSettings->enableAmplitudeThresholdDecibelScale || configSettings->enableAmplitudeThresholdPercentageScale;

    /* Configure the digital filter for the appropriate trigger */
    
    if (frequencyTriggerEnabled) {

        uint32_t frequency = MIN(effectiveSampleRate / 2, FILTER_FREQ_MULTIPLIER * configSettings->frequencyTriggerCentreFrequency);

        uint32_t windowLength = MIN(FREQUENCY_TRIGGER_WINDOW_MAXIMUM, MAX(FREQUENCY_TRIGGER_WINDOW_MINIMUM, 1 << configSettings->frequencyTriggerWindowLengthShift));

        float percentageThreshold = (float)configSettings->frequencyTriggerThresholdPercentageMantissa * powf(10.0f, (float)configSettings->frequencyTriggerThresholdPercentageExponent); 

        DigitalFilter_setFrequencyTrigger(windowLength, effectiveSampleRate, frequency, percentageThreshold);

    }

    if (amplitudeThresholdEnabled) {
        
        DigitalFilter_setAmplitudeThreshold(configSettings->amplitudeThreshold);

    }

    /* Set the initial header comment details */

    AM_recordingState_t recordingState = SDCARD_WRITE_ERROR;

    setHeaderDetails(&wavHeader, effectiveSampleRate, 0, 0);

    setHeaderComment(&wavHeader, configSettings, timeOfNextRecording, (uint8_t*)AM_UNIQUE_ID_START_ADDRESS, deploymentID, defaultDeploymentID, extendedBatteryState, temperature, externalMicrophone, recordingState, requestedFilterType);

    /* Show LED for SD card activity */

    if (enableLED) AudioMoth_setRedLED(true);

    /* Open a file with the current local time as the name */

    static char filename[MAXIMUM_FILE_NAME_LENGTH];

    static char foldername[MAXIMUM_FILE_NAME_LENGTH];

    generateFolderAndFilename(foldername, filename, timeOfNextRecording, frequencyTriggerEnabled || amplitudeThresholdEnabled);

    if (configSettings->enableDailyFolders) {

        bool directoryExists = AudioMoth_doesDirectoryExist(foldername);

        if (directoryExists == false) FLASH_LED_AND_RETURN_ON_ERROR(AudioMoth_makeDirectory(foldername));

    }

    FLASH_LED_AND_RETURN_ON_ERROR(AudioMoth_openFile(filename));

    /* Write the header */

    FLASH_LED_AND_RETURN_ON_ERROR(AudioMoth_writeToFile(&wavHeader, sizeof(wavHeader_t)));    

    FLASH_LED_AND_RETURN_ON_ERROR(AudioMoth_syncFile());

    FLASH_LED_AND_RETURN_ON_ERROR(AudioMoth_seekInFile(0));
    
    AudioMoth_setRedLED(false);

    /* Measure the time difference from the start time */

    AudioMoth_getTime(fileOpenTime, fileOpenMilliseconds);

    /* Calculate time correction for sample rate due to file header */

    uint32_t numberOfSamplesInHeader = sizeof(wavHeader_t) / NUMBER_OF_BYTES_IN_SAMPLE;

    int32_t sampleRateTimeOffset = ROUNDED_DIV(numberOfSamplesInHeader * MILLISECONDS_IN_SECOND, effectiveSampleRate);

    /* Calculate time until the recording should start */

    int64_t millisecondsUntilRecordingShouldStart = (int64_t)timeOfNextRecording * MILLISECONDS_IN_SECOND - (int64_t)*fileOpenTime * MILLISECONDS_IN_SECOND - (int64_t)*fileOpenMilliseconds - (int64_t)sampleRateTimeOffset;

    /* Calculate the actual recording start time if the intended start has been missed */

    uint32_t timeOffset = millisecondsUntilRecordingShouldStart < 0 ? 1 - millisecondsUntilRecordingShouldStart / MILLISECONDS_IN_SECOND : 0;

    recordDuration = timeOffset >= recordDuration ? 0 : recordDuration - timeOffset;

    millisecondsUntilRecordingShouldStart += timeOffset * MILLISECONDS_IN_SECOND;

    /* Calculate the period to wait before starting the DMA transfers */

    uint32_t numberOfRawSamplesPerMillisecond = configSettings->sampleRate / MILLISECONDS_IN_SECOND;

    uint32_t numberOfRawSamplesToWait = millisecondsUntilRecordingShouldStart * numberOfRawSamplesPerMillisecond;

    numberOfDMATransfersToWait = numberOfRawSamplesToWait / numberOfRawSamplesInDMATransfer;

    uint32_t remainingNumberOfRawSamples = numberOfRawSamplesToWait % numberOfRawSamplesInDMATransfer;

    uint32_t remainingMillisecondsToWait = ROUNDED_DIV(remainingNumberOfRawSamples, numberOfRawSamplesPerMillisecond);

    /* Calculate updated recording parameters */

    uint32_t maximumNumberOfSeconds = (MAXIMUM_WAV_FILE_SIZE - sizeof(wavHeader_t)) / NUMBER_OF_BYTES_IN_SAMPLE / effectiveSampleRate;

    bool fileSizeLimited = (recordDuration > maximumNumberOfSeconds);

    uint32_t numberOfSamples = effectiveSampleRate * (fileSizeLimited ? maximumNumberOfSeconds : recordDuration);

    /* Initialise main loop variables */

    uint32_t readBuffer = 0;

    uint32_t samplesWritten = 0;

    uint32_t buffersProcessed = 0;

    uint32_t numberOfCompressedBuffers = 0;

    uint32_t totalNumberOfCompressedSamples = 0;

    uint32_t numberOfTriggeredBuffersWritten = 0;

    bool triggerHasOccurred = false;

    /* Start processing DMA transfers */

    numberOfDMATransfers = 0;

    AudioMoth_delay(remainingMillisecondsToWait);

    AudioMoth_startMicrophoneSamples(configSettings->sampleRate);
    
    /* Main recording loop */

    while (samplesWritten < numberOfSamples + numberOfSamplesInHeader && !microphoneChanged && !switchPositionChanged && !magneticSwitch && !supplyVoltageLow) {

        while (readBuffer != writeBuffer && samplesWritten < numberOfSamples + numberOfSamplesInHeader && !microphoneChanged && !switchPositionChanged && !magneticSwitch && !supplyVoltageLow) {
       
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
	    float32_t NNoutput = neuralNetwork(buffersMFCC[2]);
	    uint32_t BufferGreen = 0;
	    // <---

            /* Determine the appropriate number of bytes to the SD card */

            //uint32_t numberOfSamplesToWrite = MIN(numberOfSamples + numberOfSamplesInHeader - samplesWritten, NUMBER_OF_SAMPLES_IN_BUFFER);
            // modified: write complete superbuffer
             uint32_t numberOfSamplesToWrite = MIN(numberOfSamples + numberOfSamplesInHeader - samplesWritten, NUMBER_OF_BUFFERS_IN_SUPERBUFFER * NUMBER_OF_SAMPLES_IN_BUFFER);

            /* Check if this buffer should actually be written to the SD card */

            bool writeIndicated = (amplitudeThresholdEnabled == false && frequencyTriggerEnabled == false) || writeIndicator[readBuffer];

            if (frequencyTriggerEnabled && configSettings->sampleRateDivider > 1) writeIndicated = DigitalFilter_applyFrequencyTrigger(buffers[readBuffer], NUMBER_OF_SAMPLES_IN_BUFFER);

            /* Ensure the minimum number of buffers will be written */

            triggerHasOccurred |= writeIndicated;

            numberOfTriggeredBuffersWritten = writeIndicated ? 0 : numberOfTriggeredBuffersWritten + 1;

            bool shouldWriteThisSector = true; // modified. writeIndicated || (triggerHasOccurred && numberOfTriggeredBuffersWritten < minimumNumberOfTriggeredBuffersToWrite);

            /* Compress the buffer or write the buffer to SD card */

            if (shouldWriteThisSector == false && buffersProcessed > 0 && numberOfSamplesToWrite == NUMBER_OF_SAMPLES_IN_BUFFER) {

                numberOfCompressedBuffers += NUMBER_OF_BYTES_IN_SAMPLE * NUMBER_OF_SAMPLES_IN_BUFFER / COMPRESSION_BUFFER_SIZE_IN_BYTES;

            } else { // SD card write 

                /* Light LED during SD card write if appropriate */

                if (enableLED) AudioMoth_setRedLED(true); 

                /* Encode and write compression buffer */

                if (numberOfCompressedBuffers > 0) {

                    encodeCompressionBuffer(numberOfCompressedBuffers);

                    totalNumberOfCompressedSamples += (numberOfCompressedBuffers - 1) * COMPRESSION_BUFFER_SIZE_IN_BYTES / NUMBER_OF_BYTES_IN_SAMPLE;
                    
                    FLASH_LED_AND_RETURN_ON_ERROR(AudioMoth_writeToFile(compressionBuffer, COMPRESSION_BUFFER_SIZE_IN_BYTES));
                    
                    numberOfCompressedBuffers = 0;

                }

                /* Either write the buffer or write a blank buffer */

                if (shouldWriteThisSector) {

                    // --> Introduced code:
                    if ((readBuffer+1) % NUMBER_OF_BUFFERS_IN_SUPERBUFFER == 0){ // for each superbuffer
                      if (buffersProcessed == 0) memcpy(buffers[readBuffer], &wavHeader, sizeof(wavHeader_t));
                    
                      //FLASH_LED_AND_RETURN_ON_ERROR(AudioMoth_writeToFile(buffers[readBuffer], NUMBER_OF_BYTES_IN_SAMPLE * numberOfSamplesToWrite));
                      FLASH_LED_AND_RETURN_ON_ERROR(AudioMoth_writeToFile(buffers[readBuffer-NUMBER_OF_BUFFERS_IN_SUPERBUFFER+1], NUMBER_OF_BYTES_IN_SAMPLE * numberOfSamplesToWrite)); // modified. Write complete superbuffer
                      
                      samplesWritten += numberOfSamplesToWrite;

		      buffersProcessed += 1;
                   }
                    
                } else {

                    clearCompressionBuffer();

                    uint32_t blankBuffersWritten = 0;

                    uint32_t numberOfBlankSamplesToWrite = numberOfSamplesToWrite;

                    while (numberOfBlankSamplesToWrite > 0) {

                        uint32_t numberOfSamples = MIN(numberOfBlankSamplesToWrite, COMPRESSION_BUFFER_SIZE_IN_BYTES / NUMBER_OF_BYTES_IN_SAMPLE);

                        bool writeHeader = buffersProcessed == 0 && blankBuffersWritten == 0 && numberOfSamples >= sizeof(wavHeader_t) / NUMBER_OF_BYTES_IN_SAMPLE;

                        if (writeHeader) memcpy(compressionBuffer, &wavHeader, sizeof(wavHeader_t));
                        
                        FLASH_LED_AND_RETURN_ON_ERROR(AudioMoth_writeToFile(compressionBuffer, NUMBER_OF_BYTES_IN_SAMPLE * numberOfSamples));
                        
                        if (writeHeader) memset(compressionBuffer, 0, sizeof(wavHeader_t));

                        numberOfBlankSamplesToWrite -= numberOfSamples;

                        blankBuffersWritten += 1;

                    }

                }

                // --> Introduced code: Log detections if probability exceeds threshold
		if (NNoutput > (float32_t)(THRESHOLD_DETECTION)){
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
					    BufferGreen = readBuffer;

				    } 
		//if (((readBuffer+1) % NUMBER_OF_BUFFERS_IN_SUPERBUFFER == 0) && (NNoutput < (float32_t)(THRESHOLD_DETECTION))) { // for each superbuffer{
		if ((readBuffer - BufferGreen) > NUMBER_OF_BUFFERS_IN_SUPERBUFFER) {
					    AudioMoth_setGreenLED(false);
		  }
	       // <--

                /* Clear LED */

                AudioMoth_setRedLED(false);

            }
            

            /* Increment buffer counters */

            readBuffer = (readBuffer + 1) & (NUMBER_OF_BUFFERS - 1);

            //samplesWritten += numberOfSamplesToWrite; // modified

            //buffersProcessed += 1; // modified

        }

        /* Check the voltage level */

        if (configSettings->enableLowVoltageCutoff && AudioMoth_isSupplyAboveThreshold() == false) {

            supplyVoltageLow = true;

        }

        /* Sleep until next DMA transfer is complete */

        AudioMoth_sleep();

    }

    /* Write the compression buffer files at the end */

    if (samplesWritten < numberOfSamples + numberOfSamplesInHeader && numberOfCompressedBuffers > 0) {

        /* Light LED during SD card write if appropriate */

        if (enableLED) AudioMoth_setRedLED(true);

        /* Encode and write compression buffer */

        encodeCompressionBuffer(numberOfCompressedBuffers);

        totalNumberOfCompressedSamples += (numberOfCompressedBuffers - 1) * COMPRESSION_BUFFER_SIZE_IN_BYTES / NUMBER_OF_BYTES_IN_SAMPLE;
       
        FLASH_LED_AND_RETURN_ON_ERROR(AudioMoth_writeToFile(compressionBuffer, COMPRESSION_BUFFER_SIZE_IN_BYTES));
        
        /* Clear LED */

        AudioMoth_setRedLED(false);

    }

    /* Determine recording state */

    recordingState = microphoneChanged ? MICROPHONE_CHANGED :
                     switchPositionChanged ? SWITCH_CHANGED :
                     magneticSwitch ? MAGNETIC_SWITCH :
                     supplyVoltageLow ? SUPPLY_VOLTAGE_LOW :
                     fileSizeLimited ? FILE_SIZE_LIMITED :
                     RECORDING_OKAY;

    /* Generate the new file name if necessary */
    
    static char newFilename[MAXIMUM_FILE_NAME_LENGTH];

    if (timeOffset > 0) {

        generateFolderAndFilename(foldername, newFilename, timeOfNextRecording + timeOffset, frequencyTriggerEnabled || amplitudeThresholdEnabled);

    }

    /* Write the GUANO data */

    bool gpsLocationReceived = getBackupFlag(BACKUP_GPS_LOCATION_RECEIVED);

    bool acousticLocationReceived = getBackupFlag(BACKUP_ACOUSTIC_LOCATION_RECEIVED);

    uint32_t guanoDataSize = writeGuanoData((char*)compressionBuffer, configSettings, timeOfNextRecording + timeOffset, gpsLocationReceived, gpsLastFixLatitude, gpsLastFixLongitude, acousticLocationReceived, acousticLatitude, acousticLongitude, firmwareDescription, firmwareVersion, (uint8_t*)AM_UNIQUE_ID_START_ADDRESS, deploymentID, defaultDeploymentID, timeOffset > 0 ? newFilename : filename, extendedBatteryState, temperature, requestedFilterType);

    FLASH_LED_AND_RETURN_ON_ERROR(AudioMoth_writeToFile(compressionBuffer, guanoDataSize));
    
    /* Initialise the WAV header */

    samplesWritten = MAX(numberOfSamplesInHeader, samplesWritten);

    setHeaderDetails(&wavHeader, effectiveSampleRate, samplesWritten - numberOfSamplesInHeader - totalNumberOfCompressedSamples, guanoDataSize);

    setHeaderComment(&wavHeader, configSettings, timeOfNextRecording + timeOffset, (uint8_t*)AM_UNIQUE_ID_START_ADDRESS, deploymentID, defaultDeploymentID, extendedBatteryState, temperature, externalMicrophone, recordingState, requestedFilterType);

    /* Write the header */

    if (enableLED) AudioMoth_setRedLED(true);

    FLASH_LED_AND_RETURN_ON_ERROR(AudioMoth_seekInFile(0));

    FLASH_LED_AND_RETURN_ON_ERROR(AudioMoth_writeToFile(&wavHeader, sizeof(wavHeader_t)));

    /* Close the file */

    FLASH_LED_AND_RETURN_ON_ERROR(AudioMoth_closeFile());

    AudioMoth_setRedLED(false);

    /* Rename the file if necessary */

    if (timeOffset > 0) {

        if (enableLED) AudioMoth_setRedLED(true);

        FLASH_LED_AND_RETURN_ON_ERROR(AudioMoth_renameFile(filename, newFilename));

        AudioMoth_setRedLED(false);

    }

    /* Return recording state */

    return recordingState;

}

/* Determine sunrise and sunset and schedule recording */

static void determineSunriseAndSunsetTimesAndScheduleRecording(uint32_t currentTime) {

    uint32_t scheduleTime = currentTime;

    uint32_t currentTimeOfNextRecording = *timeOfNextRecording;

    uint32_t currentStartOfRecordingPeriod = *startOfRecordingPeriod;

    /* Calculate initial sunrise and sunset time if appropriate */

    if (configSettings->enableSunRecording && *timeOfNextSunriseSunsetCalculation == 0) {

        determineSunriseAndSunsetTimes(scheduleTime);

        determineTimeOfNextSunriseSunsetCalculation(scheduleTime, timeOfNextSunriseSunsetCalculation);

        determineSunriseAndSunsetTimes(*timeOfNextSunriseSunsetCalculation - SECONDS_IN_DAY);

        determineTimeOfNextSunriseSunsetCalculation(scheduleTime, timeOfNextSunriseSunsetCalculation);

        scheduleRecording(scheduleTime, timeOfNextRecording, indexOfNextRecording, durationOfNextRecording, startOfRecordingPeriod, NULL);

        if (*timeOfNextSunriseSunsetCalculation < *timeOfNextRecording) *timeOfNextSunriseSunsetCalculation += SECONDS_IN_DAY;
    
    } else {

        scheduleRecording(scheduleTime, timeOfNextRecording, indexOfNextRecording, durationOfNextRecording, startOfRecordingPeriod, NULL);
  
    }

    /* Check if sunrise and sunset should be recalculated */

    if (configSettings->enableSunRecording && *timeOfNextRecording >= *timeOfNextSunriseSunsetCalculation) {
        
        scheduleTime = MAX(scheduleTime, *timeOfNextSunriseSunsetCalculation);

        determineSunriseAndSunsetTimes(scheduleTime);

        determineTimeOfNextSunriseSunsetCalculation(scheduleTime, timeOfNextSunriseSunsetCalculation);

        scheduleRecording(scheduleTime, timeOfNextRecording, indexOfNextRecording, durationOfNextRecording, startOfRecordingPeriod, NULL);

        if (*timeOfNextSunriseSunsetCalculation < *timeOfNextRecording) *timeOfNextSunriseSunsetCalculation += SECONDS_IN_DAY;

    }

    /* Finished if not using GPS */

    if (configSettings->enableTimeSettingFromGPS == false) return;

    /* Update GPS time setting time */
    
    bool shouldSetTimeFromGPS = false;

    uint32_t gpsTimeSettingPeriod = configSettings->gpsTimeSettingPeriod == 0 ? GPS_DEFAULT_TIME_SETTING_PERIOD : configSettings->gpsTimeSettingPeriod * SECONDS_IN_MINUTE;

    if (configSettings->enableTimeSettingBeforeAndAfterRecordings) {

        *timeOfNextGPSTimeSetting = *timeOfNextRecording - gpsTimeSettingPeriod;

        shouldSetTimeFromGPS = currentTimeOfNextRecording != UINT32_MAX && currentTimeOfNextRecording != *timeOfNextRecording;

    } else {

        *timeOfNextGPSTimeSetting = *startOfRecordingPeriod - gpsTimeSettingPeriod;

        shouldSetTimeFromGPS = currentStartOfRecordingPeriod != UINT32_MAX && currentStartOfRecordingPeriod != *startOfRecordingPeriod;

    }

    setBackupFlag(BACKUP_SHOULD_SET_TIME_FROM_GPS, shouldSetTimeFromGPS);

}

/* Determine sunrise and sunset calculation time */

static void determineTimeOfNextSunriseSunsetCalculation(uint32_t currentTime, uint32_t *timeOfNextSunriseSunsetCalculation) {

    /* Check if limited by earliest recording time */

    if (configSettings->earliestRecordingTime > 0) currentTime = MAX(currentTime, configSettings->earliestRecordingTime);

    /* Determine when the middle of largest gap between recording periods occurs */

    uint32_t startOfGapMinutes, endOfGapMinutes;

    if (*numberOfSunRecordingPeriods == 1) {

        startOfGapMinutes = firstSunRecordingPeriod->endMinutes;

        endOfGapMinutes = firstSunRecordingPeriod->startMinutes;

    } else {

        uint32_t gapFromFirstPeriodToSecondPeriod = secondSunRecordingPeriod->startMinutes - firstSunRecordingPeriod->endMinutes;

        uint32_t gapFromSecondPeriodsToFirstPeriod = secondSunRecordingPeriod->endMinutes < secondSunRecordingPeriod->startMinutes ? firstSunRecordingPeriod->startMinutes - secondSunRecordingPeriod->endMinutes : MINUTES_IN_DAY - secondSunRecordingPeriod->endMinutes + firstSunRecordingPeriod->startMinutes;

        if (gapFromFirstPeriodToSecondPeriod > gapFromSecondPeriodsToFirstPeriod) {

            startOfGapMinutes = firstSunRecordingPeriod->endMinutes;

            endOfGapMinutes = secondSunRecordingPeriod->startMinutes;

        } else {

            startOfGapMinutes = secondSunRecordingPeriod->endMinutes;

            endOfGapMinutes = firstSunRecordingPeriod->startMinutes;

        }

    }

    uint32_t calculationMinutes = endOfGapMinutes + startOfGapMinutes;

    if (endOfGapMinutes < startOfGapMinutes) calculationMinutes += MINUTES_IN_DAY;

    calculationMinutes /= 2;

    calculationMinutes = calculationMinutes % MINUTES_IN_DAY;

    /* Determine the number of seconds of this day */

    time_t rawTime = currentTime;

    struct tm *time = gmtime(&rawTime);

    uint32_t currentSeconds = SECONDS_IN_HOUR * time->tm_hour + SECONDS_IN_MINUTE * time->tm_min + time->tm_sec;

    /* Determine the time of the next sunrise and sunset calculation */

    uint32_t calculationTime = currentTime - currentSeconds + SECONDS_IN_MINUTE * calculationMinutes;

    if (calculationTime <= currentTime) calculationTime += SECONDS_IN_DAY;

    *timeOfNextSunriseSunsetCalculation = calculationTime;

}

/* Determine sunrise and sunset times */

static void determineSunriseAndSunsetTimes(uint32_t currentTime) {

    /* Calculate future sunrise and sunset time if recording is limited by earliest recording time */

    if (configSettings->earliestRecordingTime > 0) currentTime = MAX(currentTime, configSettings->earliestRecordingTime);

    /* Determine sunrise and sunset times */
    
    SR_trend_t trend;

    SR_solution_t solution;

    uint32_t sunriseMinutes, sunsetMinutes;

    float latitude = getBackupFlag(BACKUP_GPS_LOCATION_RECEIVED) ? (float)*gpsLatitude / (float)GPS_LOCATION_PRECISION : getBackupFlag(BACKUP_ACOUSTIC_LOCATION_RECEIVED) ? (float)*acousticLatitude / (float)ACOUSTIC_LOCATION_PRECISION : (float)configSettings->latitude / (float)CONFIG_LOCATION_PRECISION;

    float longitude = getBackupFlag(BACKUP_GPS_LOCATION_RECEIVED) ? (float)*gpsLongitude / (float)GPS_LOCATION_PRECISION : getBackupFlag(BACKUP_ACOUSTIC_LOCATION_RECEIVED) ? (float)*acousticLongitude / (float)ACOUSTIC_LOCATION_PRECISION : (float)configSettings->longitude / (float)CONFIG_LOCATION_PRECISION;

    Sunrise_calculateFromUnix(configSettings->sunRecordingEvent, currentTime, latitude, longitude, &solution, &trend, &sunriseMinutes, &sunsetMinutes);

    /* Calculate maximum recording duration */

    uint32_t minimumRecordingGap = MAX(MINIMUM_SUN_RECORDING_GAP, SUN_RECORDING_GAP_MULTIPLIER * configSettings->sunRoundingMinutes);

    uint32_t maximumRecordingDuration = MINUTES_IN_DAY - minimumRecordingGap;

    /* Round the sunrise and sunset times */

    uint32_t roundedSunriseMinutes = configSettings->sunRoundingMinutes > 0 ? UNSIGNED_ROUND(sunriseMinutes, configSettings->sunRoundingMinutes) : sunriseMinutes;

    uint32_t roundedSunsetMinutes = configSettings->sunRoundingMinutes > 0 ? UNSIGNED_ROUND(sunsetMinutes, configSettings->sunRoundingMinutes) : sunsetMinutes;

    /* Calculate start and end of potential recording periods */

    uint32_t beforeSunrise = (MINUTES_IN_DAY + roundedSunriseMinutes - configSettings->beforeSunriseMinutes) % MINUTES_IN_DAY;

    uint32_t afterSunrise = (MINUTES_IN_DAY + roundedSunriseMinutes + configSettings->afterSunriseMinutes) % MINUTES_IN_DAY;

    uint32_t beforeSunset = (MINUTES_IN_DAY + roundedSunsetMinutes - configSettings->beforeSunsetMinutes) % MINUTES_IN_DAY;

    uint32_t afterSunset = (MINUTES_IN_DAY + roundedSunsetMinutes + configSettings->afterSunsetMinutes) % MINUTES_IN_DAY;

    /* Determine schedule */

    recordingPeriod_t tempRecordingPeriod;
    
    if (configSettings->sunRecordingMode == SUNRISE_RECORDING) {

        /* Recording from before sunrise to after sunrise */

        *numberOfSunRecordingPeriods = 1;

        tempRecordingPeriod.startMinutes = beforeSunrise;
            
        tempRecordingPeriod.endMinutes = afterSunrise;
            
        copyToBackupDomain((uint32_t*)firstSunRecordingPeriod, (uint8_t*)&tempRecordingPeriod, sizeof(recordingPeriod_t));

    } else if (configSettings->sunRecordingMode == SUNSET_RECORDING) {

        /* Recording from before sunset to after sunset */

        *numberOfSunRecordingPeriods = 1;

        tempRecordingPeriod.startMinutes = beforeSunset;
            
        tempRecordingPeriod.endMinutes = afterSunset;

        copyToBackupDomain((uint32_t*)firstSunRecordingPeriod, (uint8_t*)&tempRecordingPeriod, sizeof(recordingPeriod_t));

    } else if (configSettings->sunRecordingMode == SUNRISE_AND_SUNSET_RECORDING) {

        /* Order the recording periods */

        uint32_t firstPeriodStartMinutes = beforeSunrise < beforeSunset ? beforeSunrise : beforeSunset;
        
        uint32_t firstPeriodEndMinutes = beforeSunrise < beforeSunset ? afterSunrise : afterSunset;
       
        uint32_t secondPeriodStartMinutes = beforeSunrise < beforeSunset ? beforeSunset : beforeSunrise;
        
        uint32_t secondPeriodEndMinutes  = beforeSunrise < beforeSunset ? afterSunset : afterSunrise;

        /* Determine whether the recording periods wrap */

        bool firstPeriodWraps = firstPeriodEndMinutes <= firstPeriodStartMinutes;

        bool secondPeriodWraps = secondPeriodEndMinutes <= secondPeriodStartMinutes;

        /* Combine recording periods together if they overlap */

        if (firstPeriodWraps) {

            *numberOfSunRecordingPeriods = 1;

            tempRecordingPeriod.startMinutes = firstPeriodStartMinutes;

            tempRecordingPeriod.endMinutes = secondPeriodWraps ? MIN(firstPeriodStartMinutes, MAX(firstPeriodEndMinutes, secondPeriodEndMinutes)) : firstPeriodEndMinutes;

            copyToBackupDomain((uint32_t*)firstSunRecordingPeriod, (uint8_t*)&tempRecordingPeriod, sizeof(recordingPeriod_t));

        } else if (secondPeriodStartMinutes <= firstPeriodEndMinutes) {

            *numberOfSunRecordingPeriods = 1;

            tempRecordingPeriod.startMinutes = firstPeriodStartMinutes;

            tempRecordingPeriod.endMinutes = secondPeriodWraps ? MIN(firstPeriodStartMinutes, secondPeriodEndMinutes) : MAX(firstPeriodEndMinutes, secondPeriodEndMinutes);

            copyToBackupDomain((uint32_t*)firstSunRecordingPeriod, (uint8_t*)&tempRecordingPeriod, sizeof(recordingPeriod_t));

        } else if (secondPeriodWraps && secondPeriodEndMinutes >= firstPeriodStartMinutes) {

            *numberOfSunRecordingPeriods = 1;

            tempRecordingPeriod.startMinutes = secondPeriodStartMinutes;

            tempRecordingPeriod.endMinutes = MAX(firstPeriodEndMinutes, secondPeriodEndMinutes);

            copyToBackupDomain((uint32_t*)firstSunRecordingPeriod, (uint8_t*)&tempRecordingPeriod, sizeof(recordingPeriod_t));

        } else {

            *numberOfSunRecordingPeriods = 2;

            tempRecordingPeriod.startMinutes = firstPeriodStartMinutes;

            tempRecordingPeriod.endMinutes = firstPeriodEndMinutes;

            copyToBackupDomain((uint32_t*)firstSunRecordingPeriod, (uint8_t*)&tempRecordingPeriod, sizeof(recordingPeriod_t));

            tempRecordingPeriod.startMinutes = secondPeriodStartMinutes;

            tempRecordingPeriod.endMinutes = secondPeriodEndMinutes;

            copyToBackupDomain((uint32_t*)secondSunRecordingPeriod, (uint8_t*)&tempRecordingPeriod, sizeof(recordingPeriod_t));

        }

        /* Adjust the size of the minimum gap between recording periods if it is less than the threshold  */

        if (*numberOfSunRecordingPeriods == 1) {

            uint32_t duration = firstSunRecordingPeriod->endMinutes <= firstSunRecordingPeriod->startMinutes ? MINUTES_IN_DAY + firstSunRecordingPeriod->endMinutes - firstSunRecordingPeriod->startMinutes : firstSunRecordingPeriod->endMinutes - firstSunRecordingPeriod->startMinutes;

            if (duration > maximumRecordingDuration) {

                tempRecordingPeriod.startMinutes = firstSunRecordingPeriod->startMinutes;

                tempRecordingPeriod.endMinutes = (firstSunRecordingPeriod->startMinutes + maximumRecordingDuration) % MINUTES_IN_DAY;

                copyToBackupDomain((uint32_t*)firstSunRecordingPeriod, (uint8_t*)&tempRecordingPeriod, sizeof(recordingPeriod_t));

            }

        }
        
        if (*numberOfSunRecordingPeriods == 2) {

            uint32_t gapFromFirstPeriodToSecondPeriod = secondSunRecordingPeriod->startMinutes - firstSunRecordingPeriod->endMinutes;

            uint32_t gapFromSecondPeriodsToFirstPeriod = secondSunRecordingPeriod->endMinutes < secondSunRecordingPeriod->startMinutes ? firstSunRecordingPeriod->startMinutes - secondSunRecordingPeriod->endMinutes : MINUTES_IN_DAY + firstSunRecordingPeriod->startMinutes - secondSunRecordingPeriod->endMinutes;

            if (gapFromFirstPeriodToSecondPeriod >= gapFromSecondPeriodsToFirstPeriod && gapFromFirstPeriodToSecondPeriod < minimumRecordingGap) {

                tempRecordingPeriod.startMinutes = firstSunRecordingPeriod->startMinutes;

                tempRecordingPeriod.endMinutes = (MINUTES_IN_DAY + firstSunRecordingPeriod->endMinutes - minimumRecordingGap + gapFromFirstPeriodToSecondPeriod) % MINUTES_IN_DAY;

                copyToBackupDomain((uint32_t*)firstSunRecordingPeriod, (uint8_t*)&tempRecordingPeriod, sizeof(recordingPeriod_t));

            } else if (gapFromSecondPeriodsToFirstPeriod >= gapFromFirstPeriodToSecondPeriod && gapFromSecondPeriodsToFirstPeriod < minimumRecordingGap) {

                tempRecordingPeriod.startMinutes = secondSunRecordingPeriod->startMinutes;

                tempRecordingPeriod.endMinutes = (MINUTES_IN_DAY + secondSunRecordingPeriod->endMinutes - minimumRecordingGap + gapFromSecondPeriodsToFirstPeriod) % MINUTES_IN_DAY;

                copyToBackupDomain((uint32_t*)secondSunRecordingPeriod, (uint8_t*)&tempRecordingPeriod, sizeof(recordingPeriod_t));

            }

        }

    } else if (configSettings->sunRecordingMode == SUNSET_TO_SUNRISE_RECORDING) {

        /* Recording from before sunset to after sunrise */

        *numberOfSunRecordingPeriods = 1;

        tempRecordingPeriod.startMinutes = beforeSunset;

        uint32_t timeFromSunsetToSunrise;

        if (roundedSunriseMinutes == roundedSunsetMinutes) {

            timeFromSunsetToSunrise = trend == SR_DAY_SHORTER_THAN_NIGHT ? MINUTES_IN_DAY : 0;

        } else {

            timeFromSunsetToSunrise = roundedSunriseMinutes < roundedSunsetMinutes ? MINUTES_IN_DAY + roundedSunriseMinutes - roundedSunsetMinutes : roundedSunriseMinutes - roundedSunsetMinutes;
        
        }
        
        uint32_t duration = timeFromSunsetToSunrise + configSettings->beforeSunsetMinutes + configSettings->afterSunriseMinutes;

        if (duration == 0) duration = 1;

        if (duration > maximumRecordingDuration) duration = maximumRecordingDuration;

        tempRecordingPeriod.endMinutes = (beforeSunset + duration) % MINUTES_IN_DAY;

        copyToBackupDomain((uint32_t*)firstSunRecordingPeriod, (uint8_t*)&tempRecordingPeriod, sizeof(recordingPeriod_t));

    } else if (configSettings->sunRecordingMode == SUNRISE_TO_SUNSET_RECORDING) {

        /* Recording from before sunrise to after sunset */

        *numberOfSunRecordingPeriods = 1;

        tempRecordingPeriod.startMinutes = beforeSunrise;

        uint32_t timeFromSunriseToSunset;

        if (roundedSunriseMinutes == roundedSunsetMinutes) {

            timeFromSunriseToSunset = trend == SR_DAY_LONGER_THAN_NIGHT ? MINUTES_IN_DAY : 0;

        } else {

            timeFromSunriseToSunset = roundedSunsetMinutes < roundedSunriseMinutes ? MINUTES_IN_DAY + roundedSunsetMinutes - roundedSunriseMinutes : roundedSunsetMinutes - roundedSunriseMinutes;

        }

        uint32_t duration = timeFromSunriseToSunset + configSettings->beforeSunriseMinutes + configSettings->afterSunsetMinutes;

        if (duration == 0) duration = 1;

        if (duration > maximumRecordingDuration) duration = maximumRecordingDuration;

        tempRecordingPeriod.endMinutes = (beforeSunrise + duration) % MINUTES_IN_DAY;
        
        copyToBackupDomain((uint32_t*)firstSunRecordingPeriod, (uint8_t*)&tempRecordingPeriod, sizeof(recordingPeriod_t));

    }

}

/* Schedule recordings */

static void adjustRecordingDuration(uint32_t *duration, uint32_t recordDuration, uint32_t sleepDuration) {

    uint32_t durationOfCycle = recordDuration + sleepDuration;

    uint32_t numberOfCycles = *duration / durationOfCycle;

    uint32_t partialCycle = *duration % durationOfCycle;

    if (partialCycle == 0) {

        *duration = *duration > sleepDuration ? *duration - sleepDuration : 0;

    } else {

        *duration = MIN(*duration, numberOfCycles * durationOfCycle + recordDuration);

    }

}

static void calculateStartAndDuration(uint32_t currentTime, uint32_t currentSeconds, recordingPeriod_t *period, uint32_t *startTime, uint32_t *duration) {

    *startTime = currentTime - currentSeconds + SECONDS_IN_MINUTE * period->startMinutes;

    *duration = period->endMinutes <= period->startMinutes ? MINUTES_IN_DAY + period->endMinutes - period->startMinutes : period->endMinutes - period->startMinutes;
    
    *duration *= SECONDS_IN_MINUTE;

}

static void scheduleRecording(uint32_t currentTime, uint32_t *timeOfNextRecording, uint32_t *indexOfNextRecording, uint32_t *durationOfNextRecording, uint32_t *startOfRecordingPeriod, uint32_t *endOfRecordingPeriod) {

    /* Enforce minumum schedule date */
    
    currentTime = MAX(currentTime, START_OF_CENTURY);

    /* Check if recording should be limited by earliest recording time */

    if (configSettings->earliestRecordingTime > 0) {

        currentTime = MAX(currentTime, configSettings->earliestRecordingTime);

    }

    /* Select appropriate recording periods */

    uint32_t activeRecordingPeriods = configSettings->enableSunRecording ? *numberOfSunRecordingPeriods : MIN(configSettings->activeRecordingPeriods, MAX_RECORDING_PERIODS);

    recordingPeriod_t *recordingPeriods = configSettings->enableSunRecording ? firstSunRecordingPeriod : configSettings->recordingPeriods;

    /* No suitable recording periods */

    if (activeRecordingPeriods == 0) {

        *timeOfNextRecording = UINT32_MAX;

        *indexOfNextRecording = 0;

        if (startOfRecordingPeriod) *startOfRecordingPeriod = UINT32_MAX;

        if (endOfRecordingPeriod) *endOfRecordingPeriod = UINT32_MAX;

        *durationOfNextRecording = 0;

        return;

    }

    /* Calculate the number of seconds of this day */

    time_t rawTime = currentTime;

    struct tm *time = gmtime(&rawTime);

    uint32_t currentSeconds = SECONDS_IN_HOUR * time->tm_hour + SECONDS_IN_MINUTE * time->tm_min + time->tm_sec;

    /* Check the last active period on the previous day */

    uint32_t startTime, duration;

    uint32_t index = activeRecordingPeriods - 1;

    recordingPeriod_t *lastPeriod = recordingPeriods + activeRecordingPeriods - 1;
    
    calculateStartAndDuration(currentTime - SECONDS_IN_DAY, currentSeconds, lastPeriod, &startTime, &duration);

    if (configSettings->disableSleepRecordCycle == false) {
    
        adjustRecordingDuration(&duration, configSettings->recordDuration, configSettings->sleepDuration);

    }

    if (currentTime < startTime + duration && duration > 0) goto done;

    /* Check each active recording period on the same day*/

    for (index = 0; index < activeRecordingPeriods; index += 1) {

        recordingPeriod_t *currentPeriod = recordingPeriods + index;

        calculateStartAndDuration(currentTime, currentSeconds, currentPeriod, &startTime, &duration);

        if (configSettings->disableSleepRecordCycle == false) {
        
            adjustRecordingDuration(&duration, configSettings->recordDuration, configSettings->sleepDuration);

        }
        
        if (currentTime < startTime + duration && duration > 0) goto done;

    }

    /* Calculate time until first period tomorrow */

    index = 0;

    recordingPeriod_t *firstPeriod = recordingPeriods;

    calculateStartAndDuration(currentTime + SECONDS_IN_DAY, currentSeconds, firstPeriod, &startTime, &duration);

    if (configSettings->disableSleepRecordCycle == false) {
    
        adjustRecordingDuration(&duration, configSettings->recordDuration, configSettings->sleepDuration);

    }

done:

    /* Set the time for start and end of the recording period */

    if (startOfRecordingPeriod) *startOfRecordingPeriod = startTime;

    if (endOfRecordingPeriod) *endOfRecordingPeriod = startTime + duration;

    /* Resolve sleep and record cycle */

    if (configSettings->disableSleepRecordCycle) {

        *timeOfNextRecording = startTime;

        *durationOfNextRecording = duration;

    } else {
        
        if (currentTime <= startTime) {

            /* Recording should start at the start of the recording period */

            *timeOfNextRecording = startTime;

            *durationOfNextRecording = MIN(duration, configSettings->recordDuration);

        } else {

            /* Recording should start immediately or at the start of the next recording cycle */

            uint32_t secondsFromStartOfPeriod = currentTime - startTime;

            uint32_t durationOfCycle = configSettings->recordDuration + configSettings->sleepDuration;

            uint32_t partialCycle = secondsFromStartOfPeriod % durationOfCycle;

            *timeOfNextRecording = currentTime - partialCycle;

            if (partialCycle >= configSettings->recordDuration) {

                /* Wait for next cycle to begin */

                *timeOfNextRecording += durationOfCycle;

            }

            uint32_t remainingDuration = startTime + duration - *timeOfNextRecording;

            *durationOfNextRecording = MIN(configSettings->recordDuration, remainingDuration);

        }

    }

    /* Update start time and duration is recording period has started */

    if (currentTime > *timeOfNextRecording) {

        *durationOfNextRecording -= currentTime - *timeOfNextRecording;

        *timeOfNextRecording = currentTime;
        
    }

    /* Check if recording should be limited by last recording time */

    uint32_t latestRecordingTime = configSettings->latestRecordingTime > 0 ? configSettings->latestRecordingTime : MIDPOINT_OF_CENTURY;

    if (*timeOfNextRecording >= latestRecordingTime) {

        *timeOfNextRecording = UINT32_MAX;

        if (startOfRecordingPeriod) *startOfRecordingPeriod = UINT32_MAX;

        if (endOfRecordingPeriod) *endOfRecordingPeriod = UINT32_MAX;

        *durationOfNextRecording = 0;

    } else {

        int64_t excessTime = (int64_t)*timeOfNextRecording + (int64_t)*durationOfNextRecording - (int64_t)latestRecordingTime;

        if (excessTime > 0) {
            
            *durationOfNextRecording -= excessTime;

            if (endOfRecordingPeriod) *endOfRecordingPeriod = *timeOfNextRecording + *durationOfNextRecording;

        }

    }

    /* Set the index of the next recording */

    *indexOfNextRecording = index;

}

/* Flash LED according to battery life */

static void flashLedToIndicateBatteryLife(void) {

    uint32_t numberOfFlashes = LOW_BATTERY_LED_FLASHES;

    uint32_t supplyVoltage = AudioMoth_getSupplyVoltage();

    if (configSettings->batteryLevelDisplayType == NIMH_LIPO_BATTERY_VOLTAGE) {

        /* Set number of flashes according to battery voltage */

        AM_extendedBatteryState_t batteryState = AudioMoth_getExtendedBatteryState(supplyVoltage);

        if (batteryState > AM_EXT_BAT_4V3) {

            numberOfFlashes = 1;

        } else if (batteryState > AM_EXT_BAT_3V5) {

            numberOfFlashes = AM_EXT_BAT_4V4 - batteryState;

        }

    } else {

        /* Set number of flashes according to battery state */

        AM_batteryState_t batteryState = AudioMoth_getBatteryState(supplyVoltage);

        if (batteryState > AM_BATTERY_LOW) {

            numberOfFlashes = (batteryState >= AM_BATTERY_4V6) ? 4 : (batteryState >= AM_BATTERY_4V4) ? 3 : (batteryState >= AM_BATTERY_4V0) ? 2 : 1;

        }

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
	static const float32_t b1[2] = {-1.6506595038998270741, 1.0714025936080897594};
	static const float32_t b2[2] = {0.71983910097112435711, 0.42310377090373713083};
	
	static const float32_t A1[48] = {-0.14701604449555649712, 0.012510029384784456669, -0.10765188152252687381, 0.071180338776944462875, 0.061337039759318652543, 0.07557964594909946654, -0.052529524209502437282, -0.18343187601660726482, -0.028753465003183625859, 0.14929364750239063064, -0.22476698332039823924, 0.22934918030278503287, -0.05035400237997162548, -0.15055370722294802999, -0.3923630242629779219, -0.13481919076067130914, -0.63449351419339827807, 0.56403213635115156954, 0.21292134952589161778, 0.057119735283437930717, -0.22679689855485193895, 0.045036925887866785523, -0.32470570602237780466, -0.018139349505913043153, 0.68511045408493331799, 0.49584749848072662282, 0.67755136527226134113, 0.069789348142964946486, -0.48259518349628083289, 0.31833828044030243465, -0.36685163332242770595, 0.32092691603317674565, -0.031015556069871316747, -0.096091285828796710322, -0.19435447514644649258, -0.15616056365643254944, -0.15479064083304672206, -0.14544622046057389952, -0.029775084319359788887, -0.25126427067816187177, 0.24481594574285836519, 0.47007313654123988877, 0.10662993499324295577, 0.57975668297760285519, 0.30475582538514162101, 0.11974450165227325249, 0.57213684340161352626, 0.51677005424638533526};
	
	static const float32_t A2[4] = {-0.14048335432837033565, -1.3336896431931430929, -0.35913970340071038612, 0.34787595174652785612};
	
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
