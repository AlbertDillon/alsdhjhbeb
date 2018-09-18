/**************************** Private Definitions *****************************/
#define RAW_IN_PIN              14 // A0
#define FILT_OUT_PIN            A14 // DAC
#define LED_QRS_PIN             13 // On-board LED
#define USART_BAUD_RATE         512000
#define ADC_RESOLUTION          12 // Maximum 13bit input, 12bit output

#define FILTER_LENGTH           100
#define STORED_DATA_LENGTH      100

#define SAMPLE_FREQUENCY        200
#define SAMPLE_PERIOD           (1000000 / SAMPLE_FREQUENCY)

#define CONSOLE_LOG             1

/******************************* Private Types ********************************/
typedef enum {
  LOWERED,
  RAISED
} Flag;

typedef struct {
    int filter[FILTER_LENGTH];
    int storedInput[STORED_DATA_LENGTH];
    int storedOutput[STORED_DATA_LENGTH];
    int in;
    int out;
    Flag QRSFlag;
    int currentRR;
    int lastRR;                             // Average of last 8 RR intervals
    int lastRRTune;                         // Only 92% < currentRR < 116%
} PanTompkin;

/************************** Private Global Variables **************************/
PanTompkin Data;
int CurrentTime, LastTime;

/************************ Private Function Prototypes *************************/
int apply_filter(int dataIn);
void create_filter(void);

/**************************** Function Definitions ****************************/

/*
    Initial Setup
*/
void setup(void) {
    /* Initialise analog input, analog output, QRS LED and USART pins */
    analogReadResolution(ADC_RESOLUTION);
    pinMode(FILT_OUT_PIN, OUTPUT);
    pinMode(LED_QRS_PIN, OUTPUT);

    Serial.begin(USART_BAUD_RATE);

    create_filter(Data.filter, FILTER_LENGTH);

    LastTime = 0;
    Data.in = 0;
    Data.out = 0;
}

/*
    Processing Loop
*/
void loop(void) {
    CurrentTime = micros();

    if ((CurrentTime - LastTime > SAMPLE_PERIOD) ||
            (LastTime - CurrentTime > SAMPLE_PERIOD)) {
        // Absolute difference > Sample period
        LastTime = micros();

        if (Data.QRSFlag == RAISED) {
            digitalWrite(LED_QRS_PIN, LOW);
            Data.QRSFlag = LOWERED;
        }

        Data.in = analogRead(RAW_IN_PIN);

        apply_filter(&Data);

        detect_qrs(&Data);

        analogWrite(FILT_OUT_PIN, Data.out);

        /* Serial logging */
        if (CONSOLE_LOG) {
            Serial.println(Data.in);
            // Serial.println(Data.out)
        }
        
    }
}

/*
    Takes a data point as an integer.  Applies digital filters.  Returns next 
    output.
*/
int apply_filter(PanTompkin* data) {
  
    return data->out;
}

/*
    Populates filter.
*/
void create_filter(int* filter, unsigned length) {
    int T = SAMPLE_PERIOD;

    // Low-Pass Filter
    // y(nT) = 2 * y(nT - T) - y(nT-2T) + x(nT) - 2 * x(nT-6T) + x(nT - 12T);

    // High-Pass Filter
    // y(nT) = 32 * x(nT - 16T) - (y(nT - T) + x(nT) - x(nT - 32T));

    // Derivative Filter
    // y(nT) = (1/8T)*(-x(nT - 2T) - 2x(nT - T) + 2x(nT + T) + x(nT + 2T));

    // Squaring Function
    // y(nT) = x(nT) * x(nT);

    int N = 10; // Integration window size
    // Moving-Window Integration
    // y(nT) = (x[nT - (N-1)*T] + x[nT - (N-2)*T] + ... + x(nT))/N;

    // Fiducial Mark

    // Adjusting Thresholds

    /*
    peakI // Overall peak

    // running estimate of signal peak
    SPKI = (peakI + 7 * SPKI) / 8; // peakI is signal peak

    // Running estimate of noise peak.  Relates to any peak that is not QRS (e.g. T)
    NPKI = (peakI + 7 * NPKI) / 8; // peakI is noise peak

    // First threshold applied
    ThresholdI1 = NPKI + 0.25(SPKI - NPKI);

    // Second threshold applied (Search-back technique)
    ThresholdI2 = 0.5 * ThresholdI1;

    // Signal peak if exceeds ThresholdI1 (first pass) or ThresholdI2 on searchback.

    // When found on searchback:
    SPKI = (peakI + 3 * SPKI) / 4;
    */

    /*

    peakF // Overall peak

    // Running estimate of signal peak.
    SPKF = (peakF + 7 * SPKF) / 8;

    // Running estimate of noise peak.
    NPKF = (peakF + 7* NPKF) / 8;

    // First threshold applied
    ThresholdF1 = NPKF + (SPKF - NPKF) / 4;

    // Second thresohld applied
    ThresholdF2 = ThresholdF2 / 2;

    // When found on searchback:
    SPKF = (peakF + 3 * SPKF) / 4;
    */

    // Adjusting Average RR Interval and Rate limits
    // RRAve = (Rn + ... + R[n-7]) / 8

    // 
    // RRAveTune = (Rn' + ... + R[n-7]) / 8;
    // Where Rn' are 


    for (int i = 0; i < length; i++) {
        filter[i] = 0;
    }
    return;
}

/*
    Pan-Tompkin method.
*/
void detect_qrs(PanTompkin* data) {
    if (data->in == 0) {
        digitalWrite(LED_QRS_PIN, HIGH);
        data->QRSFlag = RAISED;
    }
    return;
}
