/**************************** Private Definitions *****************************/
#define RAW_IN_PIN              14 // A0
#define FILT_OUT_PIN            A14 // DAC
#define LED_QRS_PIN             13 // On-board LED
#define USART_BAUD_RATE         512000
#define ADC_RESOLUTION          12 // Maximum 13bit input, 12bit output

#define FILTER_LENGTH           100
#define STORED_DATA_LENGTH      100
#define INTEGRATION_WINDOW      10

#define SAMPLE_FREQUENCY        200
#define SAMPLE_PERIOD           (1 / SAMPLE_FREQUENCY)
#define SAMPLE_PERIOD_MS        (1000 / SAMPLE_FREQUENCY)
#define SAMPLE_PERIOD_US        (1000000 / SAMPLE_FREQUENCY)

#define CONSOLE_LOG             1

/******************************* Private Types ********************************/
typedef enum {
  LOWERED,
  RAISED
} Flag;

typedef struct {
    int storedIn[STORED_DATA_LENGTH];
    int LPOut[STORED_DATA_LENGTH];
    int HPOut[STORED_DATA_LENGTH];
    int derOut[STORED_DATA_LENGTH];
    int squOut[STORED_DATA_LENGTH];
    int MWOut[STORED_DATA_LENGTH];
    int storedOut[STORED_DATA_LENGTH];
    int index;
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
void apply_filter(PanTompkin* data);
void low_pass(int* x, int* y, int length, int i);
void high_pass(int* x, int* y, int length, int i);
void derivative(int* x, int* y, int length, int i);
void squaring(int* x, int* y, int length, int i);
void moving_window(int* x, int* y, int length, int i);

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

    if ((CurrentTime - LastTime > SAMPLE_PERIOD_US) ||
            (LastTime - CurrentTime > SAMPLE_PERIOD_US)) {
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
void apply_filters(PanTompkin* data) {
    low_pass(data->storedIn, data->LPOut, STORED_DATA_LENGTH, data->index);
    high_pass(data->LPOut, data->HPOut, STORED_DATA_LENGTH, data->index);
    derivative(data->HPOut, data->derOut, STORED_DATA_LENGTH, data->index);
    squaring(data->derOut, data->squOut, STORED_DATA_LENGTH, data->index);
    moving_window(data->squOut, data->MWOut, STORED_DATA_LENGTH, data->index);

    return;
}

void low_pass(int* x, int* y, int length, int i) {
    // Low-Pass Filter
    int a = (i - 1)  % length;
    int b = (i - 2)  % length;
    int c = (i - 6)  % length;
    int d = (i - 12) % length;
    
    // y(nT) = 2 * y(nT - T) - y(nT-2T) + x(nT) - 2 * x(nT-6T) + x(nT - 12T);
    y[i] = 2 * y[a] - y[b] + x[i] - 2 * x[c] + x[d];

    return;
}

void high_pass(int* x, int* y, int length, int i) {
    // High-Pass Filter
    int a = (i - 16) % STORED_DATA_LENGTH;
    int b = (i - 1)  % STORED_DATA_LENGTH;
    int c = (i - 32) % STORED_DATA_LENGTH;

    // y(nT) = 32 * x(nT - 16T) - (y(nT - T) + x(nT) - x(nT - 32T));
    y[i] = 32 * x[a] - (y[b] + x[i] - x[c]);

    return;
}

void derivative(int* x, int* y, int length, int i) {
    // Derivative filter

    int a = (i - 4) % STORED_DATA_LENGTH;
    int b = (i - 3) % STORED_DATA_LENGTH;
    int c = (i - 1) % STORED_DATA_LENGTH;

    // y(nT) = (1/8T)*(-x(nT - 2T) - 2x(nT - T) + 2x(nT + T) + x(nT + 2T));
    y[i] = SAMPLE_FREQUENCY * (- x[a] - 2 * x[b] + 2 * x[c] + x[i]) / 8;

    return;
}

void squaring(int* x, int* y, int length, int i) {
    // Squaring Function

    // y(nT) = x(nT) * x(nT);
    y[i] = x[i] * x[i];

    return;
}

void moving_window(int* x, int* y, int length, int i) {
    // Moving-Window Integration

    // y(nT) = (x[nT - (N-1)*T] + x[nT - (N-2)*T] + ... + x(nT))/N;
    y[i] = 0;
    for (int a = 0; a < INTEGRATION_WINDOW; a++) {
        y[i] += x[(i - a) % STORED_DATA_LENGTH)];
    }
    y[i] /= INTEGRATION_WINDOW;

    return;
}

void peak_i_threshold(void) {
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
    return;
}

void peak_f_threshold(void) {
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
    return;
}

void rr_interval(void) {
    // Adjusting Average RR Interval and Rate limits
    // RRAve = (Rn + ... + R[n-7]) / 8

    // 
    // RRAveTune = (Rn' + ... + R[n-7]) / 8;
    // tuneUpper = (Rn * 116) / 100;
    // tuneLower = (Rn * 92) / 100;
    // tuneMissed = (Rn * 166) / 100;
    // if complex not found during interval specified by tuneMissed,
    //  -> Take maximal peak between established thresholds is QRS candidate.
    // if each of 8 RR intervals from RRAve within specified interval,
    // -> Heart Rate regular. RRAveTune = RRAve. 
    

    // Twave identification
    // RR interval <360ms >200ms, justify whether QRS or t-wave.
    // If maximal slope during waveform < previous QRS / 2, T-wave, otherwise
    // QRS.
    return;
}

void detect_peaks(PanTompkin* data) {

    peak_i_threshold();
    peak_f_threshold();
    rr_interval();    

    return;
}

/*
    Populates filter.
*/
void create_filter(int* filter, unsigned length) {

    


    for (int i = 0; i < length; i++) {
        filter[i] = 0;
    }
    return;
}