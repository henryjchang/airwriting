#include "CurieIMU.h"
#include "CurieTimer.h"
#include "math.h"
#include "PlainFFT.h"

////MODIFY THESE PARAMETERS////////////////////////////
byte writing = 0;
#define WINDOW_LENGTH  128 //128 //sliding window length in units of samples. Must be a power of 2 for FFT
float sampling_period = 2000; //7000; //2000;
///////////////////////////////////////////////////////////
//WINDOW_LENGTH = 128, sampling_period = 1000 works if print i several times per window


//These three used for printing out abscissa of data
#define SCL_INDEX 0x00
#define SCL_TIME 0x01
#define SCL_FREQUENCY 0x02


bool calibrateOffsets = 1; // int to determine whether calibration takes place or not

int ax, ay, az;         // accelerometer values
int gx, gy, gz;         // gyrometer values

const short oneMilliSec = 1000; //One millisecond
float oneSec = 1000000; //One second

float samplingFrequency = 1/sampling_period*1000000; //in units of per second

short window_data[WINDOW_LENGTH][6] = {0};
short window_data_full[WINDOW_LENGTH][6] = {0};


bool new_full_window = 0; //toggle to determine if should calculate features on full window data

float avg_ang_vel = 0; //initialize average angular velocity
float avg_mean_shift_accel = 0; //initialize average mean shifted acceleration
short freq_max_pow[8]; //initialize the 8 bins of frequency power sptrum from 0-8Hz

short samples = 0; //not to be confused with IMU samples. This is used for number of samples for the SVM
float features[10];

float * frequency_spectrum_x;
float * frequency_spectrum_y;
float * frequency_spectrum_z;

float * computeFFT(short data_sample[WINDOW_LENGTH][6], byte vector);


short i = 0; //Increment this to know when the window is filled
void timed_IMU_ISR()
{
  //float isr_time_start = micros();
  // read raw accel/gyro measurements from device
  CurieIMU.readMotionSensor(ax, ay, az, gx, gy, gz);
  if (i < WINDOW_LENGTH){
    
      //want to fill an array ("window") before performing any computation
      window_data[i][0] = ax;
      window_data[i][1] = ay;
      window_data[i][2] = az;
      window_data[i][3] = gx;
      window_data[i][4] = gy;
      window_data[i][5] = gz;
  }
  else{
    new_full_window = 1;
    i = -1;
  }
  i++;
  //float isr_time_end = micros();
  //Serial.print("isr time: ");
  //Serial.println(isr_time_end-isr_time_start);
  //ISR time: ~100 microseconds
  if ( i % 74 == 0){
    Serial.println(i);
  }
}


void setup() {
  Serial.begin(115200); // initialize Serial communication
  while (!Serial);    // wait for the serial port to open

  // initialize device
  Serial.println(F("Initializing IMU device..."));
  CurieIMU.begin();

  // verify connection
  Serial.println(F("Testing device connections..."));
  if (CurieIMU.begin()) {
    Serial.println(F("CurieIMU connection successful"));
  } else {
    Serial.println(F("CurieIMU connection failed"));
  }

  //leave on flat surface until calibration is done
  if(calibrateOffsets == 1){
    calibrateIMU();  
  }

  Serial.print("sampling period: ");
  Serial.println(sampling_period);
  
  while(!Serial.available());
  char UserIsReady = Serial.read(); 
  Serial.println(UserIsReady);
  //At this point, device should be on wrist
  
  //Sample at 1000 Hz
  CurieTimerOne.start(sampling_period, &timed_IMU_ISR);
  Serial.println("Started timed_IMU_ISR");

}


short i_current = 0;

void loop() {  
  if(new_full_window){
    float feature_time_start = micros();

    memcpy(window_data_full, window_data, WINDOW_LENGTH);
    // Feature extraction on window_data_full
    float avg_accel = 0;
    
    // Calculate average angular velocity and average acceleration
    for(int j = 0; j < WINDOW_LENGTH; j++){
      //Serial.print("the inside");
      //float tic = micros();
      double ax_val = (double)window_data_full[j][0];
      double ay_val = (double)window_data_full[j][1];
      double az_val = (double)window_data_full[j][2];
      double gx_val = (double)window_data_full[j][3];
      double gy_val = (double)window_data_full[j][4];
      double gz_val = (double)window_data_full[j][5];
      
      avg_ang_vel += (float)sqrt(gx_val*gx_val + gy_val*gy_val + gz_val*gz_val);
      avg_accel += (float)sqrt(ax_val*ax_val + ay_val*ay_val + az_val*az_val);
      //float toc = micros();
      //Serial.println(toc-tic);
      // Each is maximum ~700 microseconds (.0007 sec)
    }
    avg_ang_vel = avg_ang_vel/WINDOW_LENGTH; //FEATURE1: avg_ang_vel
    avg_accel = avg_accel/WINDOW_LENGTH;
    //In total ~70 milliseconds
    
//    
//    // Calculate average mean shifted acceleration
//    for(uint8_t j = 0; j < WINDOW_LENGTH; j++){
//      double ax_val = (double)window_data_full[j][0];
//      double ay_val = (double)window_data_full[j][1];
//      double az_val = (double)window_data_full[j][2];
//
//      avg_mean_shift_accel += fabs(sqrt(ax_val*ax_val + ay_val*ay_val + az_val*az_val - avg_accel));
//    }
//    avg_mean_shift_accel = (float)1/WINDOW_LENGTH*avg_mean_shift_accel; //FEATURE2: avg_mean_shift_accel
//    //In total ~40 milliseconds

    features[0] = (short)avg_ang_vel;
    //features[1] = (short)avg_mean_shift_accel;
    features[1] = (short)avg_accel;
    //Get frequency spectrum

    //float startFFT_time = micros();
    frequency_spectrum_x = computeFFT(window_data_full, 0);
    //frequency_spectrum_y = computeFFT(window_data_full, 1);
    //frequency_spectrum_z = computeFFT(window_data_full, 2);
    //PrintVector(frequency_spectrum, WINDOW_LENGTH/2, SCL_FREQUENCY);
    //float endFFT_time = micros();
    //Serial.print("FFT runtime:");
    //Serial.println(endFFT_time - startFFT_time);
    //computeFFT: ~94 milliseconds


    //float bin_start = micros();
    float abscissa[WINDOW_LENGTH/2] = {0};
    for (uint8_t k = 0; k < WINDOW_LENGTH/2; k++){
      abscissa[k] = ((k * 1.0 * samplingFrequency) / WINDOW_LENGTH);
      //Serial.println(abscissa[k]);
    }
    //Serial.println("now we here");
    //Get FEATURE3-10
    uint8_t k = 0;
    float freq_bin_avg[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    for(short bin = 0; bin < 8; bin++){
        short count = 0;
        //Serial.println("new bin");
        //Serial.println(abscissa[k]);
        while((abscissa[k] >= bin) && (abscissa[k] < bin+1)){
            //freq_bin_avg[bin] += frequency_spectrum_x[k] + frequency_spectrum_y[k] + frequency_spectrum_z[k];
            freq_bin_avg[bin] += frequency_spectrum_x[k];
            k++;
            count++;
            //Serial.println(freq_bin_avg[bin]);
        }
        features[bin+2] = freq_bin_avg[bin]/count; //get the average. 
    }
    //float bin_end = micros();
    //Serial.print("Frequency bin time:");
    //Serial.println(bin_end-bin_start);
    // ~3 milliseconds
    
    samples++;
    //Serial.print(F("samples: "));
    //Serial.println(samples);
     /* Print features in LibSVM format. 
     * Remember to add either 0 or 1 on the far left of each row to signify if that sample has handwriting or not
     */
    if(writing){
      Serial.print("1 ");
      PrintFeatures();
    }
    else{
      //float writing_start = micros();
      Serial.print("0 ");
      PrintFeatures();
      //float writing_end = micros();
      //Serial.print("writing time: ");
      //Serial.println(writing_end-writing_start);
      //~10 milliseconds
    }
 
    new_full_window = 0;
    float feature_time_end = micros();
    //Serial.print("Feature extraction time: ");
    //Serial.println(feature_time_end - feature_time_start);
    //Serial.print("i = ");
    //Serial.println(i);
  }
  //Stop after 20 seconds to come up with new sentences or switch to collecting samples with no hadwriting
  //if switching to samples without writing, remember to comment out WRITING defined at the top
//  if(samples != 0 && samples % 5 == 0){
//    CurieTimerOne.pause();
//    while(!Serial.available());
//    char userInput = Serial.read(); //type anything to continue
//    Serial.println(userInput);
//    if(userInput == '1'){
//      writing = 1;
//    }
//    else{writing = 0;}
//
//    //reset i to start a new window
//    i = 0;
//    CurieTimerOne.resume();
//  }

}

void calibrateIMU(){
    Serial.println("About to calibrate. Make sure your board is stable and upright");
    delay(5000);

    // The board must be resting in a horizontal position for
    // the following calibration procedure to work correctly!
    Serial.print(F("Starting Gyroscope calibration..."));
    CurieIMU.autoCalibrateGyroOffset();
    Serial.println(" Done");

    Serial.print(F("Starting Acceleration calibration..."));
    CurieIMU.autoCalibrateAccelerometerOffset(X_AXIS, 0);
    CurieIMU.autoCalibrateAccelerometerOffset(Y_AXIS, 0);
    CurieIMU.autoCalibrateAccelerometerOffset(Z_AXIS, 1);
    Serial.println(" Done");

    Serial.println("Internal sensor offsets AFTER calibration...");
    Serial.print(CurieIMU.getAccelerometerOffset(X_AXIS));
    Serial.print("\t"); // -76
    Serial.print(CurieIMU.getAccelerometerOffset(Y_AXIS));
    Serial.print("\t"); // -2359
    Serial.print(CurieIMU.getAccelerometerOffset(Z_AXIS));
    Serial.print("\t"); // 1688
    Serial.print(CurieIMU.getGyroOffset(X_AXIS));
    Serial.print("\t"); // 0
    Serial.print(CurieIMU.getGyroOffset(Y_AXIS));
    Serial.print("\t"); // 0
    Serial.println(CurieIMU.getGyroOffset(Z_AXIS));

    Serial.println("Enabling Gyroscope/Acceleration offset compensation");
    CurieIMU.enableGyroOffset(true);
    CurieIMU.enableAccelerometerOffset(true);

    Serial.println(CurieIMU.accelerometerOffsetEnabled());
    Serial.println(CurieIMU.gyroOffsetEnabled());
}

float * computeFFT(short data_sample[WINDOW_LENGTH][6], byte vector){
  // data_sample is the 2D array of ax,ay,az,gx,gy,gz
  // vector: choose 0-5, corresponding to ax,ay,az,gx,gy,gz

  //Serial.print("LEGEN");
  PlainFFT FFT = PlainFFT(); /* Create FFT object */
  /* 
  These are the input and output vectors 
  Input vectors receive computed results from FFT
  */
  static float vReal[WINDOW_LENGTH] = {0}; 
  //Serial.println("i'm not too big yet");
  float vImag[WINDOW_LENGTH] = {0}; //Need this as an intermediate since FFT outputs in complex numbers
  //Serial.println("still not too big");
  for(int num = 0; num < WINDOW_LENGTH; num++){
    vReal[num] = (float)data_sample[num][vector];
    //Serial.println(vReal[num]);  
  }

  //Comment out the PrintVectors - use for debugging only
  //PrintVector(vReal, WINDOW_LENGTH, SCL_TIME);
  //Serial.print("Hello ");
  FFT.Windowing(vReal, WINDOW_LENGTH, FFT_WIN_TYP_HAMMING, FFT_FORWARD);        /* Weigh data */  
  //Serial.print("from ");
  FFT.Compute(vReal, vImag, WINDOW_LENGTH, FFT_FORWARD); /* Compute FFT */
  //Serial.print("the ");
  //PrintVector(vReal, WINDOW_LENGTH, SCL_FREQUENCY);
  //PrintVector(vImag, WINDOW_LENGTH, SCL_FREQUENCY);
  FFT.ComplexToMagnitude(vReal, vImag, WINDOW_LENGTH); /* Compute magnitudes */
  //Serial.println("other side");
  //PrintVector(vReal, WINDOW_LENGTH, SCL_FREQUENCY);
  

  //Serial.println("-DARY");
  
  return vReal;
}

//Print x-axis in terms of index, time, or frequency
//Borrowed from Didier Longueville in FFT_01.pde
void PrintVector(float *vData, uint16_t bufferSize, uint8_t scaleType) 
{       
        //Serial.println("PrintVector");
        for (uint16_t bufferVal = 0; bufferVal < bufferSize; bufferVal++) {
                double abscissa;
                /* Print abscissa value */
                switch (scaleType) {
                case SCL_INDEX:
                        abscissa = (bufferVal * 1.0);
                        break;
                case SCL_TIME:
                        abscissa = ((bufferVal * 1.0) / samplingFrequency);
                        break;
                case SCL_FREQUENCY:
                        abscissa = ((bufferVal * 1.0 * samplingFrequency) / WINDOW_LENGTH);
                        break;
                }
           
                Serial.print(abscissa, 6);
                Serial.print(F(" "));
                Serial.print(vData[bufferVal], 4);
                Serial.println();
        }
        Serial.println();
}

void PrintFeatures(void){
  //1:features[0] 2:features[1] 3:features[2] 4:features[3] 5:features[4] 6:features[5] 7:features[6] 8:features[7] 9:features[8] 10:features[9]);
  Serial.print(F("1:"));
  Serial.print(features[0]);
  Serial.print(F(" 2:"));
  Serial.print(features[1]);
  Serial.print(F(" 3:"));
  Serial.print(features[2]);
  Serial.print(F(" 4:"));
  Serial.print(features[3]);
  Serial.print(F(" 5:"));
  Serial.print(features[4]);
  Serial.print(F(" 6:"));
  Serial.print(features[5]);
  Serial.print(F(" 7:"));
  Serial.print(features[6]);
  Serial.print(F(" 8:"));
  Serial.print(features[7]);
  Serial.print(F(" 9:"));
  Serial.print(features[8]);
  Serial.print(F(" 10:"));
  Serial.println(features[9]);
}


