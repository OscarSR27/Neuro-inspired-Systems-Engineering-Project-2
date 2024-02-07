#include <actuator.h>
#include <Dynamixel2Arduino.h>
#include <DynamixelSDK.h>
#include <Arduino.h>
#define _USE_MATH_DEFINES
#include <math.h>

#define DESCRIPTION_LENGTH              15
#define NUMBER_PHASE_NEURONS            7
#define ARRAY_LENGTH(array) (sizeof(array) / sizeof((array)[0]))
#define ADDR_AX_ID                      3                 
#define ADDR_AX_TORQUE_ENABLE           24                 // Control table address is different in Dynamixel model
#define ADDR_AX_GOAL_POSITION           30
#define ADDR_AX_PRESENT_POSITION        36
#define MOVING                          46

// Protocol version
#define PROTOCOL_VERSION                1.0                 // See which protocol version is used in the Dynamixel

#define BAUDRATE                        1000000
#define DEVICENAME                      "1"                 //DEVICENAME "1" -> Serial1(OpenCM9.04 DXL TTL Ports)
                                                            //DEVICENAME "2" -> Serial2
                                                            //DEVICENAME "3" -> Serial3(OpenCM 485 EXP)
#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque
#define DXL_MINIMUM_POSITION_VALUE      0                 // Dynamixel will rotate between this value
#define DXL_MAXIMUM_POSITION_VALUE      1000                 // and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
#define DXL_MOVING_STATUS_THRESHOLD     20                  // Dynamixel moving status threshold

#define ESC_ASCII_VALUE                 0x1b

// Create PortHandler instance
dynamixel::PortHandler *portHandler;

// Create PacketHandler instance
dynamixel::PacketHandler *packetHandler;

//***********Set Global Variables****************
// Motor
uint16_t goal_position[NUMBER_PHASE_NEURONS];
int dxl_comm_result = COMM_TX_FAIL;             // Communication result
uint8_t dxl_error = 0;                          // Dynamixel error
int16_t dxl_present_position = 0;               // Present position

// Sensors
int const number_readings = 2;
int analogValueA1_Baseline;
int analogValueA2_Baseline;
int analogValueA3_Baseline;
int analogValueA4_Baseline;
int analogValueA5_Baseline;
int analogValueA6_Baseline;
int analogValueA1[number_readings];
int analogValueA2[number_readings];
int analogValueA3[number_readings];
int analogValueA4[number_readings];
int analogValueA5[number_readings];
int analogValueA6[number_readings];
int A1_counter = 0;
int A2_counter = 0;
int A3_counter = 0;
int A4_counter = 0;
int A5_counter = 0;
int A6_counter = 0;

bool A1_stable_reading = false;
bool A2_stable_reading = false;
bool A3_stable_reading = false;
bool A4_stable_reading = false;
bool A5_stable_reading = false;
bool A6_stable_reading = false;

// Control Logic
int trajectory_offset = 511;
bool amplitude_flag = false;
unsigned long int myTime;
unsigned int mydelay = 10; // ms
int current_time_rotation = 0;
int current_time_amplitude = 0;
int rigth_counter = 0;
int left_counter = 0;
float delta_t_amplitude = 5000;
float delta_t_rotation = 2500;
int const sensitivity_1 = 15;
int const sensitivity_2 = 25;

//Pattern
double w = 10;
double default_amp = 120; // amplitude standard movement
double default_tau = 0.2; // tau standard movement
double narrow_amp = 60;   // amplitude narrow movement
double narrow_tau = 0.08; // tau narrow movement
double N = 1;

/******************************************************/ 
/*  
 *  phase model
 *  Preconditions:
 *  Number of neuron should match number of entries in patterns array
 *  Number of neuron should match number of entries in a array, value should be decide depending on the desire pattern
 */
/******************************************************/ 
//struct phaseNeuron 
/******************************************************/ 
struct phaseNeuron { 
   char description[DESCRIPTION_LENGTH]; // name
   double tao = default_tau;
   double A = default_amp;
   double theta_j[NUMBER_PHASE_NEURONS];
   double bias = 0;
   double v = 0.8; //intrinsic frequency
   double a[NUMBER_PHASE_NEURONS]; // weights
   double C[NUMBER_PHASE_NEURONS]; // sign of bias
   double theta_i = 0; //output
} phase_neuron[NUMBER_PHASE_NEURONS]; 
/******************************************************/
 
//struct Pattern 
/******************************************************/ 
struct Pattern{
  String str;
  double a[NUMBER_PHASE_NEURONS];
  double C[NUMBER_PHASE_NEURONS];
  };
/******************************************************/ 
//Parameter Configuration: Use the following array to define the settings for each neuron.
//Each array entry corresponds to a single neuron's configuration.

// Ensure that the number of entries in the patterns array matches the number of neurons.
// Ensure that the number of entries in the 'a' array matches the number of neurons.
Pattern patterns[NUMBER_PHASE_NEURONS] = 
{
  /*Description            a                  C          */
  {"First neuron",   {0,w,0,0,0,0,0},  {0,-1,0,0,0,0,0} },
  {"Second neuron",  {w,0,0,0,0,0,0},  {1,0,0,0,0,0,0}  }, 
  {"Third neuron",   {0,w,0,0,0,0,0},  {0,1,0,0,0,0,0}  },  
  {"Fourth neuron",  {0,0,w,0,0,0,0},  {0,0,1,0,0,0,0}  },
  {"Fifth neuron",   {0,0,0,w,0,0,0},  {0,0,0,1,0,0,0}  }, 
  {"Sixth neuron",   {0,0,0,0,w,0,0},  {0,0,0,0,1,0,0}  }, 
  {"Seventh neuron", {0,0,0,0,0,w,0},  {0,0,0,0,0,1,0}  }, 
  
};

bool read_flex_sensor(int *sensor_readings_array, int array_length, int &sensor_readings_index, int sensor_input, int baseline_value, int sensitivity);
/******************************************************/
inline double bias(double N, double neuron_number = NUMBER_PHASE_NEURONS){
  double phase_bias;
  phase_bias = 2*M_PI*N/neuron_number;
  
  return phase_bias;
}

inline double der_theta (double theta_i, double theta_j[], double a[], double v , double tao, double bias, double C[])
{ 
  double sum = 0;
  for (int j = 0; j < NUMBER_PHASE_NEURONS; j++)
  {
    
    sum = sum + (a[j]* sin(theta_j[j]-theta_i-C[j]*bias)); //think how to get bias  bias(N,NUMBER_PHASE_NEURONS)
  }
  return (double)((2*M_PI*v + sum)/tao); 
}
/******************************************************/ 
inline double x_output (double theta, double A)
{return  (double)(A*cos(theta));} 
/******************************************************/ 
void update_phase_neuron(struct phaseNeuron* phase_n)
{
  int n = 20; 
  double theta_i[n]; 
  double h; 
  h = ((double)mydelay / 1000) / n;

  //Initial conditions
  theta_i[0] = phase_n->theta_i;

  for (int i = 0; i < n - 1; i++)
  {
      double der_theta_val = der_theta(theta_i[i], phase_n->theta_j, phase_n->a, phase_n->v, phase_n->tao, phase_n->bias, phase_n->C);
      theta_i[i+1]= theta_i[i] + h * der_theta_val;
   
  }

  phase_n->theta_i = theta_i[n-1];

  return;
}
/******************************************************/ 
void setup_phase_neuron(struct phaseNeuron *phase_n,struct Pattern myP, String str)
{
  for(int i=0 ; i<ARRAY_LENGTH(str) ; i++)
  {
    phase_n->description[i] = str[i]; 
  }

  //Initialize a array and y_j array
  for (int i = 0; i < NUMBER_PHASE_NEURONS; i++)
  {
    phase_n->theta_j[i] = phase_n->theta_i;
    phase_n->a[i] = myP.a[i];
    phase_n->C[i] = myP.C[i];
  }
}
/******************************************************/ 
void update_locomotion_network(void)
{
  double theta_j[NUMBER_PHASE_NEURONS];
  for (int i = 0; i< NUMBER_PHASE_NEURONS ; i++)
  {
    update_phase_neuron(&phase_neuron[i]);
    //Keep track of every y_i for each neuron and store the values into y_j.
    //Note: the y_j member of phase_neuron structure (phase_neuron->y_j) is an array that contains the output (y_i) 
    //for each neuron. The initial value for each y_i was set to 0. Therefore, at the beginning the array phase_neuron->y_j 
    //is filled with 0's. After each iteration we get a new y_i for each neuron that will serve as an input for the next iteration.
    //In order to keep track of this new values we store them in the local array y_j of this function. Once we finish to update 
    //every neuron we write these values to phase_neuron->y_j.
    theta_j[i] = phase_neuron[i].theta_i;
  }

  //As described above once we complete a whole iteration over all neurons we need to update y_j with the new values
  for (int j = 0; j< NUMBER_PHASE_NEURONS; j++)
  {

    for (int i = 0; i< NUMBER_PHASE_NEURONS; i++)
    {
        phase_neuron[j].theta_j[i] = theta_j[i];
    }
  }
}
/******************************************************/ 
/* put your setup code in setup(), to run once */
void setup()
{
  delay(2000);
  // put your setup code here, to run once:
  Serial.begin(115200);
  //Declare pins as analog input
  pinMode(1, INPUT);
  pinMode(2, INPUT);
  pinMode(3, INPUT);
  pinMode(4, INPUT);
  pinMode(5, INPUT);
  pinMode(6, INPUT);

  analogValueA1_Baseline = analogRead(1);
  analogValueA2_Baseline = analogRead(2);
  analogValueA3_Baseline = analogRead(3);
  analogValueA4_Baseline = analogRead(4);
  analogValueA5_Baseline = analogRead(5);
  analogValueA6_Baseline = analogRead(6);
  
  // Initialize portHandler. Set the port path
  // Get methods and members of PortHandlerLinux or PortHandlerWindows
  portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

  // Initialize packetHandler. Set the protocol version
  // Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
  packetHandler= dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  if (portHandler->openPort())
  {
    Serial.print("Succeeded to open the port!\n");
  }
  else
  {
    Serial.print("Failed to open the port!\n");
    Serial.print("Press any key to terminate...\n");
    return;
  }

  // Set port baudrate
  if (portHandler->setBaudRate(BAUDRATE))
  {
    Serial.print("Succeeded to change the baudrate!\n");
  }
  else
  {
    Serial.print("Failed to change the baudrate!\n");
    Serial.print("Press any key to terminate...\n");
    return;
  } 
  
  /* set the configuration of the  neuron to match a desired output: NO_ADAPTATION, TONIC, PHASIC */
  for (int i = 0; i < NUMBER_PHASE_NEURONS; i++)
  {
    setup_phase_neuron(&phase_neuron[i],patterns[i],patterns[i].str);
  }
}

/******************************************************/ 

/* put your main code here in loop(), to run repeatedly */

void loop() {
  A1_stable_reading = read_flex_sensor(analogValueA1, ARRAY_LENGTH(analogValueA1), A1_counter, analogRead(1), analogValueA1_Baseline, sensitivity_1);
  A2_stable_reading = read_flex_sensor(analogValueA2, ARRAY_LENGTH(analogValueA2), A2_counter, analogRead(2), analogValueA2_Baseline, sensitivity_1);
  A3_stable_reading = read_flex_sensor(analogValueA3, ARRAY_LENGTH(analogValueA3), A3_counter, analogRead(3), analogValueA3_Baseline, sensitivity_2);
  A4_stable_reading = read_flex_sensor(analogValueA4, ARRAY_LENGTH(analogValueA4), A4_counter, analogRead(4), analogValueA4_Baseline, sensitivity_2);
  A5_stable_reading = read_flex_sensor(analogValueA5, ARRAY_LENGTH(analogValueA5), A5_counter, analogRead(5), analogValueA5_Baseline, sensitivity_2);
  A6_stable_reading = read_flex_sensor(analogValueA6, ARRAY_LENGTH(analogValueA6), A6_counter, analogRead(6), analogValueA6_Baseline, sensitivity_2);

  myTime = millis();
 
  /* Update the neurons output*/
  update_locomotion_network();

  for(int i=0; i<NUMBER_PHASE_NEURONS; i++)
  {
    phase_neuron[i].bias = bias(N,NUMBER_PHASE_NEURONS);
    uint8_t motor_id = i+1;
    
    if (A1_stable_reading)
    {
      Serial.print("trajectory_offset: ");
      Serial.println(trajectory_offset);
      Serial.print("rigth_counter: ");
      Serial.println(rigth_counter);
      trajectory_offset = 522 + rigth_counter;
      current_time_rotation = myTime;
      if (rigth_counter < 50)
      {
        rigth_counter++;
      }
    }

    if (A2_stable_reading)
    {
      Serial.print("trajectory_offset: ");
      Serial.println(trajectory_offset);
      Serial.print("left_counter: ");
      Serial.println(left_counter);
      trajectory_offset = 502 - left_counter;
      current_time_rotation = myTime;
      if (left_counter < 50)
      {
        left_counter++;
      }
    }

    if (A5_stable_reading)
    {
      Serial.print("analogValueA5: ");
      Serial.println(analogRead(5));
      current_time_amplitude = myTime;
      amplitude_flag = true;
    }

    if (A6_stable_reading)
    {
      Serial.print("analogValueA6: ");
      Serial.println(analogRead(6));
      current_time_amplitude = myTime;
      amplitude_flag = true;
    }
            
    if (myTime >  current_time_rotation + delta_t_rotation)
    {
      //trajectory_flag = false;
      trajectory_offset = 511;
      phase_neuron[i].tao = default_tau;
      rigth_counter = 0;
      left_counter = 0;
    }

    if (myTime >  current_time_amplitude + delta_t_amplitude)
    {
      phase_neuron[i].A = default_amp;
      phase_neuron[i].tao = default_tau;
      amplitude_flag = false;
    }
    if (amplitude_flag)
    {
      phase_neuron[i].A = narrow_amp;
      phase_neuron[i].tao = narrow_tau;
      double old_bias=bias(N,NUMBER_PHASE_NEURONS);
    }
    
    goal_position[i] = x_output(phase_neuron[i].theta_i, phase_neuron[i].A) + trajectory_offset;

    if(myTime > 5000)
    {
      dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, motor_id, ADDR_AX_GOAL_POSITION, goal_position[i], &dxl_error);
    }
    delay(1);
    packetHandler->read2ByteTxRx(portHandler, motor_id, ADDR_AX_PRESENT_POSITION, (uint16_t*)&dxl_present_position, &dxl_error);
    /*
    Serial.print("ID : ");
    Serial.print(motor_id);
    Serial.print("\t Present Position : ");
    Serial.print(dxl_present_position);
    Serial.print("\t Goal Position : ");
    Serial.print(goal_position[i]);
    Serial.print("\n");
    */
  }

  /* delay at the end */
  delay(mydelay);
}

bool read_flex_sensor(int *sensor_readings_array, int array_length, int &sensor_readings_index, int sensor_input, int baseline_value, int sensitivity)
{
  bool stable_value = true;
  sensor_readings_array[sensor_readings_index] = sensor_input;
  for (int i=0; i < array_length; i++)
  {
    if((sensor_readings_array[i] > baseline_value + sensitivity) || (sensor_readings_array[i] < baseline_value - sensitivity))
    {
      stable_value = stable_value & true;
    }
    else
    {
      stable_value = false;
    }
  }

  if (sensor_readings_index == array_length - 1)
  {
    sensor_readings_index = 0;
  }
  else
  {
    sensor_readings_index++;
  }

  return stable_value;
}
