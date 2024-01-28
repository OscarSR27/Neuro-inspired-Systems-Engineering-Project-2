#include <actuator.h>
#include <Dynamixel2Arduino.h>
#include <DynamixelSDK.h>
#include <Arduino.h>

#define NUMBER_MOTORS  7
#define DESCRIPTION_LENGTH     15
#define NUMBER_MATSUOKA_NEURONS     14
#define ARRAY_LENGTH(array) (sizeof(array) / sizeof((array)[0]))
unsigned long int myTime;
unsigned int mydelay = 10; // ms

#define ADDR_AX_ID           3                 
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
uint16_t goal_position[NUMBER_MOTORS];
int dxl_comm_result = COMM_TX_FAIL;             // Communication result
uint8_t dxl_error = 0;                          // Dynamixel error
int16_t dxl_present_position = 0;               // Present position

/******************************************************/ 
/*  
 *  Matsuoka model
 *  Preconditions:
 *  Number of neuron should match number of entries in patterns array
 *  Number of neuron should match number of entries in a array, value should be decide depending on the desire pattern
 */
/******************************************************/ 
//struct matsuokaNeuron 
/******************************************************/ 
struct matsuokaNeuron { 
   char description[DESCRIPTION_LENGTH]; // name
   float tao = 0;
   float b = 0;
   float T = 0;
   float x_i = 0;
   float a[NUMBER_MATSUOKA_NEURONS];
   float y_j[NUMBER_MATSUOKA_NEURONS];  
   float s_i = 1; //External stimulus/current, must be positive and constant
   float x_prime = 0; 
   float y_i = 0; 
} matsuoka_neuron[NUMBER_MATSUOKA_NEURONS]; 
/******************************************************/ 
//struct Pattern 
/******************************************************/ 
struct Pattern{
  String str;
  float tao;
  float b;
  float T;
  float a[NUMBER_MATSUOKA_NEURONS];
  float x_i;
  };
/******************************************************/ 
//Parameter Configuration: Use the following array to define the settings for each neuron.
//Each array entry corresponds to a single neuron's configuration.

// Ensure that the number of entries in the patterns array matches the number of neurons.
// Ensure that the number of entries in the 'a' array matches the number of neurons.
float cs = 1.5;
Pattern ladder_circle[NUMBER_MATSUOKA_NEURONS] = 
{
  /*Description  tao    b       T                                      a                                  x_i(0)*/
  {"Neuron-01",   1,   2.5,     12,     {0, 0, 0, 0, 0, 0, 0, cs, 0, 0, 0, 0, 0, 0}, 0},
  {"Neuron-02",   1,   2.5,     12,     {cs, 0, 0, 0, 0, 0, 0, 0, cs, 0, 0, 0, 0, 0}, 0},
  {"Neuron-03",   1,   2.5,     12,     {0, cs, 0, 0, 0, 0, 0, 0, 0, cs, 0, 0, 0, 0}, 0},
  {"Neuron-04",   1,   2.5,     12,     {0, 0, cs, 0, 0, 0, 0, 0, 0, 0, cs, 0, 0, 0}, 0},
  {"Neuron-05",   1,   2.5,     12,     {0, 0, 0, cs, 0, 0, 0, 0, 0, 0, 0, cs, 0, 0}, 0},
  {"Neuron-06",   1,   2.5,     12,     {0, 0, 0, 0, cs, 0, 0, 0, 0, 0, 0, 0, cs, 0}, 0},
  {"Neuron-07",   1,   2.5,     12,     {0, 0, 0, 0, 0, cs, 0, 0, 0, 0, 0, 0, 0, cs}, 0},
  {"Neuron-08",   1,   2.5,     12,     {cs, 0, 0, 0, 0, 0, 0, 0, cs, 0, 0, 0, 0, 0}, 0},
  {"Neuron-09",   1,   2.5,     12,     {0, cs, 0, 0, 0, 0, 0, 0, 0, cs, 0, 0, 0, 0}, 0},
  {"Neuron-10",   1,   2.5,     12,     {0, 0, cs, 0, 0, 0, 0, 0, 0, 0, cs, 0, 0, 0}, 0},
  {"Neuron-11",   1,   2.5,     12,     {0, 0, 0, cs, 0, 0, 0, 0, 0, 0, 0, cs, 0, 0}, 0},
  {"Neuron-12",   1,   2.5,     12,     {0, 0, 0, 0, cs, 0, 0, 0, 0, 0, 0, 0, cs, 0}, 0},
  {"Neuron-13",   1,   2.5,     12,     {0, 0, 0, 0, 0, cs, 0, 0, 0, 0, 0, 0, 0, cs}, 0},
  {"Neuron-14",   1,   2.5,     12,     {0, 0, 0, 0, 0, 0, cs, 0, 0, 0, 0, 0, 0, 0}, 0},
};

Pattern eight_mono_inhibition[NUMBER_MATSUOKA_NEURONS] = 
{
  /*Description  tao    b       T                                      a                                  x_i(0)*/
  {"Neuron-01",   1,   2.5,     12,     {0, 0, 0, 0, 0, 0, cs, 0, 0, 0, 0, 0, 0, 0}, 0.1},
  {"Neuron-02",   1,   2.5,     12,     {cs, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, 0},
  {"Neuron-03",   1,   2.5,     12,     {0, cs, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, 0},
  {"Neuron-04",   1,   2.5,     12,     {0, 0, cs, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, 0},
  {"Neuron-05",   1,   2.5,     12,     {0, 0, 0, cs, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, 0},
  {"Neuron-06",   1,   2.5,     12,     {0, 0, 0, 0, cs, 0, 0, 0, 0, 0, 0, 0, 0, 0}, 0},
  {"Neuron-07",   1,   2.5,     12,     {0, 0, 0, 0, 0, cs, 0, 0, 0, 0, 0, 0, 0, 0}, 0},
  {"Neuron-08",   1,   2.5,     12,     {cs, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, 0},
  {"Neuron-09",   1,   2.5,     12,     {0, cs, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, 0},
  {"Neuron-10",   1,   2.5,     12,     {0, 0, cs, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, 0},
  {"Neuron-11",   1,   2.5,     12,     {0, 0, 0, cs, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, 0},
  {"Neuron-12",   1,   2.5,     12,     {0, 0, 0, 0, cs, 0, 0, 0, 0, 0, 0, 0, 0, 0}, 0},
  {"Neuron-13",   1,   2.5,     12,     {0, 0, 0, 0, 0, cs, 0, 0, 0, 0, 0, 0, 0, 0}, 0},
  {"Neuron-14",   1,   2.5,     12,     {0, 0, 0, 0, 0, 0, cs, 0, 0, 0, 0, 0, 0, 0}, 0},
};

/******************************************************/ 
inline float der_x (float x, float x_prime, float a[], float y_j[], float s_i , float tao , float b)
{ 
  float sum = 0;
  for (int j = 0; j < NUMBER_MATSUOKA_NEURONS; j++)
  {
    sum = sum + (a[j]*y_j[j]);
  }
  return (float)((-sum + s_i - (b*x_prime) - x)/tao); 
}
/******************************************************/ 
inline float der_x_prime (float x_prime, float yi , float T)
{return (float)((yi - x_prime)/T);} 
/******************************************************/ 
inline float y_output (float x)
{return  (float)(std::max((float)0.0,x));} 
/******************************************************/ 
void update_matsuoka_neuron(struct matsuokaNeuron* mat_n)
{
  uint8_t n = 20; 
  float x_i[n],x_prime_i[n],y_i[n]; 
  float h = ((float)mydelay / 1000) / n;

  //Initial conditions
  x_i[0] = mat_n->x_i;
  x_prime_i[0] = mat_n->x_prime;
  y_i[0] = mat_n->y_i;

  for (int i = 0; i < n - 1; i++)
  {
      float der_x_val = der_x(x_i[i], x_prime_i[i], mat_n->a, mat_n->y_j, mat_n->s_i, mat_n->tao, mat_n->b);
      float der_x_prime_val = der_x_prime(x_prime_i[i], y_i[i], mat_n->T);

      x_i[i+1]= x_i[i] + h * der_x_val;
      x_prime_i[i+1]= x_prime_i[i]+ h * der_x_prime_val;
      y_i[i+1] = y_output(x_i[i]);
  }

  mat_n->x_i = x_i[n-1];
  mat_n->x_prime = x_prime_i[n-1];
  mat_n->y_i = y_i[n-1];

    return;
}
/******************************************************/ 
void setup_matsuoka_neuron(struct matsuokaNeuron *mat_n,struct Pattern myP, String str)
{
  for(int i=0 ; i<ARRAY_LENGTH(str) ; i++)
  {
    mat_n->description[i] = str[i]; 
  }
  
  mat_n->tao = myP.tao;
  mat_n->b = myP.b;
  mat_n->T = myP.T;
  mat_n->x_i = myP.x_i;

  //Initialize a array and y_j array
  for (uint8_t i = 0; i < NUMBER_MATSUOKA_NEURONS; i++)
  {
    mat_n->y_j[i] = mat_n->y_i;
    mat_n->a[i] = myP.a[i];
  }
}
/******************************************************/ 
void update_locomotion_network(void)
{
  float y_j[NUMBER_MATSUOKA_NEURONS];
  for (int i = 0; i< NUMBER_MATSUOKA_NEURONS ; i++)
  {
    update_matsuoka_neuron(&matsuoka_neuron[i]);
    //Keep track of every y_i for each neuron and store the values into y_j.
    //Note: the y_j member of matsuoka_neuron structure (matsuoka_neuron->y_j) is an array that contains the output (y_i) 
    //for each neuron. The initial value for each y_i was set to 0. Therefore, at the beginning the array matsuoka_neuron->y_j 
    //is filled with 0's. After each iteration we get a new y_i for each neuron that will serve as an input for the next iteration.
    //In order to keep track of this new values we store them in the local array y_j of this function. Once we finish to update 
    //every neuron we write these values to matsuoka_neuron->y_j.
    y_j[i] = matsuoka_neuron[i].y_i;
  }

  //As described above once we complete a whole iteration over all neurons we need to update y_j with the new values
  for (int j = 0; j< NUMBER_MATSUOKA_NEURONS; j++)
  {
    for (int i = 0; i< NUMBER_MATSUOKA_NEURONS; i++)
    {
        matsuoka_neuron[j].y_j[i] = y_j[i];
    }
  }
}
/******************************************************/ 
/* put your setup code in setup(), to run once */
void setup()
{
  // put your setup code here, to run once:
  Serial.begin(115200);
  while(!Serial.available());
  
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

  /* set the configuration of the MATSUOKA neuron to match a desired output: NO_ADAPTATION, TONIC, PHASIC */
  for (int i = 0; i < NUMBER_MATSUOKA_NEURONS; i++)
  {
    setup_matsuoka_neuron(&matsuoka_neuron[i],eight_mono_inhibition[i],eight_mono_inhibition[i].str);
  }
}

/******************************************************/ 


/* put your main code here in loop(), to run repeatedly */
void loop() {

  /* Read my program running time in milliseconds */
  myTime = millis();

  /* Update the neurons output*/
  update_locomotion_network();

  for(int i=0; i<NUMBER_MOTORS; i++)
  {
    uint8_t motor_id = i + 1;
    goal_position[i] = 1000*(matsuoka_neuron[i].y_i - matsuoka_neuron[i+NUMBER_MOTORS].y_i) + 500;
    //Serial.println(osc_pattern);

    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, motor_id, ADDR_AX_GOAL_POSITION, goal_position[i], &dxl_error);delay(1);
  
    packetHandler->read2ByteTxRx(portHandler, motor_id, ADDR_AX_PRESENT_POSITION, (uint16_t*)&dxl_present_position, &dxl_error);

    Serial.print("ID : ");
    Serial.print(motor_id);
    Serial.print("\t Present Position : ");
    Serial.print(dxl_present_position);
    Serial.print("\t Goal Position : ");
    Serial.print(1000*(matsuoka_neuron[i].y_i - matsuoka_neuron[i+NUMBER_MOTORS].y_i));
    Serial.print("\n");
  }

  /* delay at the end */
  delay(mydelay);
}
