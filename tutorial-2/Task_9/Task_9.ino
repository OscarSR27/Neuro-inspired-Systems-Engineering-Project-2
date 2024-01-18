#include <actuator.h>
#include <Dynamixel2Arduino.h>
#include <DynamixelSDK.h>
#include <Arduino.h>

#define DESCRIPTION_LENGTH     15
#define NUMBER_MATSUOKA_NEURONS     2
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


// Default setting
#define DXL_ID                          1                   // Dynamixel ID: 1
#define DXL_NEW_ID                      1                   // Dynamixel ID: 2


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
int goalPosition = 0;
int isMoving = 0;
int dxl_comm_result = COMM_TX_FAIL;             // Communication result
uint8_t dxl_error = 0;                          // Dynamixel error
int16_t dxl_present_position = 0;               // Present position

uint8_t dxl_new_id = DXL_NEW_ID; 
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
   double tao = 0;
   double b = 0;
   double T = 0;
   double x_i = 0;
   double a[NUMBER_MATSUOKA_NEURONS];
   double y_j[NUMBER_MATSUOKA_NEURONS];  
   double s_i = 1; //External stimulus/current, must be positive and constant
   double x_prime = 0; 
   double y_i = 0; 
} matsuoka_neuron[NUMBER_MATSUOKA_NEURONS]; 
/******************************************************/ 
//struct Pattern 
/******************************************************/ 
struct Pattern{
  String str;
  double tao;
  double b;
  double T;
  double a[NUMBER_MATSUOKA_NEURONS];
  double x_i;
  };
/******************************************************/ 
//Parameter Configuration: Use the following array to define the settings for each neuron.
//Each array entry corresponds to a single neuron's configuration.

// Ensure that the number of entries in the patterns array matches the number of neurons.
// Ensure that the number of entries in the 'a' array matches the number of neurons.
double cs = 1.3;
Pattern eight_coupling[NUMBER_MATSUOKA_NEURONS] = 
{
  /*Description  tao    b       T                                      a                                  x_i(0)*/
  {"Neuron-01",   1,   3.5,     12,     {0, cs},   0.1},
  {"Neuron-02",   1,   3.5,     12,     {cs, 0}, 0},
};

/******************************************************/ 
inline double der_x (double x, double x_prime, double a[], double y_j[], double s_i , double tao , double b)
{ 
  double sum = 0;
  for (int j = 0; j < NUMBER_MATSUOKA_NEURONS; j++)
  {
    sum = sum + (a[j]*y_j[j]);
  }
  return (double)((-sum + s_i - (b*x_prime) - x)/tao); 
}
/******************************************************/ 
inline double der_x_prime (double x_prime, double yi , double T)
{return (double)((yi - x_prime)/T);} 
/******************************************************/ 
inline double y_output (double x)
{return  (double)(std::max(0.0,x));} 
/******************************************************/ 
void update_matsuoka_neuron(struct matsuokaNeuron* mat_n)
{
  int n = 20; 

  double h; 

  h= ((double)mydelay/1000)/n;
  double x_i[n],x_prime_i[n],y_i[n]; 
  double k1,k2,k3,k4,k, l1,l2,l3,l4,l;  
  //Initial conditions
  x_i[0] = mat_n->x_i;
  x_prime_i[0] = mat_n->x_prime;
  y_i[0] = mat_n->y_i;

  for (int i=0; i<n-1 ; i++)
  {
    k1= h*der_x( x_i[i], x_prime_i[i], mat_n->a , mat_n->y_j , mat_n->s_i , mat_n->tao, mat_n->b); 
    l1 = h*der_x_prime( x_prime_i[i] , y_i[i] , mat_n->T ); 

    k2= h*der_x( x_i[i]+k1/2, x_prime_i[i]+l1/2, mat_n->a , mat_n->y_j , mat_n->s_i , mat_n->tao, mat_n->b); 
    l2 = h*der_x_prime( x_prime_i[i]+l1/2 , y_i[i] , mat_n->T ); 

    k3= h*der_x( x_i[i]+k2/2, x_prime_i[i]+l2/2, mat_n->a , mat_n->y_j , mat_n->s_i , mat_n->tao, mat_n->b); 
    l3 = h*der_x_prime( x_prime_i[i]+l2/2 , y_i[i] , mat_n->T ); 

    k4= h*der_x( x_i[i]+k3/2, x_prime_i[i]+l3/2, mat_n->a , mat_n->y_j , mat_n->s_i , mat_n->tao, mat_n->b); 
    l4 = h*der_x_prime( x_prime_i[i]+l3/2 , y_i[i] , mat_n->T );  
    
    k = 1/6.0 * (k1 + 2*k2 + 2*k3 + k4);
    l  = 1/6.0 * ( l1 + 2*l2  + 2*l3  +  l4);
  
    x_i[i+1]= x_i[i]+k;
    x_prime_i[i+1]= x_prime_i[i]+l;
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
  for (int i = 0; i < NUMBER_MATSUOKA_NEURONS; i++)
  {
    mat_n->y_j[i] = mat_n->y_i;
    mat_n->a[i] = myP.a[i];
  }
}
/******************************************************/ 
void update_locomotion_network(void)
{
  double y_j[NUMBER_MATSUOKA_NEURONS];
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
  Serial.println("Please enter a new ID");
  String a;
  if(Serial.available())
  {
    a= Serial.readString();// read the incoming data as string
    Serial.println(a); 
    dxl_new_id = a.toInt();
    delay(2000);
  }

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

  // Change ID
  for(int id=0;id<=253;id++)
  {
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, id, ADDR_AX_ID, dxl_new_id, &dxl_error);delay(1);
  dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, id, 14, 1023, &dxl_error);delay(1);//Max Torque = 1023, Half Torque = 511
  dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, id, 32, 511, &dxl_error);delay(1);//Max Speed = 1023, Half Speed = 511
  delay(10);
  }
 

  /* set the configuration of the MATSUOKA neuron to match a desired output: NO_ADAPTATION, TONIC, PHASIC */
  for (int i = 0; i < NUMBER_MATSUOKA_NEURONS; i++)
  {
    setup_matsuoka_neuron(&matsuoka_neuron[i],eight_coupling[i],eight_coupling[i].str);
  }
}

/******************************************************/ 


/* put your main code here in loop(), to run repeatedly */
void loop() {

  /* Read my program running time in milliseconds */
  myTime = millis();

  /* Update the neurons output*/
  update_locomotion_network();

  int osc_pattern = 1000*(matsuoka_neuron[0].y_i - matsuoka_neuron[1].y_i) + 500;
  //Serial.println(osc_pattern);

    // put your main code here, to run repeatedly:
  dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, dxl_new_id, 30, osc_pattern, &dxl_error);delay(1);
  packetHandler->read1ByteTxRx(portHandler, dxl_new_id, MOVING, (uint8_t*)&isMoving, &dxl_error);

  if( isMoving == 0 ){ //if Dynamixel is stopped
    //Send instruction packet to move for goalPosition
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, dxl_new_id, ADDR_AX_GOAL_POSITION, goalPosition, &dxl_error);
    //toggle the position if goalPosition is 1000, set to 0, if 0, set to 1000
    if(goalPosition == 700)
      goalPosition = 0;
    else
      goalPosition = 700;
  }
  
  packetHandler->read2ByteTxRx(portHandler, dxl_new_id, ADDR_AX_PRESENT_POSITION, (uint16_t*)&dxl_present_position, &dxl_error);

  
  //Serial.print("ID : ");
  //Serial.print(dxl_new_id);
  //Serial.print("\t Present Position : ");
  Serial.print(dxl_present_position);
  Serial.print("\n");

  /* delay at the end */
  delay(mydelay);
}
