#define _USE_MATH_DEFINES
#include <math.h>
#define DESCRIPTION_LENGTH     15
#define NUMBER_PHASE_NEURONS     7
#define ARRAY_LENGTH(array) (sizeof(array) / sizeof((array)[0]))
unsigned long int myTime;
unsigned int mydelay = 10; // ms
unsigned int start_simulation = 2000; // ms
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
   double tao = 0;
   double A = 0.1; //0.2 or 0.1
   double theta_j[NUMBER_PHASE_NEURONS];
   double x_j[NUMBER_PHASE_NEURONS];
   double phase_bias = 30;
   double v = 1; //intrinsic frequency
   double a[NUMBER_PHASE_NEURONS];
   double theta_i = 0; //output
} phase_neuron[NUMBER_PHASE_NEURONS]; 
/******************************************************/ 
//struct Pattern 
/******************************************************/ 
struct Pattern{
  String str;
  double tao;
  double A;
  double a[NUMBER_PHASE_NEURONS];
  };
/******************************************************/ 
//Parameter Configuration: Use the following array to define the settings for each neuron.
//Each array entry corresponds to a single neuron's configuration.

// Ensure that the number of entries in the patterns array matches the number of neurons.
// Ensure that the number of entries in the 'a' array matches the number of neurons.
Pattern patterns[NUMBER_PHASE_NEURONS] = 
{
  /*Description    tao   A                a    */
  {"First neuron",  1,   1,            {0,1,0,0,0,0,0} }, 
  {"Second neuron", 1,   1,            {1,0,1,0,0,0,0} }, 
  {"Third neuron",  1,   1,            {0,0,0,1,0,0,0} },  
  {"Fourth neuron",  1,   1,           {0,0,0,0,1,0,0} },
  {"Fifth neuron",  1,   1,            {0,0,0,0,0,1,0} }, 
  {"Sixth neuron",  1,   1,            {0,0,0,0,0,0,1} }, 
  {"Seventh neuron",  1,   1,          {0,0,0,0,0,0,0} }, 
  
};

/******************************************************/ 
inline double bias(double N, double neuron_number = NUMBER_PHASE_NEURONS ){
  double phase_bias;
  phase_bias = 2*M_PI*N/neuron_number;
  return phase_bias;
}
inline double der_theta (double theta_i, double theta_j[], double a[], double v , double tao)
{ 
  double sum = 0;
  for (int j = 0; j < NUMBER_PHASE_NEURONS; j++)
  {
    //for(int i = 0; i < NUMBER_PHASE_NEURONS; i++){
       //sum = sum + (a[j]*y_j[j]);
    sum = sum + (a[j]* sin(theta_j[j]-theta_i- bias(N,NUMBER_PHASE_NEURONS))); //think how to get bias

    //}
    
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

  for (int i = 0; i < n; i++)
  {
      double der_theta_val = der_theta(theta_i[i], phase_n->theta_j, phase_n->a, phase_n->v, phase_n->tao);
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
  
  phase_n->tao = myP.tao;
  phase_n->A = myP.A;
  

  //Initialize a array and y_j array
  for (int i = 0; i < NUMBER_PHASE_NEURONS; i++)
  {
    phase_n->theta_j[i] = phase_n->theta_i;
    phase_n->a[i] = myP.a[i];
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
  Serial.begin(115200);

  /* set the configuration of the  neuron to match a desired output: NO_ADAPTATION, TONIC, PHASIC */
  for (int i = 0; i < NUMBER_PHASE_NEURONS; i++)
  {
    setup_phase_neuron(&phase_neuron[i],patterns[i],patterns[i].str);
  }
}

/******************************************************/ 


/* put your main code here in loop(), to run repeatedly */
void loop() {

  /* Read my program running time in milliseconds */
  myTime = millis();

  //for (int i = 0; i< NUMBER_PHASE_NEURONS ; i++)
  //{
  //  if (myTime > start_simulation)
  //  {
  //    phase_neuron[i].s_i = 1;
  //  }
  //}

  /* Update the neurons output*/
  update_locomotion_network();
  
  /* Printing the output of the neurons on serial port*/
  for (int i = 0; i< NUMBER_PHASE_NEURONS ; i++)
  {
    Serial.print(x_output(phase_neuron[i].theta_i, phase_neuron[i].A));Serial.print(",");
    //Serial.print(x_output(phase_neuron[i]));Serial.print(",");
  }
  
  Serial.print("\n");

  /* delay at the end */
  delay(mydelay);
}
