#define DESCRIPTION_LENGTH     15
#define NUMBER_MATSUOKA_NEURONS     20
#define ARRAY_LENGTH(array) (sizeof(array) / sizeof((array)[0]))
unsigned long int myTime;
unsigned int mydelay = 10; // ms
unsigned int start_simulation = 1000; // ms
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
double cs = 1.5;
Pattern eight_coupling[NUMBER_MATSUOKA_NEURONS] = 
{
  /*Description  tao    b       T                                      a                                  x_i(0)*/
  {"Neuron-01",   1,   2.5,     12,     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, cs, cs, 0, 0, 0, 0, 0, 0, 0, 0},   0.1},
  {"Neuron-02",   1,   2.5,     12,     {cs, 0, cs, 0, 0, 0, 0, 0, 0, 0, cs, 0, cs, 0, 0, 0, 0, 0, 0, 0}, 0},
  {"Neuron-03",   1,   2.5,     12,     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, cs, cs, cs, 0, 0, 0, 0, 0, 0},  0},
  {"Neuron-04",   1,   2.5,     12,     {0, 0, cs, 0, cs, 0, 0, 0, 0, 0, 0, 0, cs, 0, cs, 0, 0, 0, 0, 0}, 0},
  {"Neuron-05",   1,   2.5,     12,     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, cs, cs, cs, 0, 0, 0, 0},  0},
  {"Neuron-06",   1,   2.5,     12,     {0, 0, 0, 0, cs, 0, cs, 0, 0, 0, 0, 0, 0, 0, cs, 0, cs, 0, 0, 0}, 0},
  {"Neuron-07",   1,   2.5,     12,     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, cs, cs, cs, 0, 0},  0},
  {"Neuron-08",   1,   2.5,     12,     {0, 0, 0, 0, 0, 0, cs, 0, cs, 0, 0, 0, 0, 0, 0, 0, cs, 0, cs, 0}, 0},
  {"Neuron-09",   1,   2.5,     12,     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, cs, cs, cs},  0},
  {"Neuron-10",   1,   2.5,     12,     {0, 0, 0, 0, 0, 0, 0, 0, cs, 0, 0, 0, 0, 0, 0, 0, 0, 0, cs, 0},   0},

  {"Neuron-11",   1,   2.5,     12,     {0, cs, 0, 0, 0, 0, 0, 0, 0, 0, 0, cs, 0, 0, 0, 0, 0, 0, 0, 0},  -0.1},
  {"Neuron-12",   1,   2.5,     12,     {cs, cs, cs, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},  0},
  {"Neuron-13",   1,   2.5,     12,     {0, cs, 0, cs, 0, 0, 0, 0, 0, 0, 0, cs, 0, cs, 0, 0, 0, 0, 0, 0}, 0},
  {"Neuron-14",   1,   2.5,     12,     {0, 0, cs, cs, cs, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},  0},
  {"Neuron-15",   1,   2.5,     12,     {0, 0, 0, cs, 0, cs, 0, 0, 0, 0, 0, 0, 0, cs, 0, cs, 0, 0, 0, 0}, 0},
  {"Neuron-16",   1,   2.5,     12,     {0, 0, 0, 0, cs, cs, cs, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},  0},
  {"Neuron-17",   1,   2.5,     12,     {0, 0, 0, 0, 0, cs, 0, cs, 0, 0, 0, 0, 0, 0, 0, cs, 0, cs, 0, 0}, 0},
  {"Neuron-18",   1,   2.5,     12,     {0, 0, 0, 0, 0, 0, cs, cs, cs, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},  0},
  {"Neuron-19",   1,   2.5,     12,     {0, 0, 0, 0, 0, 0, 0, cs, 0, cs, 0, 0, 0, 0, 0, 0, 0, cs, 0, cs}, 0},
  {"Neuron-20",   1,   2.5,     12,     {0, 0, 0, 0, 0, 0, 0, 0, cs, cs, 0, 0, 0, 0, cs, 0, cs, 0, 0, 0}, 0},
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
{return  (double)(max(0.0,x));} 
/******************************************************/ 
void update_matsuoka_neuron(struct matsuokaNeuron* mat_n)
{
  int n = 20; 
  double x_i[n],x_prime_i[n],y_i[n]; 
  double h = ((double)mydelay / 1000) / n;

  //Initial conditions
  x_i[0] = mat_n->x_i;
  x_prime_i[0] = mat_n->x_prime;
  y_i[0] = mat_n->y_i;

  for (int i = 0; i < n - 1; i++)
  {
      double der_x_val = der_x(x_i[i], x_prime_i[i], mat_n->a, mat_n->y_j, mat_n->s_i, mat_n->tao, mat_n->b);
      double der_x_prime_val = der_x_prime(x_prime_i[i], y_i[i], mat_n->T);

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
  Serial.begin(115200);

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

  /* Introduce a perturbation to the system to start the oscillation */
  // -> not necessary since initial condition of x_i(0) is sufficient to start oscillation

  /* Update the neurons output*/
  unsigned long startTime = millis();
  update_locomotion_network();
  unsigned long endTime = millis();
  unsigned long cycleTime = endTime - startTime;
  Serial.print("Cycle Time: ");
  Serial.println(cycleTime);

  /* Printing the output of the neurons on serial port*/
  for (int i = 0; i< NUMBER_MATSUOKA_NEURONS ; i++)
  {
    Serial.print(matsuoka_neuron[i].y_i);Serial.print(",");
  }
  
  Serial.print("\n");

  /* delay at the end */
  delay(mydelay);
}
