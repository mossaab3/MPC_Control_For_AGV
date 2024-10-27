
double**  cercleRef(int max_iter, double to, double freq, double R, double Xo, double Yo, double theta0);
double**  rightRef(int max_iter, double to, double a);
const int swarm_size = 80;
const int Np = 20;
const int Nu = 20;
const int dim_bird =2*Nu;
struct bird
{
    double position[dim_bird];
    double velocity[dim_bird];
    double cost;
    double best_local_position[dim_bird];
    double best_cost=1e20;
};
class swarm
{

public:
 
    bird Bird[swarm_size];
    double bestPos[dim_bird];
    double bestCost =1e20;
    swarm();
};
double* MPC(double**, double*, int, int, double*, double*, float ,int , double , double, swarm&);
