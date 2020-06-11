#include "multirotor_sim/simulator.h"
// #include "logger.h"
using namespace multirotor_sim;

int main(int argc, char *argv[])
{
    Simulator sim;
    sim.load("../params/sim_params.yaml");
    // Logger log("/tmp/VINS_MONO_ex.bin");

    while (sim.run())
    {
        // log.log(sim.t_);
        // log.logVectors(sim.state().arr); //Add estimator states
    }
    return 0;
}