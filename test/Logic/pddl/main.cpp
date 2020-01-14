#include <Logic/fol.h>
#include <Logic/fol_mcts_world.h>

//===========================================================================

void testFastDownward(){
  rai::String file = "../player/pnp.g";
  if(rai::checkParameter<rai::String>("file")) file = rai::getParameter<rai::String>("file");
  if(rai::argc>1) file = rai::argv[1];

  FOL_World world(file);
  world.verbose = rai::getParameter<int>("verbose", 2);
  world.reset_state();

  rai::String plan = world.callPDDLsolver();

  world.addDecisionSequence(plan);
}

//===========================================================================

int main(int argn, char** argv){
  rai::initCmdLine(argn, argv);
  rnd.clockSeed();

  testFastDownward();
}
