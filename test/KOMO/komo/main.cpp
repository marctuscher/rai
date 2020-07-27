#include <KOMO/komo.h>
#include <Kin/TM_default.h>
#include <Kin/F_PairCollision.h>

//===========================================================================

void TEST(Easy){
  rai::Configuration C("arm.g");
  cout <<"configuration space dim=" <<C.getJointStateDimension() <<endl;
  
  KOMO komo;
  komo.setModel(C);
  komo.setTiming(1., 100, 5., 2);
  komo.add_qControlObjective({}, 2, 1.);

  //-- set a time optim objective
//  komo.addObjective({}, make_shared<TM_Time>(), OT_sos, {1e2}, {}, 1); //smooth time evolution
//  komo.addObjective({}, make_shared<TM_Time>(), OT_sos, {1e1}, {komo.tau}, 0); //prior on timing

  komo.addObjective({1.}, FS_positionDiff, {"endeff", "target"}, OT_sos, {1e1});
  komo.addObjective({.98,1.}, FS_qItself, {}, OT_sos, {1e1}, {}, 1);
  komo.addObjective({}, FS_accumulatedCollisions, {}, OT_eq, {1.});

  komo.reportProblem();

  komo.reset();
//  komo.setSpline(5);
  komo.run();
  cout <<"TIME OPTIM: total=" <<sum(komo.getPath_times()) <<komo.getPath_times() <<endl;
  komo.plotTrajectory();
//  komo.reportProxies();
  komo.checkGradients();
  for(uint i=0;i<2;i++) komo.displayTrajectory();
}

//===========================================================================

void TEST(Align){
  rai::Configuration C("arm.g");
  cout <<"configuration space dim=" <<C.getJointStateDimension() <<endl;
  KOMO komo;
  komo.setModel(C);
  komo.setTiming(1., 100, 5., 2);
  komo.add_qControlObjective({}, 2, 1.);

  komo.addObjective({1.}, FS_positionDiff, {"endeff", "target"}, OT_eq, {1e1});
  komo.addObjective({1.}, FS_quaternionDiff, {"endeff", "target"}, OT_eq, {1e1});
  komo.addObjective({.98,1.}, FS_qItself, {}, OT_sos, {1e1}, {}, 1);
  komo.addObjective({}, FS_accumulatedCollisions, {}, OT_eq, {1.});

  komo.reset();
  komo.run();
  komo.plotTrajectory();
  for(uint i=0;i<2;i++) komo.displayTrajectory();
}

//===========================================================================

struct MyFeature : Feature {
  int i, j;               ///< which shapes does it refer to?

  MyFeature(int _i, int _j)
    : i(_i), j(_j) {}
  MyFeature(const rai::Configuration& K, const char* s1, const char* s2)
    :  i(initIdArg(K, s1)), j(initIdArg(K, s2)) {}

  virtual void phi(arr& y, arr& J, const ConfigurationL& Ctuple){
    CHECK_EQ(order, 1, "");

    auto V = TM_Default(TMT_posDiff, i, NoVector, j).setOrder(1).eval(Ctuple);

    auto C = F_PairCollision(i, j, F_PairCollision::_normal, false).eval(Ctuple);

    auto D = F_PairCollision(i, j, F_PairCollision::_negScalar, false).eval(Ctuple);

    //penalizing velocity whenever close
    double range=.2;
    if(-D.y.scalar() > range){
      y = zeros(3);
      if(!!J) J = zeros(3, V.J.d1);
      return;
    }

    double weight = 1. + D.y.scalar()/range;
    double normalWeight = 1.;

    arr P = eye(3) + normalWeight*(C.y*~C.y);
    y = weight * P * V.y;
    if(!!J){
      J = weight * P * V.J;
      J += P * V.y.reshape(3,1) * (1./range)*D.J;
      J += (weight * 2. * normalWeight * scalarProduct(C.y,V.y)) * C.J;
    }

#if 0
    //penalizing normal velocity
    double normalVel = scalarProduct(V.y, C.y);
    if(normalVel>0.){
      y = 0.;
      if(!!J) J = zeros(1, V.J.d1);
      return;
    }


    double scale = 3.;
    double weight = ::exp(scale * D.y.scalar());
    weight = 1.; scale=0.;

    y.resize(1);
    y(0) = weight * normalVel;
    if(!!J){
      J = weight * ( ~V.y * C.J + ~C.y * V.J );
      J += (normalVel * weight * scale) * D.J;
    }

#if 0
    normalVel += 1.;

    y = D.y / normalVel;

    if(!!J){
      J = D.J / normalVel;
      J += (-D.y.scalar() / (normalVel*normalVel)) * ( ~V.y * C.J + ~C.y * V.J );
    }
#endif
#endif

  }

  virtual uint dim_phi(const ConfigurationL& Ctuple) {
    return 3;
  }

};

void TEST(Thin){
  rai::Configuration C("thin.g");
  cout <<"configuration space dim=" <<C.getJointStateDimension() <<endl;

  KOMO komo;
  komo.setModel(C);
  komo.setTiming(1., 60, 5., 2);
  komo.add_qControlObjective({}, 2, 1.);

  //-- set a time optim objective
//  komo.addObjective({}, make_shared<TM_Time>(), OT_sos, {1e2}, {}, 1); //smooth time evolution
//  komo.addObjective({}, make_shared<TM_Time>(), OT_sos, {1e1}, {komo.tau}, 0); //prior on timing

  komo.addObjective({1.}, FS_positionDiff, {"ball", "target"}, OT_eq, {1e1});
  komo.addObjective({1.}, FS_qItself, {}, OT_eq, {1e1}, {}, 1);
  komo.addObjective({}, FS_distance, {"wall", "ball"}, OT_ineq, {1.});

  komo.addObjective({}, make_shared<MyFeature>(komo.world, "ball", "wall"), OT_sos, {1e1}, {}, 1);

  komo.reportProblem();

  komo.animateOptimization=1;
  komo.reset(1e-2);
//  komo.setSpline(5);
  komo.run();
  komo.plotTrajectory();
//  komo.reportProxies();
  komo.checkGradients();

  while(komo.displayTrajectory());
}

//===========================================================================

void TEST(PR2){
  rai::Configuration C("model.g");
  C.optimizeTree(true);
  cout <<"configuration space dim=" <<C.getJointStateDimension() <<endl;
  double rand = rai::getParameter<double>("KOMO/moveTo/randomizeInitialPose", .0);
  if(rand){
    rnd.seed(rai::getParameter<uint>("rndSeed", 0));
    rndGauss(C.q,rand,true);
    C.setJointState(C.q);
  }

  KOMO komo;
//  komo.logFile = new ofstream("z.dat");
//  komo.denseOptimization=true;
//  komo.sparseOptimization=true;
  komo.setModel(C);
  komo.setTiming(1., 100, 10., 2);
  komo.add_qControlObjective({}, 2, 1.);
  komo.addObjective({1.}, FS_positionDiff, {"endeff", "target"}, OT_eq, {1e1});
  komo.addObjective({.98,1.}, FS_qItself, {}, OT_sos, {1e1}, {}, 1);
  komo.addObjective({}, FS_accumulatedCollisions, {}, OT_eq, {1.});

  komo.reset();
//  komo.setSpline(10);
  komo.run();
  komo.plotTrajectory();
//  komo.checkGradients();
  for(uint i=0;i<2;i++) komo.displayTrajectory();
}

//===========================================================================

// void TEST(FinalPosePR2){
//   rai::Configuration K("model.g");
//   K.pruneRigidJoints();
//   K.optimizeTree();
//   makeConvexHulls(K.frames);
//   cout <<"configuration space dim=" <<K.getJointStateDimension() <<endl;
//   arr x = finalPoseTo(K, *K.getFrameByName("endeff"), *K.getFrameByName("target"));
//   K.setJointState(x.reshape(x.N));
//   K.watch(true);
// }

//===========================================================================

int main(int argc,char** argv){
  rai::initCmdLine(argc,argv);

  rnd.clockSeed();

//  testEasy();
//  testAlign();
  testThin();
//  testPR2();

  return 0;
}

