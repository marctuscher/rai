/*  ------------------------------------------------------------------
    Copyright (c) 2019 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "switch.h"
#include "kin.h"
#include <climits>
#include "contact.h"

//===========================================================================

/* x_{-1} = x_{time=0}
 * x_{9}: phase=1 (for stepsPerPhase=10 */
int conv_time2step(double time, uint stepsPerPhase) {
  return (floor(time*double(stepsPerPhase) + .500001))-1;
}
double conv_step2time(int step, uint stepsPerPhase) {
  return double(step+1)/double(stepsPerPhase);
}
//#define STEP(t) (floor(t*double(stepsPerPhase) + .500001))-1

//===========================================================================

template<> const char* rai::Enum<rai::SwitchType>::names []= {
  "noJointLink",
  "joint",
  "makeDynamic",
  "makeKinematic",
  "delContact",
  "addContact",
  nullptr
};

template<> const char* rai::Enum<rai::SwitchInitializationType>::names []= {
  "zero",
  "copy",
  "random",
  nullptr
};

//===========================================================================
//
// Kinematic Switch
//

rai::KinematicSwitch::KinematicSwitch()
  : symbol(SW_none), jointType(JT_none), init(SWInit_zero), timeOfApplication(-1), fromId(-1), toId(-1), jA(0), jB(0)
{}

rai::KinematicSwitch::KinematicSwitch(SwitchType _symbol, JointType _jointType, int aFrame, int bFrame, SwitchInitializationType _init, int _timeOfApplication, const rai::Transformation& jFrom, const rai::Transformation& jTo)
  : symbol(_symbol),
    jointType(_jointType),
    init(_init),
    timeOfApplication(_timeOfApplication),
    fromId(aFrame), toId(bFrame),
    jA(0), jB(0) {
  if(!!jFrom) jA = jFrom;
  if(!!jTo)   jB = jTo;
}

rai::KinematicSwitch::KinematicSwitch(rai::SwitchType op, rai::JointType type, const char* ref1, const char* ref2, const rai::Configuration& K, rai::SwitchInitializationType _init, int _timeOfApplication, const rai::Transformation& jFrom, const rai::Transformation& jTo)
  : KinematicSwitch(op, type, initIdArg(K, ref1), initIdArg(K, ref2), _init, _timeOfApplication, jFrom, jTo)
{}

void rai::KinematicSwitch::setTimeOfApplication(double time, bool before, int stepsPerPhase, uint T) {
  if(stepsPerPhase<0) stepsPerPhase=T;
  timeOfApplication = (time<0.?0:conv_time2step(time, stepsPerPhase))+(before?0:1);
}

void rai::KinematicSwitch::apply(Configuration& K) {
  Frame* from=nullptr, *to=nullptr;
  if(fromId!=-1) from=K.frames(fromId);
  if(toId!=-1) to=K.frames(toId);

  if(symbol==SW_joint || symbol==SW_joint) {
    rai::Transformation orgX = to->ensure_X();

    //first find link frame above 'to', and make it a root
#if 0 //THIS is the standard version that worked with pnp LGP tests - but is a problem for the crawler
    rai::Frame* link = to->getUpwardLink(NoTransformation, false);
    if(link->parent) link->unLink();
#else //THIS is the version that works for the crawler; I guess the major difference is 'upward until part break' and 'flip frames'
    K.reconfigureRoot(to, true);
#endif

    //create a new joint
    to->linkFrom(from, false);
    Joint* j = new Joint(*to);
    j->setType(jointType);

    if(!jA.isZero()) j->frame->insertPreLink(jA);
    if(!jB.isZero()) { HALT("only to be careful: does the orgX still work?"); j->frame->insertPostLink(jB); }

    //initialize to zero, copy, or random
    if(init==SWInit_zero) { //initialize the joint with zero transform
      j->frame->Q.setZero();
    } else if(init==SWInit_copy) { //set Q to the current relative transform, modulo DOFs
      j->frame->Q = orgX / j->frame->parent->ensure_X(); //that's important for the initialization of x during the very first komo.setupConfigurations !!
      //cout <<j->frame->Q <<' ' <<j->frame->Q.rot.normalization() <<endl;
      arr q = j->calc_q_from_Q(j->frame->Q);
      j->frame->Q.setZero();
      j->calc_Q_from_q(q, 0);
    } if(init==SWInit_random) { //random, modulo DOFs
      j->frame->Q.setRandom();
      arr q = j->calc_q_from_Q(j->frame->Q);
      j->frame->Q.setZero();
      j->calc_Q_from_q(q, 0);
    }
    j->frame->_state_updateAfterTouchingQ();

    //K.reset_q();
    //K.calc_q(); K.checkConsistency();
//    {
//      static int i=0;
//      FILE(STRING("z.switch_"<<i++<<".g")) <<K;
//    }
    return;
  }

  if(symbol==SW_noJointLink) {
    CHECK_EQ(jointType, JT_none, "");

    if(to->parent) to->unLink();
    to->linkFrom(from, true);

    K.reset_q();
    return;
  }

  if(symbol==makeDynamic) {
    CHECK_EQ(jointType, JT_none, "");
    CHECK_EQ(to, 0, "");
    CHECK(from->inertia, "can only make frames with intertia dynamic");

    from->inertia->type=rai::BT_dynamic;
    if(from->joint) {
      from->joint->H = 1e-1;
    }
    return;
  }

  if(symbol==makeKinematic) {
    CHECK_EQ(jointType, JT_none, "");
    CHECK_EQ(to, 0, "");
    CHECK(from->inertia, "can only make frames with intertia kinematic");

    from->inertia->type=rai::BT_kinematic;
//    if(from->joint){
//      from->joint->constrainToZeroVel=false;
//      from->joint->H = 1e-1;
//    }
    return;
  }

  if(symbol==SW_addContact) {
    CHECK_EQ(jointType, JT_none, "");
    new rai::Contact(*from, *to);
    return;
  }

  if(symbol==SW_delContact) {
    CHECK_EQ(jointType, JT_none, "");
    rai::Contact* c = nullptr;
    for(rai::Contact* cc:to->contacts) if(&cc->a==from || &cc->b==from) { c=cc; break; }
    if(!c) HALT("not found");
    delete c;
    return;
  }

  HALT("shouldn't be here!");
}

rai::String rai::KinematicSwitch::shortTag(const rai::Configuration* G) const {
  rai::String str;
  str <<"  timeOfApplication=" <<timeOfApplication;
  str <<"  symbol=" <<symbol;
  str <<"  jointType=" <<jointType;
  str <<"  fromId=" <<(fromId==-1?"nullptr":(G?G->frames(fromId)->name:STRING(fromId)));
  str <<"  toId=" <<(G?G->frames(toId)->name:STRING(toId)) <<endl;
  return str;
}

void rai::KinematicSwitch::write(std::ostream& os, rai::Configuration* K) const {
  os <<"SWITCH  timeOfApplication=" <<timeOfApplication;
  os <<"  symbol=" <<symbol;
  os <<"  jointType=" <<jointType;
  os <<"  fromId=" <<(int)fromId;
  if(K && fromId<-1) os <<"'" <<K->frames(fromId)->name <<"'";
  os <<"  toId=" <<toId;
  if(K && toId<-1) os <<"'" <<K->frames(toId)->name <<"'";
}

//===========================================================================

/*
rai::KinematicSwitch* rai::KinematicSwitch::newSwitch(const Node *specs, const rai::Configuration& world, int stepsPerPhase, uint T) {
  if(specs->parents.N<2) return nullptr;

  //-- get tags
  rai::String& tt=specs->parents(0)->keys.last();
  rai::String& type=specs->parents(1)->keys.last();
  const char *ref1=nullptr, *ref2=nullptr;
  if(specs->parents.N>2) ref1=specs->parents(2)->keys.last().p;
  if(specs->parents.N>3) ref2=specs->parents(3)->keys.last().p;

  if(tt!="MakeJoint") return nullptr;
  rai::KinematicSwitch* sw = newSwitch(type, ref1, ref2, world, stepsPerPhase + 1);

  if(specs->isGraph()) {
    const Graph& params = specs->graph();
    sw->setTimeOfApplication(params.get<double>("time",1.), params.get<bool>("time", false), stepsPerPhase, T);
//    sw->timeOfApplication = *stepsPerPhase + 1;
    params.get(sw->jA, "from");
    params.get(sw->jB, "to");
  }
  return sw;
}
*/

/*
rai::KinematicSwitch* rai::KinematicSwitch::newSwitch(const rai::String& type, const char* ref1, const char* ref2, const rai::Configuration& world, int _timeOfApplication, const rai::Transformation& jFrom, const rai::Transformation& jTo) {
  //-- create switch
  rai::KinematicSwitch *sw= new rai::KinematicSwitch();
  if(type=="addRigid") { sw->symbol=rai::SW_joint; sw->jointType=rai::JT_rigid; }
//  else if(type=="addRigidRel"){ sw->symbol = rai::KinematicSwitch::addJointAtTo; sw->jointType=rai::JT_rigid; }
  else if(type=="rigidZero") { sw->symbol = rai::SW_joint; sw->jointType=rai::JT_rigid; }
  else if(type=="transXActuated") { sw->symbol = rai::SW_joint; sw->jointType=rai::JT_transX; }
  else if(type=="transXYPhiZero") { sw->symbol = rai::SW_joint; sw->jointType=rai::JT_transXYPhi; }
  else if(type=="transXYPhiActuated") { sw->symbol = rai::SW_joint; sw->jointType=rai::JT_transXYPhi; }
  else if(type=="freeZero") { sw->symbol = rai::SW_joint; sw->jointType=rai::JT_free; }
  else if(type=="freeActuated") { sw->symbol = rai::SW_joint; sw->jointType=rai::JT_free; }
  else if(type=="ballZero") { sw->symbol = rai::SW_joint; sw->jointType=rai::JT_quatBall; }
  else if(type=="hingeZZero") { sw->symbol = rai::SW_joint; sw->jointType=rai::JT_hingeZ; }
  else if(type=="JT_XBall") { sw->symbol = rai::SW_joint; sw->jointType=rai::JT_XBall; }
  else if(type=="JT_transZ") { sw->symbol = rai::SW_joint; sw->jointType=rai::JT_transZ; }
  else if(type=="JT_transX") { sw->symbol = rai::SW_joint; sw->jointType=rai::JT_transX; }
  else if(type=="JT_trans3") { sw->symbol = rai::SW_joint; sw->jointType=rai::JT_trans3; }
  else if(type=="makeDynamic") { sw->symbol = rai::makeDynamic; }
  else if(type=="makeKinematic") { sw->symbol = rai::makeKinematic; }
  else HALT("unknown type: "<< type);
  if(ref1) sw->fromId = world.getFrameByName(ref1)->ID;
  if(ref2) sw->toId = world.getFrameByName(ref2)->ID;
//  if(!ref2){
//    CHECK_EQ(sw->symbol, rai::deleteJoint, "");
//    rai::Body *b = fromShape->body;
//    if(b->hasJoint()==1){
////      CHECK_EQ(b->parentOf.N, 0, "");
//      sw->toId = sw->fromId;
//      sw->fromId = b->joint()->from->shapes.first()->index;
//    }else if(b->parentOf.N==1){
//      CHECK_EQ(b->hasJoint(), 0, "");
//      sw->toId = b->parentOf(0)->from->shapes.first()->index;
//    }else if(b->hasJoint()==0 && b->parentOf.N==0){
//      RAI_MSG("No link to delete for shape '" <<ref1 <<"'");
//      delete sw;
//      return nullptr;
//    }else HALT("that's ambiguous");
//  }else{

  sw->timeOfApplication = _timeOfApplication;
  if(!!jFrom) sw->jA = jFrom;
  if(!!jTo) sw->jB = jTo;
  return sw;
}
*/
