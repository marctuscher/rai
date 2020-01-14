/*  ------------------------------------------------------------------
    Copyright (c) 2019 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include <Core/array.h>
#include <Core/thread.h>
#include <Control/ctrlMsg.h>
#include <Kin/kin.h>

#ifdef RAI_ROS
#include <sensor_msgs/JointState.h>
bool baxter_get_q_qdot_u(arr& q, arr& q_dot, arr& u, const sensor_msgs::JointState& msg, const rai::Configuration& baxterModel);
#endif

struct SendPositionCommandsToBaxter : Thread {
  Var<CtrlMsg> ctrl_ref;
  struct sBaxterInterface* s;

  SendPositionCommandsToBaxter(const rai::Configuration& baxterWorld, const Var<CtrlMsg>& _ctrl_ref);
  ~SendPositionCommandsToBaxter() {}

  void open();
  void step();
  void close();

  bool enablePositionControlL = true;
  bool enablePositionControlR = true;
  bool totalTorqueModeL = false;
  bool totalTorqueModeR = false;
};

struct BaxterInterface {
  struct sBaxterInterface* s;

  BaxterInterface(bool useRosDefault);
  ~BaxterInterface();

  arr get_q();
  arr get_qdot();
  arr get_u();
  bool get_grabbed(const std::string& whichArm);
  bool get_opened(const std::string& whichArm);

  void send_q(const arr& q_ref, bool enableL=true, bool enableR=true);
};
