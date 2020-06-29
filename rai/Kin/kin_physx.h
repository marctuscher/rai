/*  ------------------------------------------------------------------
    Copyright (c) 2019 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "kin.h"

namespace physx {
class PxMaterial;
}


namespace rai {
  class ContactPair {
    public:
      rai::Frame* first;
      rai::Frame* second;
      arr contact_positions;
      arr contact_impulses;
      bool isContact;
      ContactPair(rai::Frame* _first, rai::Frame* _second, const arr& _contact_positions, const arr& _contact_impulses, const bool& _isContact) {
        first = _first;
        second = _second;
        contact_positions = _contact_positions;
        contact_impulses = _contact_impulses;
        isContact = _isContact;
      };
  };
}
struct PhysXInterface : GLDrawer {
  struct PhysXInterface_self* self=0;

  PhysXInterface(const rai::Configuration& C, int verbose=1);
  ~PhysXInterface();

  void step(double tau=.01);

  void pushKinematicStates(const FrameL& frames, const arr &q, const arr& q_dot=NoArr);
  void pushFullState(const FrameL& frames, const arr& frameVelocities=NoArr, bool onlyKinematic=false);
  void pullDynamicStates(FrameL& frames, arr& frameVelocities=NoArr);

  void changeObjectType(rai::Frame* f, int type);
  void setArticulatedBodiesKinematic(const rai::Configuration& C);
  void ShutdownPhysX();

  void glDraw(OpenGL&);
  void watch(bool pause = false, const char *txt = nullptr);
  bool doesGripperGraspObject(const rai::Frame* gripper, const rai::Frame* object);

  void addForce(rai::Vector& force, rai::Frame* b);
  rai::Array<std::shared_ptr<rai::ContactPair>> getContacts();
  void addForce(rai::Vector& force, rai::Frame* b, rai::Vector& pos);
};
