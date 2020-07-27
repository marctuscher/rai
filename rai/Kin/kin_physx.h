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

struct PhysXInterface : GLDrawer {
  struct PhysXInterface_self* self=0;

  PhysXInterface(const rai::Configuration& C, bool verbose=false);
  ~PhysXInterface();

  void step(double tau=.01);

  void pushKinematicStates(const FrameL& frames);
  void pushFullState(const FrameL& frames, const arr& frameVelocities=NoArr, bool onlyKinematic=false);
  void pullDynamicStates(FrameL& frames, arr& frameVelocities=NoArr);

  void changeObjectType(rai::Frame* f, int type);
  void setArticulatedBodiesKinematic(const rai::Configuration& C);
  void ShutdownPhysX();

  void glDraw(OpenGL&);
  void watch(bool pause=false, const char* txt=nullptr);

  void addForce(rai::Vector& force, rai::Frame* b);
  void addForce(rai::Vector& force, rai::Frame* b, rai::Vector& pos);
};
