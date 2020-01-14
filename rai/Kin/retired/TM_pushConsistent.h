/*  ------------------------------------------------------------------
    Copyright (c) 2019 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once
#include "feature.h"

struct TM_PushConsistent : Feature {
  int i, j;               ///< which shapes does it refer to?

  TM_PushConsistent(int iShape=-1, int jShape=-1);

  TM_PushConsistent(const rai::Configuration& G,
                    const char* iShapeName=nullptr, const char* jShapeName=nullptr);

  virtual void phi(arr& y, arr& J, const ConfigurationL& Ktuple);
  virtual void phi(arr& y, arr& J, const rai::Configuration& G) {  HALT("you shouldn't be here!");  }
  virtual uint dim_phi(const rai::Configuration& G) { return 3; }
  virtual rai::String shortTag(const rai::Configuration& G);
};

