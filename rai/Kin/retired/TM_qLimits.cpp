/*  ------------------------------------------------------------------
    Copyright (c) 2019 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "TM_qLimits.h"

//===========================================================================

void LimitsConstraint::phi(arr& y, arr& J, const rai::Configuration& G) {
//  if(!limits.N)
  limits = G.getLimits();
  G.kinematicsLimitsCost(y, J, limits, margin);
  y -= .5;
}

