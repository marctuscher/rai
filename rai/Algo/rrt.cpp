/*  ------------------------------------------------------------------
    Copyright (c) 2019 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "rrt.h"

#include <Algo/ann.h>

struct sRRT {
  ANN ann;
  uintA parent;
  double stepsize;
  uint nearest;
};

RRT::RRT(const arr& q0, double _stepsize) : s(new sRRT()) {
  s->ann   .append(q0); //append q as the root of the tree
  s->parent.append(0);    //q has itself as parent
  s->stepsize = _stepsize;
}
double RRT::getProposalTowards(arr& proposal, const arr& q) {
  //find NN
  s->nearest=s->ann.getNN(q);

  //compute little step
  arr d = q - s->ann.X[s->nearest]; //difference vector between q and nearest neighbor
  double dist = length(d);
  if(dist > s->stepsize)
    proposal = s->ann.X[s->nearest] + s->stepsize/dist * d;
  else
    proposal = q;
  return dist;
}
void RRT::add(const arr& q) {
  s->ann.append(q);
  s->parent.append(s->nearest);
}

//some access routines
double RRT::getStepsize() { return s->stepsize; }
uint RRT::getNearest() { return s->nearest; }
uint RRT::getParent(uint i) { return s->parent(i); }
uint RRT::getNumberNodes() { return s->ann.X.d0; }
arr RRT::getNode(uint i) { return s->ann.X[i]; }
void RRT::getRandomNode(arr& q) { q = s->ann.X[rnd(s->ann.X.d0)]; }
arr RRT::getRandomNode() { return s->ann.X[rnd(s->ann.X.d0)]; }

