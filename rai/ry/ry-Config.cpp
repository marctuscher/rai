#ifdef RAI_PYBIND

#include "ry-Config.h"
#include "ry-Frame.h"
#include "ry-KOMO.h"
#include "ry-LGP_Tree.h"
#include "ry-Bullet.h"
#include "ry-PhysX.h"
#include "ry-Operate.h"
#include "ry-Simulation.h"

#include "types.h"

#include <Kin/kin_bullet.h>
#include <Kin/kin_physx.h>
#include <Operate/robotOperation.h>
#include <Kin/kin.h>
#include <Kin/proxy.h>
#include <Kin/kinViewer.h>
#include <Kin/cameraview.h>
#include <Kin/simulation.h>
#include <Gui/viewer.h>
#include <LGP/LGP_tree.h>


void init_Config(pybind11::module &m) {
    pybind11::class_<ry::Config>(m, "Config", "This is a class docstring")
.def(pybind11::init<>())

.def("clear", [](ry::Config& self) {
  self.set()->clear();
} )

.def("copy", [](ry::Config& self, ry::Config& C2) {
  self.set()->copy(C2.get());
},
"make C a (deep) copy of the given C2",
pybind11::arg("C2")
    )

//-- setup/edit the configuration

.def("addFile", [](ry::Config& self, const std::string& fileName) {
  self.set()->addFile(fileName.c_str());
},
"add the contents of the file to C",
pybind11::arg("file_name")
    )

.def("addFrame", [](ry::Config& self, const std::string& name, const std::string& parent, const std::string& args) {
  ry::RyFrame f;
  f.config = self.data;
  f.frame = self.set()->addFrame(name.c_str(), parent.c_str(), args.c_str());
  return f;
},
"add a new frame to C; optionally make this a child to the given parent",
pybind11::arg("name"),
pybind11::arg("parent") = std::string(),
pybind11::arg("args") = std::string()
    )

.def("frame", [](ry::Config& self, const std::string& frameName) {
  ry::RyFrame f;
  f.config = self.data;
  f.frame = self.get()->getFrameByName(frameName.c_str(), true);
  return f;
},
"get access to a frame by name",
pybind11::arg("frameName")
    )

.def("setFrameRelativePose", [](ry::Config& self, const std::string& frame, const std::vector<double>& x) {
  auto Kset = self.set();
  rai::Frame* f = Kset->getFrameByName(frame.c_str(), true);
  f->set_Q()->set(conv_stdvec2arr(x));
}, "TODO remove -> use frame")

.def("delFrame", [](ry::Config& self, const std::string& frameName) {
  auto Kset = self.set();
  rai::Frame* p = Kset->getFrameByName(frameName.c_str(), true);
  if(p) delete p;
},
"destroy and remove a frame from C",
pybind11::arg("frameName")
    )

.def("addObject", [](ry::Config& self, const std::string& name, const std::string& parent,
                     rai::ShapeType shape,
                     const std::vector<double>& size,
                     const std::vector<double>& color,
                     const std::vector<double>& pos,
const std::vector<double>& quat) {
  auto Kset = self.set();
  ry::RyFrame f;
  f.config = self.data;
  f.frame = Kset->addObject(name.c_str(), parent.c_str(), shape, conv_stdvec2arr(size), conv_stdvec2arr(color), conv_stdvec2arr(pos), conv_stdvec2arr(quat));
//    f->name = name;
//    if(parent.size()){
//      rai::Frame *p = Kset->getFrameByName(parent.c_str());
//      if(p) f->linkFrom(p);
//    }
//    if(pos.size()) f->Q.pos.set(pos);
//    if(quat.size()) f->Q.rot.set(quat);
//    if(rot.size()) f->Q.addRelativeRotationDeg(rot[0], rot[1], rot[2], rot[3]);
//    if(f->parent){
//      f->X = f->parent->X * f->Q;
//    }else{
//      f->X = f->Q;
//    }
  return f;
}, "TODO remove! use addFrame only",
pybind11::arg("name"),
pybind11::arg("parent") = std::string(),
pybind11::arg("shape"),
pybind11::arg("size") = std::vector<double>(),
pybind11::arg("color") = std::vector<double>(),
pybind11::arg("pos") = std::vector<double>(),
pybind11::arg("quat") = std::vector<double>()
    )

.def("getJointNames", [](ry::Config& self) {
  return I_conv(self.get()->getJointNames());
},
"get the list of joint names"
    )

.def("getJointDimension", [](ry::Config& self) {
  return self.get()->getJointStateDimension();
},
"get the total number of degrees of freedom"
    )

.def("getJointState", [](ry::Config& self, const ry::I_StringA& joints) {
  arr q;
  if(joints.size()) q = self.get()->getJointState(I_conv(joints));
  else q = self.get()->getJointState();
  return pybind11::array(q.dim(), q.p);
},
"get the joint state as a numpy vector, optionally only for a subset of joints specified as list of joint names",
pybind11::arg("joints") = ry::I_StringA()
    )

.def("setJointState", [](ry::Config& self, const std::vector<double>& q, const ry::I_StringA& joints) {
  arr _q = conv_stdvec2arr(q);
  if(joints.size()) {
    self.set()->setJointState(_q, I_conv(joints));
  } else {
    self.set()->setJointState(_q);
  }
},
"set the joint state, optionally only for a subset of joints specified as list of joint names",
pybind11::arg("q"),
pybind11::arg("joints") = ry::I_StringA()
    )

.def("getFrameNames", [](ry::Config& self) {
  return I_conv(self.get()->getFrameNames());
},
"get the list of frame names"
    )

.def("getJointDimension", [](ry::Config& self) {
  return self.get()->frames.N;
},
"get the total number of frames"
    )

.def("getFrameState", [](ry::Config& self) {
  arr X = self.get()->getFrameState();
  return pybind11::array(X.dim(), X.p);
},
"get the frame state as a n-times-7 numpy matrix, with a 7D pose per frame"
    )

.def("getFrameState", [](ry::Config& self, const char* frame) {
  arr X;
  auto Kget = self.get();
  rai::Frame* f = Kget->getFrameByName(frame, true);
  if(f) X = f->ensure_X().getArr7d();
  return pybind11::array(X.dim(), X.p);
}, "TODO remove -> use individual frame!")

.def("setFrameState", [](ry::Config& self, const std::vector<double>& X, const ry::I_StringA& frames) {
  arr _X = conv_stdvec2arr(X);
  _X.reshape(_X.N/7, 7);
  self.set()->setFrameState(_X, I_conv(frames));
},
"set the frame state, optionally only for a subset of frames specified as list of frame names. \
This also computes the consistent joint state based on the relative poses.",
pybind11::arg("X"),
pybind11::arg("frames") = ry::I_StringA()
    )

.def("setFrameState", [](ry::Config& self, const pybind11::array& X, const ry::I_StringA& frames, bool calc_q_from_X) {
  arr _X = numpy2arr(X);
  _X.reshape(_X.N/7, 7);
  self.set()->setFrameState(_X, I_conv(frames), calc_q_from_X);
},
"set the frame state, optionally only for a subset of frames specified as list of frame names. \
By default this also computes and sets the consistent joint state based on the relative poses.\
Setting calc_q_from_x to false will not compute the joint state and leave the configuration in an inconsistent state!",
pybind11::arg("X"),
pybind11::arg("frames") = ry::I_StringA(),
pybind11::arg("calc_q_from_X") = true
    )

.def("feature", [](ry::Config& self, FeatureSymbol featureSymbol, const ry::I_StringA& frameNames) {
  ry::RyFeature F;
  F.feature = symbols2feature(featureSymbol, I_conv(frameNames), self.get());
  return F;
},
"create a feature (a differentiable map from joint state to a vector space), as they're typically used for IK or optimization. See the dedicated tutorial for details.\
featureSymbol defines which mapping this is (position, vectors, collision distance, etc).\
many mapping refer to one or several frames, which need to be specified using frameNames",
pybind11::arg("featureSymbol"),
pybind11::arg("frameNames"))

.def("evalFeature", [](ry::Config& self, FeatureSymbol fs, const ry::I_StringA& frames) {
  arr y, J;
  self.get()->evalFeature(y, J, fs, I_conv(frames));
  return pybind11::make_tuple(pybind11::array(y.dim(), y.p), pybind11::array(J.dim(), J.p));
}, "TODO remove -> use feature directly"
    )

.def("selectJoints", [](ry::Config& self, const ry::I_StringA& jointNames) {
  // TODO: this is joint groups
  // TODO: maybe call joint groups just joints and joints DOFs
  self.set()->selectJointsByName(I_conv(jointNames));
},
"redefine what are considered the DOFs of this configuration: only joint listed in jointNames are considered\
part of the joint state and define the number of DOFs",
pybind11::arg("jointNames")
    )

.def("selectJointsByTag", [](ry::Config& self, const ry::I_StringA& jointGroups) {
  auto Kset = self.set();
  Kset->selectJointsByGroup(I_conv(jointGroups), true, true);
  Kset->ensure_q();
},
"redefine what are considered the DOFs of this configuration: only joint that have a tag listed in jointGroups are considered\
part of the joint state and define the number of DOFs",
pybind11::arg("jointGroups")
    )

.def("makeObjectsFree", [](ry::Config& self, const ry::I_StringA& objs) {
  self.set()->makeObjectsFree(I_conv(objs));
}, "TODO remove -> to frame")

.def("makeObjectsConvex", [](ry::Config& self) {
  makeConvexHulls(self.set()->frames);
},
"remake all meshes associated with all frames to become their convex hull"
    )

.def("attach", [](ry::Config& self, const std::string& frame1, const std::string& frame2) {
  auto Kset = self.set();
  Kset->attach(frame1.c_str(), frame2.c_str());
},
"change the configuration by creating a rigid joint from frame1 to frame2, adopting their current\
relative pose. This also breaks the first joint joints that is parental to frame2 and reverses the\
topological order from frame2 to the broken joint"
    )

.def("computeCollisions", [](ry::Config& self) {
  self.set()->stepSwift();
},
"call the broadphase collision engine (SWIFT++) to generate the list of collisions (or near proximities)\
between all frame shapes that have the collision tag set non-zero"
    )

.def("getCollisions", [](ry::Config& self, double belowMargin) {
  pybind11::list ret;
  auto Kget = self.get();
  for(const rai::Proxy& p: Kget->proxies) {
    if(!p.coll)((rai::Proxy*)&p)->calc_coll(Kget);
    if(p.d>belowMargin) continue;
    pybind11::tuple tuple(3);
    tuple[0] = p.a->name.p;
    tuple[1] = p.b->name.p;
    tuple[2] = p.d;
//      tuple[3] = p.posA;
//      tuple[4] = p.posB;
    ret.append(tuple) ;
  }
  return ret;
},
"return the results of collision computations: a list of 3 tuples with (frame1, frame2, distance).\
Optionally report only on distances below a margin\
To get really precise distances and penetrations use the FS.distance feature with the two frame names",
pybind11::arg("belowMargin") = 1.
    )

.def("getFrameBox", [](ry::Config& self, const std::string& framename) {
  auto Kget = self.get();
  rai::Frame* f = Kget->getFrameByName(framename.c_str(), true);
  rai::Shape* s = f->shape;
  CHECK(s, "frame " <<f->name <<" does not have a shape");
  CHECK(s->type() == rai::ST_ssBox || s->type() == rai::ST_box,
        "frame " <<f->name <<" needs to be a box");
  arr range = s->size();
  return pybind11::array(range.dim(), range.p);
}, "TODO remove -> frame.getShape")

.def("view", [](ry::Config& self, const std::string& frame) {
  ry::ConfigViewer view;
  view.view = make_shared<KinViewer>(self, -1, rai::String(frame));
  return view;
},
"create a viewer for this configuration. Optionally, specify a frame that is the origin of the viewer camera",
pybind11::arg("frame")="")

.def("cameraView", [](ry::Config& self) {
  ry::RyCameraView view;
  view.cam = make_shared<rai::CameraView>(self.get(), true, 0);
  return view;
},
"create an offscreen renderer for this configuration"
    )

.def("edit", [](ry::Config& self, const char* fileName) {
  rai::Configuration K;
  editConfiguration(fileName, K);
  self.set() = K;
},
"launch a viewer that listents (inode) to changes of a file (made by you in an editor), and\
reloads, displays and animates the configuration whenever the file is changed"
    )

.def("komo_IK", [](ry::Config& self, bool useSwift) {
  ry::RyKOMO komo;
  komo.komo = make_shared<KOMO>(self.get(), useSwift);
  komo.config.set() = komo.komo->world;
  komo.komo->setIKOpt();
  return komo;
},
"create KOMO solver configured to IK, useSwift determine whether for each\
query the broadphase collision computations are done. (Necessary only when generic\
FS.accumulatedCollision feature is needed. The explicit distance feature is independent\
from broadphase collision computation)",
pybind11::arg("useSwift")
    )

.def("komo_CGO", [](ry::Config& self, uint numConfigs, bool useSwift) {
  CHECK_GE(numConfigs, 1, "");
  ry::RyKOMO komo;
  komo.komo = make_shared<KOMO>(self.get(), useSwift);
  komo.config.set() = komo.komo->world;
  komo.komo->setDiscreteOpt(numConfigs);
  return komo;
},
"create KOMO solver configured for dense graph optimization,\
numConfig gives the number of configurations optimized over,\
useSwift determine whether for each\
query the broadphase collision computations are done. (Necessary only when generic\
FS.accumulatedCollision feature is needed. The explicit distance feature is independent\
from broadphase collision computation)",
pybind11::arg("numConfigs"),
pybind11::arg("useSwift")
    )

.def("komo_path",  [](ry::Config& self, double phases, uint stepsPerPhase, double timePerPhase, bool useSwift) {
  ry::RyKOMO komo;
  komo.komo = make_shared<KOMO>(self.get(), useSwift);
  komo.config.set() = komo.komo->world;
  komo.komo->setPathOpt(phases, stepsPerPhase, timePerPhase);
  komo.komo->setSquaredQAccVelHoming();
  return komo;
},
"create KOMO solver configured for sparse path optimization",
pybind11::arg("phases"),
pybind11::arg("stepsPerPhase")=20,
pybind11::arg("timePerPhase")=5.,
pybind11::arg("useSwift")
    )

.def("lgp", [](ry::Config& self, const std::string& folFileName) {
  ry::RyLGP_Tree lgp;
  lgp.lgp = make_shared<LGP_Tree_Thread>(self.get(), folFileName.c_str());
  return lgp;
},
"create an LGP solver"
    )

.def("bullet", [](ry::Config& self) {
  ry::RyBullet bullet;
  bullet.bullet = make_shared<BulletInterface>(self.set());
  return bullet;
},
"create a Bullet engine for physical simulation from the configuration: The configuration\
is being exported into a bullet instance, which can be stepped forward, and the result syced back to this configuration"
    )

.def("physx", [](ry::Config& self) {
  ry::RyPhysX physx;
  physx.physx = make_shared<PhysXInterface>(self.set());
  return physx;
},
"create a PhysX engine for physical simulation from the configuration: The configuration\
is being exported into a bullet instance, which can be stepped forward, and the result syced back to this configuration"
    )

.def("simulation", [](ry::Config& self, rai::Simulation::SimulatorEngine engine, bool display) {
  ry::RySimulation sim;
  sim.sim = make_shared<rai::Simulation>(self.set(), engine, display);
  return sim;
},
"create a generic Simulation engine, which can internally call PhysX, Bullet, or just kinematics to forward simulate,\
allows you to control robot motors by position, velocity, or accelerations,\
    and allows you go query camera images and depth",
    pybind11::arg("engine"),
    pybind11::arg("display")
    )

.def("operate", [](ry::Config& self, const char* rosNodeName) {
  ry::RyOperate op;
  op.R = make_shared<RobotOperation>(self.get(), .01, rosNodeName);
  return op;
},
"create a module (including ROS node) to sync this configuration both ways (reading state, and controlling) to a real robot",
pybind11::arg("rosNodeName")
    )

.def("sortFrames", [](ry::Config& self) {
  self.set()->sortFrames();
})

.def("equationOfMotion", [](ry::Config& self, std::vector<double>& qdot, bool gravity) {
  arr M, F;
  arr _qdot = conv_stdvec2arr(qdot);
  self.set()->equationOfMotion(M, F, _qdot, gravity);
  return pybind11::make_tuple(pybind11::array(M.dim(), M.p), pybind11::array(F.dim(), F.p));
}, "",
pybind11::arg("qdot"),
pybind11::arg("gravity"))

.def("stepDynamics", [](ry::Config& self, std::vector<double>& qdot, std::vector<double>& u_control, double tau, double dynamicNoise, bool gravity) {
  arr _qdot = conv_stdvec2arr(qdot);
  arr _u = conv_stdvec2arr(u_control);
  self.set()->stepDynamics(_qdot, _u, tau, dynamicNoise, gravity);
  return pybind11::array(_qdot.dim(), _qdot.p);
}, "",
pybind11::arg("qdot"),
pybind11::arg("u_control"),
pybind11::arg("tau"),
pybind11::arg("dynamicNoise"),
pybind11::arg("gravity"))

;

//===========================================================================

pybind11::class_<ry::ConfigViewer>(m, "ConfigViewer");
pybind11::class_<ry::PathViewer>(m, "PathViewer");
pybind11::class_<ry::PointCloudViewer>(m, "PointCloudViewer");
pybind11::class_<ry::ImageViewer>(m, "ImageViewer");

//===========================================================================

pybind11::class_<ry::RyCameraView>(m, "CameraView")
.def("updateConfig", [](ry::RyCameraView& self, ry::Config& config) {
  auto Cget = config.get();
  if(Cget->frames.N!= self.cam->K.frames.N) {
    self.cam->K.copy(Cget);
  } else {
    self.cam->K.setFrameState(Cget->getFrameState());
  }
})

.def("addSensor", [](ry::RyCameraView& self, const char* name, const char* frameAttached, uint width, uint height, double focalLength, double orthoAbsHeight, const std::vector<double>& zRange, const std::string& backgroundImageFile) {
  self.cam->addSensor(name, frameAttached, width, height, focalLength, orthoAbsHeight, arr(zRange), backgroundImageFile.c_str());
}, "",
pybind11::arg("name"),
pybind11::arg("frameAttached"),
pybind11::arg("width"),
pybind11::arg("height"),
pybind11::arg("focalLength") = -1.,
pybind11::arg("orthoAbsHeight") = -1.,
pybind11::arg("zRange") = std::vector<double>(),
pybind11::arg("backgroundImageFile") = std::string())
.def("addSensor", [](ry::RyCameraView& self, const char* frameAttached) {
  self.cam->addSensor(frameAttached);
})

.def("selectSensor", [](ry::RyCameraView& self, const char* sensorName) {
  self.cam->selectSensor(sensorName);
}, "",
pybind11::arg("name"))

.def("computeImageAndDepth", [](ry::RyCameraView& self, bool visualsOnly) {
  auto imageSet = self.image.set();
  auto depthSet = self.depth.set();
  if(visualsOnly) self.cam->renderMode = rai::CameraView::visuals;
  else self.cam->renderMode = rai::CameraView::all;
  self.cam->computeImageAndDepth(imageSet, depthSet);
  pybind11::tuple ret(2);
  ret[0] = pybind11::array(imageSet->dim(), imageSet->p);
  ret[1] = pybind11::array(depthSet->dim(), depthSet->p);
  return ret;
},
pybind11::arg("visualsOnly")=true
    )

.def("computePointCloud", [](ry::RyCameraView& self, const pybind11::array& depth, bool globalCoordinates) {
  arr _depth = numpy2arr(depth);
  floatA __depth; copy(__depth, _depth);
  auto ptsSet = self.pts.set();
  self.cam->computePointCloud(ptsSet, __depth, globalCoordinates);
  return pybind11::array(ptsSet->dim(), ptsSet->p);
}, "",
pybind11::arg("depth"),
pybind11::arg("globalCoordinates") = true)

.def("computeSegmentation", [](ry::RyCameraView& self) {
  auto segSet = self.segmentation.set();
  self.cam->computeSegmentation(segSet);
  return pybind11::array(segSet->dim(), segSet->p);
})

.def("pointCloudViewer", [](ry::RyCameraView& self) {
  ry::PointCloudViewer ret;
  ret.view = make_shared<PointCloudViewer>(self.pts, self.image);
  return ret;
})

.def("imageViewer", [](ry::RyCameraView& self) {
  ry::ImageViewer ret;
  ret.view = make_shared<ImageViewer>(self.image);
  return ret;
})

.def("segmentationViewer", [](ry::RyCameraView& self) {
  ry::ImageViewer ret;
  ret.view = make_shared<ImageViewer>(self.segmentation);
  return ret;
})

//-- displays
//      void watch_PCL(const arr& pts, const byteA& rgb);
;

//===========================================================================

pybind11::class_<ry::RyFeature>(m, "Feature")
.def("eval", [](ry::RyFeature& self, ry::Config& K) {
  arr y, J;
  self.feature->__phi(y, J, K.get());
  pybind11::tuple ret(2);
  ret[0] = pybind11::array(y.dim(), y.p);
  ret[1] = pybind11::array(J.dim(), J.p);
  return ret;
})
.def("eval", [](ry::RyFeature& self, pybind11::tuple& Kpytuple) {
  ConfigurationL Ktuple;
  for(uint i=0; i<Kpytuple.size(); i++) {
    ry::Config& K = Kpytuple[i].cast<ry::Config&>();
    Ktuple.append(&K.set()());
  }

  arr y, J;
  self.feature->order=Ktuple.N-1;
  self.feature->__phi(y, J, Ktuple);
  cout <<"THERE!!" <<J.dim() <<endl;
  pybind11::tuple ret(2);
  ret[0] = pybind11::array(y.dim(), y.p);
  ret[1] = pybind11::array(J.dim(), J.p);
  return ret;
})
.def("description", [](ry::RyFeature& self, ry::Config& K) {
  std::string s = self.feature->shortTag(K.get()).p;
  return s;
})
;

#define ENUMVAL(pre, x) .value(#x, pre##_##x)

  pybind11::enum_<rai::ShapeType>(m, "ST")
  ENUMVAL(rai::ST, none)
  ENUMVAL(rai::ST, box)
  ENUMVAL(rai::ST, sphere)
  ENUMVAL(rai::ST, capsule)
  ENUMVAL(rai::ST, mesh)
  ENUMVAL(rai::ST, cylinder)
  ENUMVAL(rai::ST, marker)
  ENUMVAL(rai::ST, pointCloud)
  ENUMVAL(rai::ST, ssCvx)
  ENUMVAL(rai::ST, ssBox)
  .export_values();

  pybind11::enum_<FeatureSymbol>(m, "FS")
  ENUMVAL(FS, position)
  ENUMVAL(FS, positionDiff)
  ENUMVAL(FS, positionRel)
  ENUMVAL(FS, quaternion)
  ENUMVAL(FS, quaternionDiff)
  ENUMVAL(FS, quaternionRel)
  ENUMVAL(FS, pose)
  ENUMVAL(FS, poseDiff)
  ENUMVAL(FS, poseRel)
  ENUMVAL(FS, vectorX)
  ENUMVAL(FS, vectorXDiff)
  ENUMVAL(FS, vectorXRel)
  ENUMVAL(FS, vectorY)
  ENUMVAL(FS, vectorYDiff)
  ENUMVAL(FS, vectorYRel)
  ENUMVAL(FS, vectorZ)
  ENUMVAL(FS, vectorZDiff)
  ENUMVAL(FS, vectorZRel)
  ENUMVAL(FS, scalarProductXX)
  ENUMVAL(FS, scalarProductXY)
  ENUMVAL(FS, scalarProductXZ)
  ENUMVAL(FS, scalarProductYX)
  ENUMVAL(FS, scalarProductYY)
  ENUMVAL(FS, scalarProductYZ)
  ENUMVAL(FS, scalarProductZZ)
  ENUMVAL(FS, gazeAt)

  ENUMVAL(FS, angularVel)

  ENUMVAL(FS, accumulatedCollisions)
  ENUMVAL(FS, jointLimits)
  ENUMVAL(FS, distance)
  ENUMVAL(FS, oppose)

  ENUMVAL(FS, qItself)

  ENUMVAL(FS, aboveBox)
  ENUMVAL(FS, insideBox)

  ENUMVAL(FS, standingAbove)

  ENUMVAL(FS, physics)
  ENUMVAL(FS, contactConstraints)
  ENUMVAL(FS, energy)

  ENUMVAL(FS, transAccelerations)
  ENUMVAL(FS, transVelocities)
  .export_values();

#undef ENUMVAL
#define ENUMVAL(x) .value(#x, rai::Simulation::_##x)

  pybind11::enum_<rai::Simulation::SimulatorEngine>(m, "SimulatorEngine")
      ENUMVAL(physx)
      ENUMVAL(bullet)
      ENUMVAL(kinematic)
      .export_values();

  pybind11::enum_<rai::Simulation::ControlMode>(m, "ControlMode")
      ENUMVAL(position)
      ENUMVAL(velocity)
      ENUMVAL(acceleration)
      .export_values();

}

#endif
