/*  ------------------------------------------------------------------
    Copyright (c) 2019 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include <Kin/kin.h>
#include <Core/thread.h>

struct RosCamera {
  Var<byteA> rgb;
  Var<floatA> depth;
  std::shared_ptr<struct sRosCamera> s;

  RosCamera(Var<byteA>& _rgb, Var<floatA> _depth,
            const char* rosNodeName = "rai_node",
            const char* rgb_topic = "/camera/rgb/image_rect_color",
            const char* depth_topic = "/camera/depth_registered/image_raw",
            bool useUint=false);
  ~RosCamera();
};
