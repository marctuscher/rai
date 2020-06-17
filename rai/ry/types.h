#pragma once

#ifdef RAI_PYBIND

#include "../Core/array.h"
#include "../Core/graph.h"

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>

namespace ry {
  typedef std::pair<std::vector<unsigned int>, std::vector<double>> I_arr;
  typedef std::vector<std::string> I_StringA;
  typedef std::map<std::string, std::string> I_dict;
  typedef std::map<std::string, std::vector<double>> I_args;

  typedef std::tuple<std::vector<double>, int, int, I_StringA, I_args> I_feature;
  typedef std::vector<I_feature> I_features;

  typedef std::tuple<std::vector<double>, std::string, I_StringA, I_args> I_objective;
  typedef std::vector<I_objective> I_objectives;
};

pybind11::dict graph2dict(const rai::Graph& G);

pybind11::list graph2list(const rai::Graph& G);

pybind11::tuple uintA2tuple(const uintA& tup);

template<class T> rai::Array<T> numpy2arr(const pybind11::array_t<T>& X) {
  rai::Array<T> Y;
  uintA dim(X.ndim());
  for(uint i=0; i<dim.N; i++) dim(i)=X.shape()[i];
  Y.resize(dim);
  auto ref = X.unchecked();
  if(Y.nd==1) {
    for(uint i=0; i<Y.d0; i++) Y(i) = ref(i);
    return Y;
  } else if(Y.nd==2) {
    for(uint i=0; i<Y.d0; i++) for(uint j=0; j<Y.d1; j++) Y(i, j) = ref(i, j);
    return Y;
  } else if(Y.nd==3) {
    for(uint i=0; i<Y.d0; i++) for(uint j=0; j<Y.d1; j++) for(uint k=0; k<Y.d2; k++) Y(i, j, k) = ref(i, j, k);
    return Y;
  }
  NIY;
  return Y;
}

arr vecvec2arr(const std::vector<std::vector<double>>& X);

inline StringA I_conv(const ry::I_StringA& x) {
  StringA y(x.size());
  for(uint i=0; i<y.N; i++) y(i) = x[i];
  return y;
}

inline ry::I_StringA I_conv(const StringA& x) {
  ry::I_StringA y;
  for(const rai::String& s:x) y.push_back(s.p);
  return y;
}

inline rai::Graph I_conv(const ry::I_dict& x) {
  return rai::Graph(x);
}

inline ry::I_arr I_conv(const arr& x) {
  ry::I_arr y;
  y.first = x.dim();
  y.second = x;
  return y;
}

inline arr I_conv(const ry::I_arr& x) {
  arr y;
  y = conv_stdvec2arr(x.second);
  y.reshape(conv_stdvec2arr(x.first));
  return y;
}

namespace pybind11 {
  namespace detail {

    //** StringA <--> vector<std::string>
    template <>  struct type_caster<StringA> {
    public:
      PYBIND11_TYPE_CASTER(StringA, _("StringA"));

      /// Conversion part 1 (Python->C++): convert numpy array to rai::Array<T>
      bool load(pybind11::handle src, bool) {
        std::vector<std::string> strings = src.cast<std::vector<std::string>>();
        value = I_conv(strings);
        /* Ensure return code was OK (to avoid out-of-range errors etc) */
        return !PyErr_Occurred();
      }

      /// Conversion part 2 (C++ -> Python): convert rai::Array<T> instance to numpy array
      static handle cast(StringA src, return_value_policy /* policy */, handle /* parent */) {
        std::vector<std::string> strings = I_conv(src);
        return pybind11::cast(strings);
      }
    };

    //** rai::Array <--> numpy
    template <typename T>  struct type_caster<rai::Array<T>> {
    public:
      PYBIND11_TYPE_CASTER(rai::Array<T>, _("rai::Array<T>"));

      /// Conversion part 1 (Python->C++): convert numpy array to rai::Array<T>
      bool load(pybind11::handle src, bool) {
        auto buf = pybind11::array_t<T>::ensure(src);
        if ( !buf ){
          LOG(-1) <<"THIS IS NOT A NUMPY ARRAY!";
          return false;
        }
        value = numpy2arr<T>(buf);
        /* Ensure return code was OK (to avoid out-of-range errors etc) */
        return !PyErr_Occurred();
      }

      /// Conversion part 2 (C++ -> Python): convert rai::Array<T> instance to numpy array
      static handle cast(rai::Array<T> src, return_value_policy /* policy */, handle /* parent */) {
        pybind11::array_t<T> ret(src.dim(), src.p);
        return ret.release();
      }
    };
  }
}

#endif
