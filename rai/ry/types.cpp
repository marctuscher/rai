#ifdef RAI_PYBIND

#include "types.h"
#include "../Geo/geoms.h"

pybind11::dict graph2dict(const rai::Graph& G) {
  pybind11::dict dict;
  for(rai::Node* n:G) {
    rai::String key;
    if(key.N) key=n->key;
    else key <<n->index;

    //-- write value
    if(n->isGraph()) {
      dict[key.p] = graph2dict(n->get<rai::Graph>());
    } else if(n->isOfType<rai::String>()) {
      dict[key.p] = n->get<rai::String>().p;
    } else if(n->isOfType<arr>()) {
      dict[key.p] = conv_arr2stdvec(n->get<arr>());
    } else if(n->isOfType<intA>()) {
      dict[key.p] = conv_arr2stdvec(n->get<intA>());
    } else if(n->isOfType<uintA>()) {
      dict[key.p] = conv_arr2stdvec(n->get<uintA>());
    } else if(n->isOfType<boolA>()) {
      dict[key.p] = conv_arr2stdvec(n->get<boolA>());
    } else if(n->isOfType<double>()) {
      dict[key.p] = n->get<double>();
    } else if(n->isOfType<int>()) {
      dict[key.p] = n->get<int>();
    } else if(n->isOfType<uint>()) {
      dict[key.p] = n->get<uint>();
    } else if(n->isOfType<bool>()) {
      dict[key.p] = n->get<bool>();
    } else if(n->isOfType<rai::Enum<rai::ShapeType>>()) {
      dict[key.p] = n->get<rai::Enum<rai::ShapeType>>().name();
    } else {
      LOG(-1) <<"can't convert node of type " <<n->type.name() <<" to dictionary";
    }
  }
  return dict;
}

pybind11::list graph2list(const rai::Graph& G) {
  pybind11::list list;
  for(rai::Node* n:G) {
    //-- write value
    if(n->isGraph()) {
      list.append(graph2dict(n->get<rai::Graph>()));
    } else if(n->isOfType<rai::String>()) {
      list.append(n->get<rai::String>().p);
    } else if(n->isOfType<arr>()) {
      list.append(conv_arr2stdvec(n->get<arr>()));
    } else if(n->isOfType<double>()) {
      list.append(n->get<double>());
    } else if(n->isOfType<int>()) {
      list.append(n->get<int>());
    } else if(n->isOfType<uint>()) {
      list.append(n->get<uint>());
    } else if(n->isOfType<bool>()) {
      list.append(n->get<bool>());
    } else {
    }

  }
  return list;
}

pybind11::tuple uintA2tuple(const uintA& tup) {
  pybind11::tuple tuple;
  for(uint i=0; i<tup.N; i++) tuple[i] = tup(i);
  return tuple;
}

arr numpy2arr(const pybind11::array& X) {
  arr Y;
  uintA dim(X.ndim());
  for(uint i=0; i<dim.N; i++) dim(i)=X.shape()[i];
  Y.resize(dim);
  auto ref = X.unchecked<double>();
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

byteA numpy2arr(const pybind11::array_t<byte>& X) {
  byteA Y;
  uintA dim(X.ndim());
  for(uint i=0; i<dim.N; i++) dim(i)=X.shape()[i];
  Y.resize(dim);
  auto ref = X.unchecked<>();
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

uintA numpy2arr(const pybind11::array_t<uint> &X)
{
  uintA Y;
  uintA dim(X.ndim());
  for (uint i = 0; i < dim.N; i++)
    dim(i) = X.shape()[i];
  Y.resize(dim);
  auto ref = X.unchecked<>();
  if (Y.nd == 1)
  {
    for (uint i = 0; i < Y.d0; i++)
      Y(i) = ref(i);
    return Y;
  }
  else if (Y.nd == 2)
  {
    for (uint i = 0; i < Y.d0; i++)
      for (uint j = 0; j < Y.d1; j++)
        Y(i, j) = ref(i, j);
    return Y;
  }
  else if (Y.nd == 3)
  {
    for (uint i = 0; i < Y.d0; i++)
      for (uint j = 0; j < Y.d1; j++)
        for (uint k = 0; k < Y.d2; k++)
          Y(i, j, k) = ref(i, j, k);
    return Y;
  }
  NIY;
  return Y;
}

floatA numpy2arr(const pybind11::array_t<float> &X)
{
  floatA Y;
  uintA dim(X.ndim());
  for (uint i = 0; i < dim.N; i++)
    dim(i) = X.shape()[i];
  Y.resize(dim);
  auto ref = X.unchecked<>();
  if (Y.nd == 1)
  {
    for (uint i = 0; i < Y.d0; i++)
      Y(i) = ref(i);
    return Y;
  }
  else if (Y.nd == 2)
  {
    for (uint i = 0; i < Y.d0; i++)
      for (uint j = 0; j < Y.d1; j++)
        Y(i, j) = ref(i, j);
    return Y;
  }
  else if (Y.nd == 3)
  {
    for (uint i = 0; i < Y.d0; i++)
      for (uint j = 0; j < Y.d1; j++)
        for (uint k = 0; k < Y.d2; k++)
          Y(i, j, k) = ref(i, j, k);
    return Y;
  }
  NIY;
  return Y;
}

intA numpy2arr(const pybind11::array_t<int> &X)
{
  intA Y;
  uintA dim(X.ndim());
  for (uint i = 0; i < dim.N; i++)
    dim(i) = X.shape()[i];
  Y.resize(dim);
  auto ref = X.unchecked<>();
  if (Y.nd == 1)
  {
    for (uint i = 0; i < Y.d0; i++)
      Y(i) = ref(i);
    return Y;
  }
  else if (Y.nd == 2)
  {
    for (uint i = 0; i < Y.d0; i++)
      for (uint j = 0; j < Y.d1; j++)
        Y(i, j) = ref(i, j);
    return Y;
  }
  else if (Y.nd == 3)
  {
    for (uint i = 0; i < Y.d0; i++)
      for (uint j = 0; j < Y.d1; j++)
        for (uint k = 0; k < Y.d2; k++)
          Y(i, j, k) = ref(i, j, k);
    return Y;
  }
  NIY;
  return Y;
}

arr vecvec2arr(const std::vector<std::vector<double> >& X) {
  CHECK(X.size()>0, "");
  arr Y(X.size(), X[0].size());
  for(uint i=0; i<Y.d0; i++) for(uint j=0; j<Y.d1; j++) Y(i, j) = X[i][j];
  return Y;
}

namespace pybind11 {
  namespace detail {
    template <typename T> struct type_caster<rai::Array<T>> {
    public:
      PYBIND11_TYPE_CASTER(rai::Array<T>, _("rai::Array<T>"));

      /// Conversion part 1 (Python->C++): convert numpy array to rai::Array<T>
      bool load(handle src, bool) {
        auto buf = pybind11::array_t<T>::ensure(src);
        if ( !buf )
          return false;
        rai::Array<T> a = numpy2arr(buf);
        value = a;
        /* Ensure return code was OK (to avoid out-of-range errors etc) */
        return !PyErr_Occurred();
      }

      /// Conversion part 2 (C++ -> Python): convert rai::Array<T> instance to numpy array
      static handle cast(rai::Array<T> src, return_value_policy /* policy */, handle /* parent */) {
        pybind11::array ret(src.dim(), src.p);
        return ret.release();
      }
    };
  }
}
#endif
