body cabinet_base {
  X=<T t(0 0 0) d(0 0 0 1)> size=[1. 1. 1. .0] color=[.8 0 0] 
}
shape (cabinet_base) {
  name="top" shape:box color=[0 0.3 0.7] rel=<T t(.0 .0 1.3)> size=[1. 1. .2 .0]
}
shape (cabinet_base) {
  name="bottom" shape:box color=[0 0.3 0.7] rel=<T t(.0 .0 .4)> size=[1. 1. .8 .0]
}
shape (cabinet_base) {
  name="left" shape:box color=[0 0.3 0.7] rel=<T t(.0 .55 .7)> size=[1. .1 1.4 .0]
}
shape (cabinet_base) {
  name="right" shape:box color=[0 .3 .7] rel=<T t(.0 -.55 .7)> size=[1. .1 1.4 .0]
}
body cabinet_drawer { shape:box size=[1. 1. .4 .0] color=[.5 .5 0]  }

joint drawer_joint (cabinet_base cabinet_drawer) {
  joint:transX A=<T t(0 0 1)> limit=[-.9 .9 0 0 0]
}

body marker { X=<T t(2 0 1)> size=[.5 .5 .5 .5] color=[0 0 .7] shape:marker }
