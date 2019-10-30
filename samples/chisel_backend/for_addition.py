# import heterocl package
import heterocl as hcl

# initialize the hcl project
hcl.init()
# constructure hcl placeholder for inputs/outputs
a = hcl.placeholder((), "a")
# the computing function
def for_loop(a):
  with hcl.Stage("S"):
    b = hcl.local(a, "b")
    with hcl.for_(0, 5, name="i") as i:
      b[0] = b[0] + 1
    return b
# create a schedule for compute optimization
s = hcl.create_schedule([a], for_loop)
# print out the lowered function, i.e., hcl IR
print(hcl.lower(s))
# generate the code for a specific backend
f = hcl.build(s, target='chisel', name="for_addition")
# print out the generated code
print(f)
