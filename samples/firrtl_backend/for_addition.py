import heterocl as hcl

hcl.init()
a = hcl.placeholder((), "a")

def for_loop(a):
  with hcl.Stage("S"):
    b = hcl.local(a, "b")
    with hcl.for_(0, 5, name="i") as i:
      b[0] = b[0] + 1
    return b

s = hcl.create_schedule([a], for_loop)
print(hcl.lower(s))
f = hcl.build(s, target='forfirrtl', name="for_addition")
print(f)
