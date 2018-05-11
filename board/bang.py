from panda import Panda

panda = Panda()
panda.set_safety_mode(3) #SAFETY_GM
panda.set_gmlan(2)

panda.can_clear(0)

panda.can_send(0x10400060, "873c01ff", 2) #chime on gmlan (bus #3)

print(panda.health())