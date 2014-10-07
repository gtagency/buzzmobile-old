from differential_driver import DifferentialDriver
import math

driver = DifferentialDriver(1, 2, 1, 1, 0.1, 0.1)
print "Expected []:", driver.waypoints
print "Expected (0, 0):", driver.makeTrajectory()
print "Expected (15, 10, 1.884956):", driver.convertToDriverFrame((15, 10, math.pi*3/5))
print "Expected (0, 0):", driver.updatePlan([(15, 10, math.pi*3/5)])
print "Expected True:", driver.updatePosition(10, 5, math.pi/10)
print "Expected (6.30, 3.21, 1.57):", driver.convertToDriverFrame((15, 10, math.pi*3/5))
print "Expected (0.6885, 1.3115):", driver.makeTrajectory()
print "Expected False:", driver.updatePosition(12, 8, math.pi/10)
print "Expected True:", driver.updatePosition(15, 10, math.pi*3/5)
print "Expected []:", driver.waypoints
print "Expected (0, 0):", driver.makeTrajectory()
print "Expected (?, ?):", driver.updatePlan([(20, 15, math.pi*11/10)])
print "Expected (?, ?):", driver.makeTrajectory()
print "Expected (0, 0):", driver.updatePlan([])
print "Expected (0, 0):", driver.makeTrajectory()