import math

class DeadReckoningLocalizer:
	def __init__(self, sideWheelSeparation):
		self.sideWheelSeparation = sideWheelSeparation
		self.x = 0
		self.y = 0
		self.theta = 0

	def update(v, angle, time):
		w = v * math.tan(angle) / self.sideWheelSeparation
		dxR = v * math.sin(w * time) / w
		dyR = v * (1 - math.cos(w * time)) / w
		dthetaR = w * time
		x, y, t = getPosition()
		self.x = math.cos(t) * dxR - math.sin(t) * dyR + x
		self.y = math.sin(t) * dxR + math.cos(t) * dyR + y
		self.theta = (t + dthetaR) % (2 * math.pi)
		return getPosition()

	def getPosition():
		return (self.x, self.y, self.theta)