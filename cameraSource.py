import sensor, image, time
from pyb import UART
import math
import pyb
uart = UART(3,115200, timeout_char = 1000)
uart.init(115200, bits=8, parity=None, stop=1, timeout_char=1000)
orangeThreshold = [(40, 70, 10, 80, 20, 80)]
yellowThreshold = [(30, 80, -20, 35, 40, 80)]
blueThreshold = [(0, 0, 0, 0, 0, 0)]
goalAreaThresh = 80
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQVGA)
sensor.set_windowing((120,120))
sensor.skip_frames(time=2500)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False, rgb_gain_db = (-3.0, -4.0, -0.5))
sensor.set_vflip(True)
current_exposure_time_in_microseconds = sensor.get_exposure_us()
EXPOSURE_TIME_SCALE = 0.5
set_exposure_time_in_microseconds = current_exposure_time_in_microseconds * EXPOSURE_TIME_SCALE
sensor.set_auto_exposure(False, \
	exposure_us = int(set_exposure_time_in_microseconds))
sensor.skip_frames(time=2500)
calibratedL_mean = 40
brightnesses = [-3, -2, -1, 0, 1, 2, 3]
l_meanDiff = 1000
previousBrightness = -5
for brightness in brightnesses:
	sensor.set_brightness(brightness)
	img = sensor.snapshot()
	currDiff = abs(calibratedL_mean-img.get_statistics().l_mean())
	if currDiff >= l_meanDiff:
		break
	else:
		l_meanDiff = currDiff
		previousBrightness = brightness
sensor.set_brightness(previousBrightness)
print(img.get_statistics().l_mean(), current_exposure_time_in_microseconds)
clockFlag = False
ledFlag = False
crossFlag = True
distFlag = False
ballDebugFlag = False
goalDebugFlag = False
roleDebug = True
heartbeatDebug = False
bGoalFlag = True
yGoalFlag = True
roleBGoalSend = False
roleYGoalSend = True
roleBallSend = False
roleRecv = True
otherBot = True
timeLastTalked = pyb.millis()
botCx = 59
botCy = 57
def getCentroid(bl):
	blobRect = bl.rect()
	cx = blobRect[0]+blobRect[2]/2
	cy = blobRect[1]+blobRect[3]/2
	return cx, cy, blobRect
def getPolar(x, y):
	radius = ((x-botCx)**2 + (y-botCy)**2)**0.5
	angle = math.atan2(y-botCy,x-botCx)
	return radius, angle
def getBotPolar(x, y):
	radius = ((x-botCx)**2 + (y-botCy)**2)**0.5
	angle = int(round(math.atan2(botCx-x,botCy-y) * 180/math.pi))
	if distFlag:
		print(radius, radiusCorr)
	return radius, angle
def endsToCentre(lowMarkX, lowMarkY, highMarkX, highMarkY):
	lowRadiusCorr, lowAngleCorr = getPolar(lowMarkX, lowMarkY)
	highRadiusCorr, highAngleCorr = getPolar(highMarkX, highMarkY)
	lowXCorr, lowYCorr = lowRadiusCorr*math.cos(lowAngleCorr), lowRadiusCorr*math.sin(lowAngleCorr)
	highXCorr, highYCorr = highRadiusCorr*math.cos(highAngleCorr), highRadiusCorr*math.sin(highAngleCorr)
	goalCxCorr, goalCyCorr = (lowXCorr+highXCorr)/2, (lowYCorr+highYCorr)/2
	goalAngle = int(round(math.atan2(-goalCxCorr,-goalCyCorr) * 180/math.pi))
	goalRadiusCorr = int(round((goalCxCorr**2+goalCyCorr**2)**0.5))
	if crossFlag:
		goalRadiusIm = (goalCxCorr**2+goalCyCorr**2)**0.5
		goalAngleIm = math.atan2(goalCyCorr,goalCxCorr)
		goalXIm, goalYIm = goalRadiusIm*math.cos(goalAngleIm), goalRadiusIm*math.sin(goalAngleIm)
		img.draw_cross(round(goalXIm)+botCx, round(goalYIm)+botCy)
	return goalRadiusCorr, goalAngle
def blobAngleRadius(goalBlob, img, colorThresh):
	goalCx, goalCy, goalRect = getCentroid(goalBlob)
	_, _, rectW, rectH = goalRect
	if rectW >= rectH:
		lowMarkX, highMarkX = goalCx-rectW/2, goalCx+rectW/2
		lowMarkY, highMarkY = goalCy, goalCy
	else:
		lowMarkX, highMarkX = goalCx, goalCx
		lowMarkY, highMarkY = goalCy-rectH/2, goalCy+rectH/2
	if rectW >= rectH:
		lowSideBound, highSideBound = goalCx-rectW/2, goalCx+rectW/2
		lowSearchBound, highSearchBound = round(goalCy-rectH/2), min(round(goalCy+rectH/2)+1, 120)
		if goalCy <= botCy:
			searchRange = range(lowSearchBound, highSearchBound)
		else:
			searchRange = reversed(range(lowSearchBound, highSearchBound))
	else:
		lowSideBound, highSideBound = goalCy-rectH/2, goalCy+rectH/2
		lowSearchBound, highSearchBound = round(goalCx-rectW/2), min(round(goalCx+rectW/2)+1, 120)
		if goalCx <= botCx:
			searchRange = range(lowSearchBound, highSearchBound)
		else:
			searchRange = reversed(range(lowSearchBound, highSearchBound))
	lowColorThresh, highColorThresh = colorThresh[0], colorThresh[1]
	lowMarkDetected, highMarkDetected = False, False
	for i in searchRange:
		if not lowMarkDetected:
			if rectW >= rectH:
				pixelLow = image.rgb_to_lab(img.get_pixel(round(lowSideBound)+1, i))
			else:
				pixelLow = image.rgb_to_lab(img.get_pixel(i, round(lowSideBound)+1))
			if all(a >= b for a, b in zip(pixelLow, lowColorThresh)) and\
			all(a <= b for a, b in zip(pixelLow, highColorThresh)):
				if rectW >= rectH:
					lowMarkX, lowMarkY = lowSideBound+1, i
				else:
					lowMarkY, lowMarkX = lowSideBound+1, i
				lowMarkDetected = True
		if not highMarkDetected:
			if rectW >= rectH:
				pixelHigh = image.rgb_to_lab(img.get_pixel(round(highSideBound-1), i))
			else:
				pixelHigh = image.rgb_to_lab(img.get_pixel(i, round(highSideBound-1)))
			if all(a >= b for a, b in zip(pixelHigh, lowColorThresh)) and\
			all(a <= b for a, b in zip(pixelHigh, highColorThresh)):
				if rectW >= rectH:
					highMarkX, highMarkY = highSideBound-1, i
				else:
					highMarkY, highMarkX = highSideBound-1, i
				highMarkDetected = True
	goalRadiusCorr, goalAngle = endsToCentre(lowMarkX, lowMarkY, highMarkX, highMarkY)
	if crossFlag:
		img.draw_rectangle(goalRect)
		img.draw_cross(round(goalCx), round(goalCy))
		img.draw_line(round(lowMarkX), round(lowMarkY), round(highMarkX), round(highMarkY))
	return goalRadiusCorr, goalAngle, (lowMarkX, lowMarkY), (highMarkX, highMarkY)
def drawNearestGoal(angle):
	if angle <= 0:
		gradient = math.tan(angle/180*math.pi-math.pi/2)
		if gradient > 500:
			gradient = 500
		if gradient < -500:
			gradient = -500
		img.draw_line(botCx, botCy, 120, int(round((botCx-120)*gradient+botCy)))
	else:
		gradient = math.tan(angle/180*math.pi+math.pi/2)
		if gradient > 500:
			gradient = 500
		if gradient < -500:
			gradient = -500
		img.draw_line(botCx, botCy, 0, int(round((botCx-0)*gradient+botCy)))
def writeUART(radius, angle, objCode, target):
	if radius == False or angle == False:
		target.writechar(objCode)
	else:
		if angle <= 0:
			angle += 360
		angle = int((angle/360)*250)
		radius = int(radius)
		target.writechar(objCode)
		target.writechar(angle)
		target.writechar(min(radius,250))
def sortAngleRadius(goalRadiusEnd1, goalAngleEnd1, goalRadiusEnd2, goalAngleEnd2):
	if goalAngleEnd1 < 0:
		goalAngleEnd1Compare = goalAngleEnd1 + 360
	else:
		goalAngleEnd1Compare = goalAngleEnd1
	if goalAngleEnd2 < 0:
		goalAngleEnd2Compare = goalAngleEnd2 + 360
	else:
		goalAngleEnd2Compare = goalAngleEnd2
	if goalAngleEnd1Compare <= goalAngleEnd2Compare:
		return goalRadiusEnd1, goalAngleEnd1, goalRadiusEnd2, goalAngleEnd2
	elif goalAngleEnd2Compare < goalAngleEnd1Compare:
		return goalRadiusEnd2, goalAngleEnd2, goalRadiusEnd1, goalAngleEnd1
def detectGoal(colorThresh):
	goalBlobs = img.find_blobs(colorThresh, pixels_threshold=25, area_threshold=goalAreaThresh ,merge = True)
	goalBlobs = sorted(goalBlobs, key=lambda x: x.area())[:2]
	colorThreshZip = [(colorThresh[0][0], colorThresh[0][2], colorThresh[0][4]),\
					(colorThresh[0][1], colorThresh[0][3], colorThresh[0][5])]
	if not goalBlobs:
		return None, None
	elif len(goalBlobs) == 1:
		goalBlob = goalBlobs[0]
		goalRadius, goalAngle, lowMark, highMark = blobAngleRadius(goalBlob, img, colorThreshZip)
		goalRadiusEnd1, goalAngleEnd1 = getBotPolar(*lowMark)
		goalRadiusEnd2, goalAngleEnd2 = getBotPolar(*highMark)
		if crossFlag:
			drawNearestGoal(goalAngle)
		goalRadiusLow, goalAngleLow, goalRadiusHigh, goalAngleHigh = sortAngleRadius(goalRadiusEnd1, goalAngleEnd1, goalRadiusEnd2, goalAngleEnd2)
	else:
		goalBlob1, goalBlob2 = goalBlobs[0], goalBlobs[1]
		goalRadius1, goalAngle1, lowMark1, highMark1 = blobAngleRadius(goalBlob1, img, colorThreshZip)
		goalRadius2, goalAngle2, lowMark2, highMark2 = blobAngleRadius(goalBlob2, img, colorThreshZip)
		if goalRadius1 < goalRadius2:
			goalRadiusEnd1, goalAngleEnd1 = getBotPolar(*lowMark1)
			goalRadiusEnd2, goalAngleEnd2 = getBotPolar(*highMark1)
			if crossFlag:
				drawNearestGoal(goalAngle1)
		else:
			goalRadiusEnd1, goalAngleEnd1 = getBotPolar(*lowMark2)
			goalRadiusEnd2, goalAngleEnd2 = getBotPolar(*highMark2)
			if crossFlag:
				drawNearestGoal(goalAngle2)
		goalRadiusLow, goalAngleLow, goalRadiusHigh, goalAngleHigh = sortAngleRadius(goalRadiusEnd1, goalAngleEnd1, goalRadiusEnd2, goalAngleEnd2)
		xCoords = (lowMark1[0], lowMark2[0], highMark1[0], highMark2[0])
		yCoords = (lowMark1[1], lowMark2[1], highMark1[1], highMark2[1])
		xLen = max(xCoords) - min(xCoords)
		yLen = max(yCoords) - min(yCoords)
		if xLen >= yLen:
			minIndex, maxIndex = xCoords.index(min(xCoords)), xCoords.index(max(xCoords))
			lowMarkX, highMarkX = xCoords[minIndex], xCoords[maxIndex]
			lowMarkY, highMarkY = yCoords[minIndex], yCoords[maxIndex]
		else:
			minIndex, maxIndex = yCoords.index(min(yCoords)), yCoords.index(max(yCoords))
			lowMarkX, highMarkX = xCoords[minIndex], xCoords[maxIndex]
			lowMarkY, highMarkY = yCoords[minIndex], yCoords[maxIndex]
		goalRadius, goalAngle = endsToCentre(lowMarkX, lowMarkY, highMarkX, highMarkY)
	if colorThresh == blueThreshold:
		writeUART(goalRadius, goalAngle, 251, uart)
	elif colorThresh == yellowThreshold:
		writeUART(goalRadius, goalAngle, 252, uart)
	if goalDebugFlag:
		if len(goalBlobs) == 1:
			if colorThresh == yellowThreshold:
				print("B:", goalRadius, goalAngle)
			else:
				print("Y:", goalRadius, goalAngle)
		else:
			if colorThresh == blueThreshold:
				print("B1:", goalRadius1, goalAngle1, "\t", "B2:", goalRadius2, goalAngle2)
			else:
				print("Y1:", goalRadius1, goalAngle1, "\t", "Y2:", goalRadius2, goalAngle2)
	return goalRadius, goalAngle
def readBluetooth():
	state = 0
	while state < 4:
		if not hc.any():
			continue
		if state == 0:
			objCode = hc.readchar()
			if objCode >= 251 and objCode <=253:
				state += 1
				continue
		elif state == 1:
			angle = hc.readchar()
			state += 1
			continue
		elif state == 2:
			radius = hc.readchar()
			state += 1
			continue
		elif state == 3:
			sign = hc.readchar()
			state += 1
			continue
	if sign == 255:
		signedAngle = 360-angle
	elif sign == 254:
		signedAngle = angle
	else:
		signedAngle = 0
	return radius, signedAngle, objCode
if ledFlag:
	red_led = pyb.LED(1)
	green_led = pyb.LED(2)
	green_led.off()
	red_led.off()
if clockFlag:
	clock = time.clock()
hc = UART(1, 38400, timeout_char=100)
hc.init(38400, bits=8, parity=None, stop=1, timeout_char=100)
while True:
	img = sensor.snapshot()
	img.draw_ellipse(botCx, botCy+5, 58, 60, 0, 0, 0, 11, False)
	img.draw_ellipse(botCx, botCy, 33, 20, 0, 0, 0, 0, True)
	if crossFlag:
		img.draw_cross(botCx, botCy)
	if bGoalFlag:
		bGoalRadius, bGoalAngle = detectGoal(blueThreshold)
	if yGoalFlag:
		yGoalRadius, yGoalAngle = detectGoal(yellowThreshold)
	ballBlobs = img.find_blobs(orangeThreshold, pixels_threshold=2, area_threshold=1,merge = True)
	if ballBlobs:
		ballBlob = sorted(ballBlobs, key=lambda x: x.area(), reverse=True)[0]
		ballCx, ballCy  = ballBlob.cx(), ballBlob.cy()
		ballRadius, ballAngle = getBotPolar(ballCx, ballCy)
		print(ballAngle)
		writeUART(ballRadius, ballAngle, 253, uart)
		if ballDebugFlag:
			print(ballRadius, ballAngle)
		if crossFlag:
			img.draw_rectangle(ballBlob.rect(), color=(0,0,0))
			img.draw_cross(ballCx, ballCy, color=(0,0,0))
		if ledFlag:
			green_led.on()
	else:
		if ledFlag:
			green_led.off()
		writeUART(False, False, 255, uart)
	if clockFlag:
		print(clock.fps())
		clock.tick()
	if roleRecv:
		if hc.any():
			radiusOther, angleOther, objCodeOther = readBluetooth()
			timeLastTalked = pyb.millis()
			if bGoalFlag:
				if objCodeOther == 251 and bGoalRadius:
					if radiusOther <= bGoalRadius:
						uart.writechar(101)
						if roleDebug:
							print("striker")
					else:
						uart.writechar(100)
						if roleDebug:
							print("goalie")
			if yGoalFlag:
				if objCodeOther == 252 and yGoalRadius:
					if radiusOther <= yGoalRadius:
						uart.writechar(101)
						if roleDebug:
							print("striker")
					else:
						uart.writechar(100)
						if roleDebug:
							print("goalie")
		if pyb.millis()-timeLastTalked > 1000:
			otherBotStatus = False
		else:
			otherBotStatus = True
		if heartbeatDebug:
			if otherBotStatus:
				print("alive")
			else:
				print("dead")
