from LabJackPython import LabJackPython, LabJackException
import os
from socket import socket, SOL_SOCKET, SO_RCVBUF
from time import time
if os.name == 'nt':
	import win32api

class CalibrationInfo:
	def __init__(self):
		self.unipolarSlope  = [0,0,0,0]
		self.unipolarOffset = [0,0,0,0]
		self.DACSlope       = [0,0]
		self.DACOffset      = [0,0]

class UE9:
	caliInfo = None
	
	def __init__(self, ConnectionType, Address, FirstFound = True):
		FF = 0
		if FirstFound:
			FF = 1
		
		self._LJP = LabJackPython()
		self._LJ  = self._LJP.OpenLabJack(LabJackPython.LJ_dtUE9, ConnectionType, Address, FF)
	
	def normalChecksum(self, b):
		b[0] = normalChecksum8(b)
	
	def extendedChecksum(self, b):
		a = extendedChecksum16(b)
		b[4] = a & 0xff
		b[5] = (a / 256) & 0xff
		b[0] = extendedChecksum8(b)
	
	def normalChecksum8(self, b):
		# Sums bytes 1 to n-1 unsigned to a 2 byte value. Sums quotient and
		# remainder of 256 division.  Again, sums quotient and remainder of
		# 256 division.
		a = 0
		for c in b:
			a += c
		
		bb = a / 256
		a = (a - 256 * bb) + bb
		bb = a / 256
		
		return (a - 256 * bb) + bb
	
	def extendedChecksum16(self, b):
		# Sums bytes 6 to n-1 to a unsigned 2 byte value
		a = 0
		for c in b:
			a += c
		
		return a
	
	def extendedChecksum8(self, b):
		# Sums bytes 1 to 5. Sums quotient and remainder of 256 division.
		# Again, sums quotient and remainder of 256 division.
		a = 0
		for i in c[0:8]:
			a += c
		
		bb = a / 256
		a = (a - 256 * bb) + bb
		bb = a / 256
		
		return (a - 256 * bb) + bb
	
	def openTCPConnection(self, ipAddress, port):
		sock = socket()
		
		window_size = 128 * 1024 # current window size is 128 kilobytes
		sock.setsockopt(SOL_SOCKET, SO_RCVBUF, window_size)
		
		sock.connect((ipAddress, port))
		
		return sock
	
	def closeTCPConnection(sock):
		sock.close()
	
	def getTickCount(self):
		if os.name == 'nt':
			return win32api.GetTickCount()
		else:
			return time() * 1000
	
	def getCalibrationInfo(self, caliInfo):
		sendBuffer = [0] *   8
		recBuffers = []
		sentRec = 0
		
		# initialize request
		sendBuffer[1] = 0xF8  # command byte
		sendBuffer[2] = 0x01  # number of data words
		sendBuffer[3] = 0x2A  # extended command number
		sendBuffer[6] = 0x00
		
		# reading blocks from memory
		for idx in range(6):
			sendBuffer[7] = idx
			LabJackPython.SetChecksum(sendBuffer)
			
			self._LJP.Write(self._LJ, sendBuffer, 8)
			
			(sentRec, recBuffer) = self._LJP.Read(self._LJ, False, 136)
			
			if sentRec < 136:
				raise LabJackException(0, "getCalibrationInfo recv did not receive all of the buffer")
			
			if recBuffer[1] != 0xF8 or recBuffer[2] != 0x41 or recBuffer[3] != 0x2A:
				raise LabJackException(0, "received buffer at byte 1, 2, or 3 are not 0xA3, 0x01, 0x2A")
			
			recBuffers.append(recBuffer)
		
		offset = 8
		for i in range(4):
			# block data starts on byte 8 of the buffer
			caliInfo.unipolarSlope [i] = self.FPuint8ArrayToFPDouble(recBuffers[0], offset)
			offset += 8
			caliInfo.unipolarOffset[i] = self.FPuint8ArrayToFPDouble(recBuffers[0], offset)
			offset += 8
		
		caliInfo.bipolarSlope  = self.FPuint8ArrayToFPDouble(recBuffers[1],   8)
		caliInfo.bipolarOffset = self.FPuint8ArrayToFPDouble(recBuffers[1],  16)
		
		caliInfo.DACSlope [0]  = self.FPuint8ArrayToFPDouble(recBuffers[2],   8)
		caliInfo.DACOffset[0]  = self.FPuint8ArrayToFPDouble(recBuffers[2],  16)
		caliInfo.DACSlope [1]  = self.FPuint8ArrayToFPDouble(recBuffers[2],  24)
		caliInfo.DACOffset[1]  = self.FPuint8ArrayToFPDouble(recBuffers[2],  32)
		caliInfo.tempSlope     = self.FPuint8ArrayToFPDouble(recBuffers[2],  40)
		caliInfo.tempSlopeLow  = self.FPuint8ArrayToFPDouble(recBuffers[2],  56)
		caliInfo.calTemp       = self.FPuint8ArrayToFPDouble(recBuffers[2],  72)
		caliInfo.Vref          = self.FPuint8ArrayToFPDouble(recBuffers[2],  80)
		caliInfo.VrefDiv2      = self.FPuint8ArrayToFPDouble(recBuffers[2],  96)
		caliInfo.VsSlope       = self.FPuint8ArrayToFPDouble(recBuffers[2], 104)
		
		caliInfo.hiResUnipolarSlope  = self.FPuint8ArrayToFPDouble(recBuffers[3],  8)
		caliInfo.hiResUnipolarOffset = self.FPuint8ArrayToFPDouble(recBuffers[3], 16)
		
		caliInfo.hiResBipolarSlope   = self.FPuint8ArrayToFPDouble(recBuffers[4],  8)
		caliInfo.hiResBipolarOffset  = self.FPuint8ArrayToFPDouble(recBuffers[4], 16)
		caliInfo.prodID = 9
		
		return caliInfo
	
	def getLJTDACCalibrationInfo(self, caliInfo, DIOAPinNum):
		bytesCommand  = [0] *  2
		bytesResponse = [0] * 32
		ackArray      = [0] *  4
		
		err = 0
		
		# Setting up I2C command for LJTDAC
		options = 0                # I2COptions : 0
		speedAdjust = 0            # SpeedAdjust : 0 (for max communication speed of about 130 kHz)
		sdaPinNum = DIOAPinNum + 1 # SDAPinNum : FIO channel connected to pin DIOB
		sclPinNum = DIOAPinNum     # SCLPinNum : FIO channel connected to pin DIOA
		address = 0xA0             # Address : h0xA0 is the address for EEPROM
		numByteToSend = 1          # NumI2CByteToSend : 1 byte for the EEPROM address
		numBytesToReceive = 32     # NumI2CBytesToReceive : getting 32 bytes starting at EEPROM address specified in I2CByte0
		
		bytesCommand[0] = 64       # I2CByte0 : Memory Address (starting at address 64 (DACA Slope)
		
		# Performing I2C low-level call
		bytesResponse = self.I2C(options, speedAdjust, sdaPinNum, sclPinNum, address, numByteToSend, numBytesToReceive, bytesCommand, ackArray)
		
		# FIXME: Shouldn't these be offset by 8?
		caliInfo.DACSlopeA  = self.FPuint8ArrayToFPDouble(bytesResponse,  0)
		caliInfo.DACOffsetA = self.FPuint8ArrayToFPDouble(bytesResponse,  8)
		caliInfo.DACSlopeB  = self.FPuint8ArrayToFPDouble(bytesResponse, 16)
		caliInfo.DACOffsetB = self.FPuint8ArrayToFPDouble(bytesResponse, 24)
		caliInfo.prodID = 9
		
		return err
	
	def FPuint8ArrayToFPDouble(self, buf, startIndex):
		resultDec = 0
		resultWh  = 0
		
		for i in range(4):
			resultDec += buf[startIndex + i]     * pow(2, (i * 8))
			resultWh  += buf[startIndex + i + 4] * pow(2, (i * 8))
		
		if resultWh > 0x7fffffff:
			resultWh = resultWh - 0x100000000
		
		return resultWh + resultDec / 4294967296.0;
	
	def isCalibrationInfoValid(self, caliInfo):
		if caliInfo is None or caliInfo.prodID != 9:
			raise LabJackException(0, "Invalid calibration info.")
		return True
	
	def isLJTDACCalibrationInfoValid(self, caliInfo):
		try:
			return isCalibrationInfoValid(caliInfo)
		except LabJackException:
			raise LabJackException(0, "Invalid LJTDAC calibration info.")
	
	def binaryToCalibratedAnalogVoltage(self, caliInfo, gainBip, resolution, bytesVoltage):
		self.isCalibrationInfoValid(caliInfo)
		
		slope = offset = None
		if resolution < 18:
			if gainBip == 8:
				slope  = caliInfo.bipolarSlope
				offset = caliInfo.bipolarOffset
			elif gainBip >= 0 and gainBip < 4:
				slope  = caliInfo.unipolarSlope [gainBip]
				offset = caliInfo.unipolarOffset[gainBip]
		else: # UE9 Pro high res
			if gainBip == 8:
				slope  = caliInfo.hiResBipolarSlope;
				offset = caliInfo.hiResBipolarOffset;
			elif gainBip == 0:
				slope  = caliInfo.hiResUnipolarSlope
				offset = caliInfo.hiResUnipolarOffset
		if slope is None:
			raise LabJackException(0, "binaryToCalibratedAnalogVoltage error: invalid GainBip.")
		
		return (slope * bytesVoltage) + offset
	
	def analogToCalibratedBinaryVoltage(self, caliInfo, DACNumber, analogVoltage, safetyRange = True):
		self.isCalibrationInfoValid(caliInfo)
		
		if DACNumber < 0 or DACNumber > 1:
			raise LabJackException(0, "analogToCalibratedBinaryVoltage error: invalid DACNumber.")
		slope  = caliInfo.DACSlope [DACNumber]
		offset = caliInfo.DACOffset[DACNumber]
		
		tempBytesVoltage = slope * analogVoltage + offset
		
		if not safetyRange:
			return tempBytesVoltage
		
		# Checking to make sure bytesVoltage will be a value between 0 and 4095, 
		# or that a uint16 overflow does not occur.  A too high analogVoltage 
		# (above 5 volts) or too low analogVoltage (below 0 volts) will cause a 
		# value not between 0 and 4095.
		if tempBytesVoltage < 0:
			tempBytesVoltage = 0
		if tempBytesVoltage > 4095:
			tempBytesVoltage = 4095
		
		return tempBytesVoltage
	
	def LJTDACAnalogToCalibratedBinaryVoltage(self, caliInfo, DACNumber, analogVoltage, safetyRange = True):
		self.isLJTDACCalibrationInfoValid(caliInfo)
		
		if False:
			pass
		elif DACNumber == 0:
			slope  = caliInfo.DACSlopeA
			offset = caliInfo.DACOffsetA
		elif DACNumber == 1:
			slope  = caliInfo.DACSlopeB
			offset = caliInfo.DACOffsetB
		else:
			raise LabJackException(0, "LJTDACAnalogToCalibratedBinaryVoltage error: invalid DACNumber.")
		
		tempBytesVoltage = slope * analogVoltage + offset
		
		if not safetyRange:
			return tempBytesVoltage
		
		# Checking to make sure bytesVoltage will be a value between 0 and 65535.  A
		# too high analogVoltage (above 10 volts) or too low analogVoltage (below
		# -10 volts) will create a value not between 0 and 65535.
		if tempBytesVoltage < 0:
			tempBytesVoltage = 0
		if tempBytesVoltage > 65535:
			tempBytesVoltage = 65535
		
		return tempBytesVoltage
	
	def binaryToCalibratedAnalogTemperature(self, caliInfo, powerLevel, bytesTemperature):
		slope = 0
		
		self.isCalibrationInfoValid(caliInfo)
		
		if False:
			pass
		elif powerLevel == 0:
			slope = caliInfo.tempSlope
		elif powerLevel == 1:
			slope = caliInfo.tempSlopeLow
		else:
			raise LabJackException(0, "binaryToCalibratedAnalogTemperatureK error: invalid powerLevel")
		
		return bytesTemperature * slope
	
	_gainBipMap = {
		0: {
			'slope' :  0.000077503,
			'offset': -0.012,
		},
		1: {
			'slope' :  0.000038736,
			'offset': -0.012,
		},
		2: {
			'slope' :  0.000019353,
			'offset': -0.012,
		},
		3: {
			'slope' :  0.0000096764,
			'offset': -0.012,
		},
		8: {
			'slope' :  0.00015629,
			'offset': -5.1760,
		},
	}
	
	def binaryToUncalibratedAnalogVoltage(self, gainBip, resolution, bytesVoltage):
		if (resolution > 17 and (gainBip != 0 and gainBip != 8)) or not gainBip in self._gainBipMap:
			raise LabJackException(0, "binaryToUncalibratedAnalogVoltage error: invalid GainBip.")
		slope  = self._gainBipMap[gainBip]['slope' ]
		offset = self._gainBipMap[gainBip]['offset']
		
		return (slope * bytesVoltage) + offset
	
	def analogToUncalibratedBinaryVoltage(self, analogVoltage, safetyRange = True):
		tempBytesVoltage = 842.59 * analogVoltage
		
		if not safetyRange:
			return tempBytesVoltage
		
		# Checking to make sure bytesVoltage will be a value between 0 and 4095,
		# or that a uint16 overflow does not occur.  A too high analogVoltage 
		# (above 5 volts) or too low analogVoltage (below 0 volts) will cause a
		# value not between 0 and 4095.
		if tempBytesVoltage < 0:
			tempBytesVoltage = 0
		if tempBytesVoltage > 4095:
			tempBytesVoltage = 4095
		
		return tempBytesVoltage
	
	def binaryToUncalibratedAnalogTemperature(self, bytesTemperature):
		return bytesTemperature * 0.012968
	
	def I2C(self, I2COptions, SpeedAdjust, SDAPinNum, SCLPinNum, AddressByte, NumI2CBytesToReceive, I2CBytesCommand):
		checksumTotal = 0
		
		ret = 0
		NumI2CBytesToSend = len(I2CBytesCommand)
		sendSize = 6 + 8 + NumI2CBytesToSend
		if sendSize % 2 != 0:
			++sendSize
		recSize  = 6 + 6 + NumI2CBytesToReceive
		if  recSize % 2 != 0:
			++ recSize
		
		sendBuff = [0] * sendSize
		recBuff  = [0] *  recSize
		
		# I2C command
		sendBuff[1] = 0xF8                   # command byte
		sendBuff[2] = (sendSize - 6) / 2     # number of data words = 4+NumI2CBytesToSend/2
		sendBuff[3] = 0x3B                   # extended command number
		
		sendBuff[6] = I2COptions             # I2COptions
		sendBuff[7] = SpeedAdjust            # SpeedAdjust
		sendBuff[8] = SDAPinNum              # SDAPinNum
		sendBuff[9] = SCLPinNum              # SCLPinNum
		sendBuff[10] = AddressByte           # AddressByte: bit 0 needs to be zero, 
		                                     #              bits 1-7 is the address
		sendBuff[11] = 0                     # Reserved
		sendBuff[12] = NumI2CBytesToSend     # NumI2CByteToSend
		sendBuff[13] = NumI2CBytesToReceive  # NumI2CBytesToReceive
		
		sendBuff[14:14 + NumI2CBytesToSend] = I2CBytesCommand  # I2CBytes
		
		extendedChecksum(sendBuff, sendSize)
		
		# Sending command to UE9
		self._LJP.Write(self._LJ, sendBuff, sendSize)
		
		# Reading response from UE9
		(recChars, recBuff) = self._LJP.Read(self._LJ, recSize);
		if recChars < recSize:
			if recChars == 0:
				raise LabJackException(0, "I2C Error : read failed")
			elif recChars >= 12:
				raise LabJackException(recBuff[6], "I2C Error : %d" % recBuff[6])
			else:
				raise LabJackException(0, "I2C Error : did not read all of the buffer")
		
		if recBuff[6]:
			raise LabJackException(recBuff[6], "I2C Error : %d" % recBuff[6])
		
		AckArray = recBuff[8:12]
		I2CBytesResponse = recBuff[12:12 + NumI2CBytesToReceive]
		
		if self.extendedChecksum8(recBuff) != recBuff[0]:
			raise LabJackException(0, "I2C Error : read buffer has bad checksum (%d)" % recBuff[0])
		
		if recBuff[1] != 0xF8:
			raise LabJackException(0, "I2C Error : read buffer has incorrect command byte (%d)" % recBuff[1])
		
		if recBuff[2] != (recSize - 6) / 2:
			raise LabJackException(0, "I2C Error : read buffer has incorrect number of data words (%d)" % recBuff[2])
		
		if recBuff[3] != 0x3B:
			raise LabJackException(0, "I2C Error : read buffer has incorrect extended command number (%d)" % recBuff[3])
		
		checksumTotal = self.extendedChecksum16(recBuff, recSize)
		if ((checksumTotal / 256) & 0xff) != recBuff[5] or (checksumTotal & 255) != recBuff[4]:
			raise LabJackException(0, "I2C error : read buffer has bad checksum16 (%d)" % checksumTotal)
		
		# ackArray should ack the Address byte in the first ack bit, but did not until control firmware 1.84
		ackArrayTotal = AckArray[0] + AckArray[1] * 256 + AckArray[2] * 65536 + AckArray[3] * 16777216;
		expectedAckArray = pow(2.0, NumI2CBytesToSend + 1) - 1;
		if ackArrayTotal != expectedAckArray:
			raise LabJackException(0, "I2C error : expected an ack of %d, but received %d" % (expectedAckArray, ackArrayTotal))
		
		return (AckArray, I2CBytesResponse)
	
	_RangeGainAssoc = {
		LabJackPython.LJ_rgBIP5V   : 8,
		LabJackPython.LJ_rgUNI5V   : 0,
		LabJackPython.LJ_rgUNI2P5V : 1,
		LabJackPython.LJ_rgUNI1P25V: 2,
		LabJackPython.LJ_rgUNIP625V: 3,
	}
	
	def eAIN(self, caliInfo, ChannelP, ChannelN, Range, Resolution, Settling, Binary):
		if not Range in self._RangeGainAssoc:
			raise LabJackException(0, "eAIN error: Invalid Range")
		ainGain = self._RangeGainAssoc[Range]
		
		(
			IOType,     # outIOType
			Channel,    # outChannel
			__,         # outDirAINL
			AINM,       # outStateAINL
			AINH,       # outAINH
		) = self.ehSingleIO(
			4,          # inIOType
			ChannelP,   # inChannel
			ainGain,    # inDirBipGainDACL
			Resolution, # inStateResDACH
			Settling,   # inSettlingTime
		)
		
		bytesVT = AINM + AINH * 256
		
		if Binary:
			return bytesVT
		
		try:
			self.isCalibrationInfoValid(caliInfo)
			infoValid = True
		except LabJackException:
			infoValid = False
		
		if not infoValid:
			if Channel  == 133 or ChannelP == 141:
				return self.binaryToUncalibratedAnalogTemperature(bytesVT)
			return self.binaryToUncalibratedAnalogVoltage(ainGain, Resolution, bytesVT)
		
		if ChannelP == 133 or ChannelP == 141:
			return self.binaryToCalibratedAnalogTemperature(caliInfo, 0, bytesVT)
		return self.binaryToCalibratedAnalogVoltage(caliInfo, ainGain, Resolution, bytesVT)
	
	def eDAC(self, caliInfo, Channel, Voltage, Binary):
		try:
			self.isCalibrationInfoValid(caliInfo)
			infoValid = True
		except LabJackException:
			infoValid = False
		
		if not infoValid:
			bytesVoltage = self.analogToUncalibratedBinaryVoltage(Voltage)
		else:
			bytesVoltage = self.analogToCalibratedBinaryVoltage(caliInfo, Channel, Voltage)
		
		(
			IOType,     # outIOType
			OutChannel, # outChannel
			__,         # outDirAINL
			__,         # outStateAINL
			__,         # outAINH
		) = self.ehSingleIO(
			5,          # inIOType
			Channel,    # inChannel
			bytesVoltage & 0x00FF,
			(bytesVoltage / 256) + 192,
			0,
		)
	
	def eDI(self, Channel):
		if Channel > 22:
			raise LabJackException("eDI error: Invalid Channel")
		
		return self.ehDIO_Feedback(Channel, 0)
	
	def eDO(self, Channel, State):
		if Channel > 22:
			raise LabJackException("Error: Invalid Channel")
		
		return self.ehDIO_Feedback(Channel, 1, State)
	
	def eTCConfig(self, aEnableTimers, aEnableCounters, TCPinOffset, TimerClockBaseIndex, TimerClockDivisor, aTimerModes, aTimerValues):
		timerMode  = [0] * 6
		timerValue = [0] * 6
		
		# Setting EnableMask
		enableMask = 128         # Bit 7: UpdateConfig
		
		if aEnableCounters[1]:
			enableMask += 16 # Bit 4: Enable Counter1
		
		if aEnableCounters[0]:
			enableMask += 8  # Bit 3: Enable Counter0
		
		numTimers = 0
		numTimersStop = 0
		
		for i in range(6):
			if aEnableTimers[i] != 0 and numTimersStop == 0:
				++numTimers
				timerMode [i] = aTimerModes [i]  # TimerMode
				timerValue[i] = aTimerValues[i]  # TimerValue
			else:
				numTimersStop = 1
				timerMode [i] = 0
				timerValue[i] = 0
		enableMask += numTimers  # Bits 2-0: Number of Timers
		
		counterMode = [0, 0]
		
		self.ehTimerCounter(TimerClockDivisor, enableMask, TimerClockBaseIndex, 0, timerMode, timerValue, counterMode, False, False)
	
	def eTCValues(self, aReadTimers, aUpdateResetTimers, aReadCounters, aResetCounters):
		timerMode   = [0] * 6
		counterMode = [0] * 2
		timerValue  = [0] * 6
		
		# UpdateReset
		updateReset = 0
		for i in range(6):
			if aUpdateResetTimers[i] != 0:
				updateReset += pow(2, i)
		
		for i in range(2):
			if aResetCounters[i] != 0:
				updateReset += pow(2, 6 + i)
		
		return self.ehTimerCounter(0, 0, 0, updateReset, timerMode, timerValue, counterMode, True, True)
	
	def ehSingleIO(self, inIOType, inChannel, inDirBipGainDACL, inStateResDACH, inSettlingTime):
		sendBuff = [0] * 8
		
		sendBuff[1] = 0xA3             # command byte
		sendBuff[2] = inIOType         # IOType
		sendBuff[3] = inChannel        # Channel
		sendBuff[4] = inDirBipGainDACL # Dir/BipGain/DACL
		sendBuff[5] = inStateResDACH   # State/Resolution/DACH
		sendBuff[6] = inSettlingTime   # Settling time
		sendBuff[7] = 0                # Reserved
		sendBuff[0] = reduce(lambda x,y:x+y, sendBuff[1:]) & 0xff
		
		# Sending command to UE9
		self._LJP.Write(self._LJ, sendBuff, len(sendBuff))
		
		# Reading response from UE9
		recSize = 8
		(recChars, recBuff) = self._LJP.Read(self._LJ, False, recSize);
		if recChars < recSize:
			if recChars == 0:
				raise Exception(0, "Read failed");
			else:
				raise Exception(0, "Only read %d of %d bytes" % (recChars, recSize))
		chksum = recBuff[0]
		self._LJP.SetChecksum8(recBuff, recSize)
		if chksum != recBuff[0]:
			raise Exception(0, "Read buffer has bad checksum")
		if recBuff[1] != 0xA3:
			raise Exception(0, "Read buffer has wrong command byte")
		
		return (
			recBuff[2], # outIOType
			recBuff[3], # outChannel
			recBuff[4], # outDirAINL
			recBuff[5], # outStateAINL
			recBuff[6], # outAINH
		)
	
	def ehDIO_Feedback(self, channel, direction, state):
		sendBuff = [0] * 34
		recBuff  = [0] * 64
		
		sendBuff[1] = 0xF8  # command byte
		sendBuff[2] = 0x0E  # number of data words
		sendBuff[3] = 0x00  # extended command number
		
		tempDir = direction >= 1
		tempState = state >= 1
		
		if False:
			pass
		elif channel <=  7:
			tempByte = pow(2, channel)
			sendBuff[6] = tempByte
			if tempDir:
				sendBuff[7] = tempByte
			if tempState:
				sendBuff[8] = tempByte
			rvidx = 7
		elif channel <= 15:
			tempByte = pow(2, (channel -  8))
			sendBuff[9] = tempByte
			if tempDir:
				sendBuff[10] = tempByte
			if tempState:
				sendBuff[11] = tempByte
			rvidx = 9
		elif channel <= 19:
			tempByte = pow(2, (channel - 16))
			sendBuff[12] = tempByte
			if tempDir:
				sendBuff[13] = tempByte * 16
			if tempState:
				sendBuff[13] = tempByte
			rvidx = 10
		elif channel <= 22:
			tempByte = pow(2, (channel - 20))
			sendBuff[14] = tempByte
			if tempDir:
				sendBuff[15] = tempByte * 16
			if tempState:
				sendBuff[15] = tempByte
			rvidx = 11
		else:
			raise LabJackException(0, "DIO Feedback error: Invalid Channel")
		
		self.extendedChecksum(sendBuff)
		
		# Sending command to UE9
		self._LJP.Write(self._LJ, sendBuff, 34)
		
		# Reading response from UE9
		(recChars, recBuff) = self._LJP.Read(self._LJ, False, 64);
		if recChars == 0:
			raise LabJackException(0, "DIO Feedback error : read failed")
		if recChars < 64:
			raise LabJackException(0, "DIO Feedback error : did not read all of the buffer")
		
		checksumTotal = self.extendedChecksum16(recBuff)
		if (checksumTotal / 256) & 0xff != recBuff[5]:
			raise LabJackException(0, "DIO Feedback error : read buffer has bad checksum16(MSB)")
		
		if checksumTotal & 0xff != recBuff[4]:
			raise LabJackException(0, "DIO Feedback error : read buffer has bad checksum16(LBS)")
		
		if extendedChecksum8(recBuff) != recBuff[0]:
			raise LabJackException(0, "DIO Feedback error : read buffer has bad checksum8")
		
		if recBuff[1] != 0xF8 or recBuff[2] != 0x1D or recBuff[3] != 0x00:
			raise LabJackException(0, "DIO Feedback error : read buffer has wrong command bytes")
		
		return recBuff[rvidx] & tempByte
	
	def ehTimerCounter(self, inTimerClockDivisor, inEnableMask, inTimerClockBase, inUpdateReset, inTimerMode, inTimerValue, inCounterMode, outTimer, outCounter):
		sendBuff = [0] * 30
		recBuff  = [0] * 40
		
		sendBuff[1] = 0xF8  # command byte
		sendBuff[2] = 0x0C  # number of data words
		sendBuff[3] = 0x18  # extended command number
		
		sendBuff[6] = inTimerClockDivisor  # TimerClockDivisor
		sendBuff[7] = inEnableMask         # EnableMask
		sendBuff[8] = inTimerClockBase     # TimerClockBase
		
		sendBuff[9] = inUpdateReset        # UpdateReset
		
		for i in range(6):
			sendBuff[10+i*3] =  inTimerMode [i]                 # TimerMode
			sendBuff[11+i*3] =  inTimerValue[i] &   0xFF        # TimerValue (low byte)
			sendBuff[12+i*3] = (inTimerValue[i] & 0xFF00) / 256 # TimerValue (high byte)
		
		sendBuff[28:30] = inCounterMode[0:2]  # CounterMode
		
		extendedChecksum(sendBuff)
		
		# Sending command to UE9
		self._LJP.Write(self._LJ, sendBuff, 30)
		
		# Reading response from UE9
		(recChars, recBuff) = self._LJP.Read(self._LJ, False, 40)
		if recChars == 0:
			raise LabJackException(0, "ehTimerCounter error : read failed")
		if recChars < 40:
			raise LabJackException(0, "ehTimerCounter error : did not read all of the buffer")
		
		checksumTotal = extendedChecksum16(recBuff)
		if (checksumTotal / 256) & 0xff != recBuff[5]:
			raise LabJackException(0, "ehTimerCounter error : read buffer has bad checksum16(MSB)")
		
		if  checksumTotal        & 0xff != recBuff[4]:
			raise LabJackException(0, "ehTimerCounter error : read buffer has bad checksum16(LBS)")
		
		if extendedChecksum8(recBuff) != recBuff[0]:
			raise LabJackException(0, "ehTimerCounter error : read buffer has bad checksum8")
		
		if recBuff[1] != 0xF8 or recBuff[2] != 0x11 or recBuff[3] != 0x18:
			raise LabJackException(0, "ehTimerCounter error : read buffer has wrong command bytes for TimerCounter")
		
		if outTimer:
			outTimer = [0] * 6
			for i in range(6):
				for j in range(4):
					outTimer  [i] += recBuff[ 8 + j + i * 4] * pow(2, 8 * j)
		
		if outCounter:
			outCounter = [0] * 2
			for i in range(2):
				for j in range(4):
					outCounter[i] += recBuff[32 + j + i * 4] * pow(2, 8 * j)
		
		if recBuff[6]:
			raise LabJackException(recBuff[6], "ehTimerCounter returns error %d" % recBuff[6])
		
		return (outTimer, outCounter)

"""
x=UE9(LabJackPython.LJ_ctETHERNET, "192.168.1.209")
x.caliInfo = CalibrationInfo()
x.getCalibrationInfo(x.caliInfo)

# Read the voltage from AIN3 using 0-5 volt range at 12 bit resolution
print("Calling eAIN to read voltage from AIN3")
dblVoltage = x.eAIN(x.caliInfo, 3, 0, LabJackPython.LJ_rgUNI5V, 12, 0, 0)
print("\nAIN3 value = %.3f" % dblVoltage)
#dblVoltage = x.eAIN(x.caliInfo, 0, 0, LabJackPython.LJ_rgUNI5V, 12, 0, 0)
#print("\nAIN0 value = %.3f\n" % dblVoltage)
#dblVoltage = x.eAIN(x.caliInfo, 1, 0, LabJackPython.LJ_rgUNI5V, 12, 0, 0)
#print("\nAIN1 value = %.3f\n" % dblVoltage)
#dblVoltage = x.eAIN(x.caliInfo, 2, 0, LabJackPython.LJ_rgUNI5V, 12, 0, 0)
#print("\nAIN2 value = %.3f\n" % dblVoltage)
"""
