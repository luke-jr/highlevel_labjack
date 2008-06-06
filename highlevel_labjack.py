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
		self.hvAINSlope     = [0] * 4
		self.hvAINOffset    = [0] * 4

class _common:
	caliInfo = None
	
	def __init__(self, ConnectionType, Address, FirstFound = True):
		self.reinit(ConnectionType, Address, FirstFound)
	
	def reinit(self, ConnectionType = None, Address = None, FirstFound = None):
		if ConnectionType is None: ConnectionType = self.ConnectionType
		if Address        is None: Address        = self.Address
		if FirstFound     is None: FirstFound     = self.FirstFound
		
		self.ConnectionType = ConnectionType
		self.Address = Address
		self.FirstFound = FirstFound
		
		FF = 0
		if FirstFound:
			FF = 1
		
		self._LJP = LabJackPython()
		self._LJ  = self._LJP.OpenLabJack(self._type, ConnectionType, Address, FF)
	
	def normalChecksum(self, b):
		b[0] = self.normalChecksum8(b)
	
	def extendedChecksum(self, b):
		a = self.extendedChecksum16(b)
		b[4] = a & 0xff;
		b[5] = (a / 256) & 0xff
		b[0] = self.extendedChecksum8(b)
	
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
		for c in b[6:]:
			a += c
		
		return a
	
	def extendedChecksum8(self, b):
		# Sums bytes 1 to 5. Sums quotient and remainder of 256 division.
		# Again, sums quotient and remainder of 256 division.
		a = 0
		for i in b[1:6]:
			a += i
		
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
		caliInfo.DACSlope [0] = self.FPuint8ArrayToFPDouble(bytesResponse,  0)
		caliInfo.DACOffset[0] = self.FPuint8ArrayToFPDouble(bytesResponse,  8)
		caliInfo.DACSlope [1] = self.FPuint8ArrayToFPDouble(bytesResponse, 16)
		caliInfo.DACOffset[1] = self.FPuint8ArrayToFPDouble(bytesResponse, 24)
		caliInfo.prodID = self.prodID
		
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
		if caliInfo is None or caliInfo.prodID != self.prodID:
			raise LabJackException(0, "Invalid calibration info.")
		return True
	
	def isLJTDACCalibrationInfoValid(self, caliInfo):
		try:
			return isCalibrationInfoValid(caliInfo)
		except LabJackException:
			raise LabJackException(0, "Invalid LJTDAC calibration info.")
	
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
		# HACKY:
		if tempBytesVoltage > 4095 and caliInfo.hardwareVersion is None:
			tempBytesVoltage = 4095
		if tempBytesVoltage > 255 and caliInfo.hardwareVersion < 1.3:
			tempBytesVoltage = 255
		
		return tempBytesVoltage
	
	def LJTDACAnalogToCalibratedBinaryVoltage(self, caliInfo, DACNumber, analogVoltage, safetyRange = True):
		self.isLJTDACCalibrationInfoValid(caliInfo)
		
		if DACNumber < 0 or DACNumber > 1:
			raise LabJackException(0, "LJTDACAnalogToCalibratedBinaryVoltage error: invalid DACNumber.")
		
		slope  = caliInfo.DACSlope [DACNumber]
		offset = caliInfo.DACOffset[DACNumber]
		
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
		elif powerLevel == 1 and not caliInfo.tempSlopeLow is None:
			slope = caliInfo.tempSlopeLow
		else:
			raise LabJackException(0, "binaryToCalibratedAnalogTemperatureK error: invalid powerLevel")
		
		return bytesTemperature * slope
	
	_uncali_mult = 842.59
	_uncali_max = 4095
	def analogToUncalibratedBinaryVoltage(self, analogVoltage, safetyRange = True):
		tempBytesVoltage = _uncali_mult * analogVoltage
		
		if not safetyRange:
			return tempBytesVoltage
		
		# Checking to make sure bytesVoltage will be a value between 0 and 4095,
		# or that a uint16 overflow does not occur.  A too high analogVoltage 
		# (above 5 volts) or too low analogVoltage (below 0 volts) will cause a
		# value not between 0 and 4095.
		if tempBytesVoltage < 0:
			tempBytesVoltage = 0
		if tempBytesVoltage > _uncali_max:
			tempBytesVoltage = _uncali_max
		
		return tempBytesVoltage
	
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
	
	def eDI(self, Channel):
		return self.ehDIO_Feedback(Channel)
	
	def eDO(self, Channel, State):
		return self.ehDIO_Feedback(Channel, State)

class UE9(_common):
	_type = LabJackPython.LJ_dtUE9
	prodID = 9
	
	def getCalibrationInfo(self, caliInfo = True):
		if caliInfo is True:
			if self.caliInfo is None:
				self.caliInfo = CalibrationInfo()
			caliInfo = self.caliInfo
		if caliInfo is None:
			caliInfo = CalibrationInfo()
		
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
				raise LabJackException(0, "getCalibrationInfo received wrong command bytes for ReadMem")
			
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
		caliInfo.prodID = self.prodID
		
		return caliInfo
	
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
	
	def binaryToUncalibratedAnalogTemperature(self, bytesTemperature):
		return bytesTemperature * 0.012968
	
	_RangeGainAssoc = {
		LabJackPython.LJ_rgBIP5V   : 8,
		LabJackPython.LJ_rgUNI5V   : 0,
		LabJackPython.LJ_rgUNI2P5V : 1,
		LabJackPython.LJ_rgUNI1P25V: 2,
		LabJackPython.LJ_rgUNIP625V: 3,
	}
	
	def eAIN(self, ChannelP, ChannelN, Range, Resolution, Settling, Binary, caliInfo = True):
		if caliInfo is True:
			caliInfo = self.caliInfo
		
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
	
	def ehDIO_Feedback(self, channel, newValue = None):
		if channel < 0 or channel > 22:
			raise LabJackException(0, "DIO Feedback error: Invalid Channel")
		
		sendBuff = [0] * 34
		recBuff  = [0] * 64
		
		sendBuff[1] = 0xF8  # command byte
		sendBuff[2] = 0x0E  # number of data words
		sendBuff[3] = 0x00  # extended command number
		
		tempOffset = 6 + ((channel / 8) * 3)
		tempByte = pow(2, channel % 8)
		sendBuff[tempOffset] = tempByte
		rvidx = {
			 6:  7,
			 9:  9,
			12: 10,
			14: 11,
		}[tempOffset]
		
		if not newValue is None:
			sendBuff[tempOffset + 1] = tempByte
			if newValue:
				sendBuff[tempOffset + 2] = tempByte
		
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
		
		if self.extendedChecksum8(recBuff) != recBuff[0]:
			raise LabJackException(0, "DIO Feedback error : read buffer has bad checksum8")
		
		if recBuff[1] != 0xF8 or recBuff[2] != 0x1D or recBuff[3] != 0x00:
			raise LabJackException(0, "DIO Feedback error : read buffer has wrong command bytes")
		
		return recBuff[rvidx] & tempByte
	
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
				raise LabJackException(0, "Read failed");
			else:
				raise LabJackException(0, "Only read %d of %d bytes" % (recChars, recSize))
		chksum = recBuff[0]
		self._LJP.SetChecksum8(recBuff, recSize)
		if chksum != recBuff[0]:
			raise LabJackException(0, "Read buffer has bad checksum")
		if recBuff[1] != 0xA3:
			raise LabJackException(0, "Read buffer has wrong command byte")
		
		return (
			recBuff[2], # outIOType
			recBuff[3], # outChannel
			recBuff[4], # outDirAINL
			recBuff[5], # outStateAINL
			recBuff[6], # outAINH
		)
	
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
		
		checksumTotal = self.extendedChecksum16(recBuff)
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

class U3(_common):
	_type = LabJackPython.LJ_dtU3
	prodID = 3
	
	def getCalibrationInfo(self, caliInfo = True):
		if caliInfo is True:
			if self.caliInfo is None:
				self.caliInfo = CalibrationInfo()
			caliInfo = self.caliInfo
		if caliInfo is None:
			caliInfo = CalibrationInfo()
		
		sendBuffer = [0] *   8
		recBuffers = []
		cU3SendBuffer = [0] * 26
		cU3RecBuffer = [0] * 38
		sentRec = 0
		i = 0
		
		# sending ConfigU3 command to get hardware version and see if HV
		cU3SendBuffer[1] = 0xF8  # command byte
		cU3SendBuffer[2] = 0x0A  # number of data words
		cU3SendBuffer[3] = 0x08  # extended command number
		
		self.extendedChecksum(cU3SendBuffer)
		
		self._LJP.Write(self._LJ, cU3SendBuffer, 26)
		
		(sentRec, cU3RecBuffer) = self._LJP.Read(self._LJ, False, 38)
		if sentRec < 38:
			raise LabJackException(0, "getCalibrationInfo recv did not receive all of the buffer")
		
		if cU3RecBuffer[1] != 0xF8 or cU3RecBuffer[2] != 0x10 or cU3RecBuffer[3] != 0x08:
			raise LabJackException(0, "getCalibrationInfo received wrong command bytes for ConfigU3")
		
		caliInfo.hardwareVersion = cU3RecBuffer[14] + cU3RecBuffer[13] / 100.
		caliInfo.highVoltage = 0
		if cU3RecBuffer[37] & 18 == 18:
			caliInfo.highVoltage = 1
		
		# initialize request
		sendBuffer[1] = 0xF8  # command byte
		sendBuffer[2] = 0x01  # number of data words
		sendBuffer[3] = 0x2D  # extended command number
		sendBuffer[6] = 0x00
		
		# reading blocks from memory
		for idx in range(6):
			sendBuffer[7] = idx
			LabJackPython.SetChecksum(sendBuffer)
			
			self._LJP.Write(self._LJ, sendBuffer, 8)
			
			(sentRec, recBuffer) = self._LJP.Read(self._LJ, False, 40)
			
			if sentRec < 40:
				raise LabJackException(0, "getCalibrationInfo recv did not receive all of the buffer")
			
			if recBuffer[1] != 0xF8 or recBuffer[2] != 0x11 or recBuffer[3] != 0x2D:
				raise LabJackException(0, "getCalibrationInfo received wrong command bytes for ReadMem")
			
			recBuffers.append(recBuffer)
		
		# block data starts on byte 8 of the buffer
		caliInfo.ainSESlope    = self.FPuint8ArrayToFPDouble(recBuffers[0],   0)
		caliInfo.ainSEOffset   = self.FPuint8ArrayToFPDouble(recBuffers[0],   8)
		caliInfo.ainDiffSlope  = self.FPuint8ArrayToFPDouble(recBuffers[0],  16)
		caliInfo.ainDiffOffset = self.FPuint8ArrayToFPDouble(recBuffers[0],  24)
		
		caliInfo.DACSlope [0]  = self.FPuint8ArrayToFPDouble(recBuffers[1],   8)
		caliInfo.DACOffset[0]  = self.FPuint8ArrayToFPDouble(recBuffers[1],  16)
		caliInfo.DACSlope [1]  = self.FPuint8ArrayToFPDouble(recBuffers[1],  24)
		caliInfo.DACOffset[1]  = self.FPuint8ArrayToFPDouble(recBuffers[1],  32)
		
		caliInfo.tempSlope     = self.FPuint8ArrayToFPDouble(recBuffers[2],   0)
		caliInfo.Vref          = self.FPuint8ArrayToFPDouble(recBuffers[2],   8)
		caliInfo.Vref15        = self.FPuint8ArrayToFPDouble(recBuffers[2],  16)
		caliInfo.Vreg          = self.FPuint8ArrayToFPDouble(recBuffers[2],  24)
		
		for i in range(4):
			caliInfo.hvAINSlope[i] = self.FPuint8ArrayToFPDouble(recBuffers[3],  i * 8)
		
		for i in range(4):
			caliInfo.hvAINOffset[i] = self.FPuint8ArrayToFPDouble(recBuffers[4],  i * 8)
		
		caliInfo.prodID = self.prodID
		
		return caliInfo
	
	def binaryToCalibratedAnalogVoltage(self, caliInfo, dacEnabled, negChannel, bytesVoltage):
		self.isCalibrationInfoValid(caliInfo)
		
		analogVoltage = None
		if caliInfo.hardwareVersion >= 1.3:
			if caliInfo.highVoltage == 1:
				raise LabJackException(0, "binaryToCalibratedAnalogVoltage error: cannot handle U3-HV device.  Please use binaryToCalibratedAnalogVoltage_hw130 function.")
			else:
				return binaryToCalibratedAnalogVoltage_hw130(caliInfo, 0, negChannel, bytesVoltage, analogVoltage)
		
		if (negChannel >= 0 and negChannel <= 15) or negChannel == 30:
			if dacEnabled == 0:
				analogVoltage = caliInfo.ainDiffSlope * bytesVoltage + caliInfo.ainDiffOffset
			else:
				analogVoltage = (bytesVoltage / 65536.) * caliInfo.vreg * 2. - caliInfo.vreg
		elif negChannel == 31:
			if dacEnabled == 0:
				analogVoltage = caliInfo.ainSESlope * bytesVoltage + caliInfo.ainSEOffset
			else:
				analogVoltage = (bytesVoltage / 65536.) * caliInfo.vreg
		else:
			raise LabJackException(0, "binaryToCalibratedAnalogVoltage error: invalid negative channel.")
		
		return analogVoltage
	
	def analogToCalibratedBinary8BitVoltage(self, caliInfo, DACNumber, analogVoltage, safetyRange = True):
		return self.analogToCalibratedBinaryVoltage(caliInfo, DACNumber, analogVoltage, safetyRange)
	
	def analogToCalibratedBinary16BitVoltage(self, caliInfo, DACNumber, analogVoltage, safetyRange = True):
		self.isCalibrationInfoValid(caliInfo)
		
		if DACNumber < 0 or DACNumber > 1:
			raise LabJackException(0, "analogToCalibratedBinaryVoltage error: invalid DACNumber.")
		slope  = caliInfo.DACSlope [DACNumber]
		offset = caliInfo.DACOffset[DACNumber]
		
		if caliInfo.hardwareVersion >= 1.3:
			slope *= 256
			offset *= 256
		
		tempBytesVoltage = slope * analogVoltage + offset
		
		if not safetyRange:
			return tempBytesVoltage
		
		# Checking to make sure bytesVoltage will be a value between 0 and 255/65535.  Too
		# high of an analogVoltage (about 4.95 and above volts) or too low (below 0
		# volts) will cause a value not between 0 and 255/65535.
		if tempBytesVoltage < 0:
			tempBytesVoltage = 0
		if tempBytesVoltage > 65535 and caliInfo.hardwareVersion >= 1.30:
			tempBytesVoltage = 65535
		elif tempBytesVoltage > 255 and caliInfo.hardwareVersion <  1.30:
			tempBytesVoltage = 255
		
		return tempBytesVoltage
	
	def binaryToUncalibratedAnalogVoltage(self, dac1Enabled, negChannel, bytesVoltage):
		if (negChannel >= 0 and negChannel <= 15) or negChannel == 30:
			if dac1Enabled == 0:
				analogVoltage = bytesVoltage * 0.000074463 - 2.44
			else:
				analogVoltage = bytesVoltage / 65536. * 6.6 - 3.3
		elif negChannel == 31:
			if dac1Enabled == 0:
				analogVoltage = bytesVoltage * 0.000037231
			else:
				analogVoltage = bytesVoltage / 65536. * 3.3
		else:
			raise LabJackException(0, "binaryToCalibratedAnalogVoltage error: invalid negative channel.")
		
		return analogVoltage
	
	def analogToUncalibratedBinaryVoltage(self, analogVoltage, safetyRange = True):
		return self.analogToUncalibratedBinary8BitVoltage(caliInfo, analogVoltage, safetyRange)
	
	def analogToUncalibratedBinary8BitVoltage(self, analogVoltage, safetyRange = True):
		self._uncali_mult = 51.717
		self._uncali_max = 255
		return _common.analogToUncalibratedBinaryVoltage(self, caliInfo, analogVoltage, safetyRange)
	
	def analogToUncalibratedBinary16BitVoltage(self, analogVoltage, safetyRange = True):
		self._uncali_mult = 51.717 * 256
		self._uncali_max = 65535
		return _common.analogToUncalibratedBinaryVoltage(self, caliInfo, analogVoltage, safetyRange)
	
	def binaryToUncalibratedAnalogTemperature(self, bytesTemperature):
		return bytesTemperature * 0.013021
	
	#TODO: new output DAC1Enable
	# NOTE: moved ConfigIO param to the end optional for UE9 compat
	def eAIN(self, ChannelP, ChannelN, Range, Resolution, inSettling, Binary, caliInfo = True, ConfigIO = False):
		if caliInfo is True:
			caliInfo = self.caliInfo
		
		self.isCalibrationInfoValid(caliInfo)
		
		hwver = caliInfo.hardwareVersion
		hv = caliInfo.highVoltage
		
		if ChannelP < 0 or (ChannelP > 15 and ChannelP != 30 and ChannelP != 31):
			raise LabJackException(0, "eAIN error: Invalid positive channel")
		
		if ChannelN < 0 or (ChannelN > 15 and ChannelN != 30 and ChannelN != 31) or (hwver >= 1.3 and hv == 1 and ((ChannelP < 4 and ChannelN != 31) or ChannelN < 4)):
			raise LabJackException(0, "eAIN error: Invalid negative channel")
		
		if hwver >= 1.3 and hv == 1 and ChannelP < 4:
			pass
		elif ConfigIO:
			FIOAnalog = 0
			EIOAnalog = 0
			
			# Setting ChannelP and ChannelN channels to analog using
			# FIOAnalog and EIOAnalog
			if ChannelP <= 7:
				FIOAnalog = pow(2, ChannelP)
			elif ChannelP <= 15:
				EIOAnalog = pow(2, (ChannelP - 8))
			
			if ChannelN <= 7:
				FIOAnalog = FIOAnalog | pow(2, ChannelN)
			elif ChannelN <= 15:
				EIOAnalog = EIOAnalog | pow(2, (ChannelN - 8))
			
			# Using ConfigIO to get current FIOAnalog and EIOAnalog settings
			(
				curTCConfig,   # outTimerCounterConfig
				outDAC1Enalbe, # outDAC1Enable
				curFIOAnalog,  # outFIOAnalog
				curEIOAnalog,  # outEIOAnalog
			) = self.ehConfigIO(
				0,             # inWriteMask
				0,             # inTimerCounterConfig
				0,             # inDAC1Enable
				0,             # inFIOAnalog
				0,             # inEIOAnalog
			)
			
			DAC1Enable = outDAC1Enable
			
			if not (FIOAnalog == curFIOAnalog and EIOAnalog == curEIOAnalog):
				# Creating new FIOAnalog and EIOAnalog settings
				FIOAnalog = FIOAnalog | curFIOAnalog
				EIOAnalog = EIOAnalog | curEIOAnalog
				
				# Using ConfigIO to set new FIOAnalog and EIOAnalog settings
				(
					__,           # outTimerCounterConfig
					__,           # outDAC1Enable
					curFIOAnalog, # outFIOAnalog
					curEIOAnalog, # outEIOAnalog
				) = ehConfigIO(
					12,           # inWriteMask
					curTCConfig,  # inTimerCounterConfig
					0,            # inDAC1Enable
					FIOAnalog,    # inFIOAnalog
					EIOAnalog,    # inEIOAnalog
				)
		
		# Setting up Feedback command to read analog input
		sendDataBuff = [0] * 3
		sendDataBuff[0] = 1;    # IOType is AIN
		
		settling = 0
		if inSettling: settling = 1
		quicksample = 0
		if Resolution: quicksample = 1
		sendDataBuff[1] = ChannelP + settling * 64 + quicksample * 128  # Positive channel (bits 0-4), LongSettling (bit 6)
		                                                                # QuickSample (bit 7)
		sendDataBuff[2] = ChannelN   # Negative channel
		
		recDataBuff = self.ehFeedback(
			sendDataBuff, # inIOTypesDataBuff
			2,            # outDataSize
		)
		
		bytesVT = recDataBuff[0] + recDataBuff[1] * 256
		
		if Binary:
			return bytesVT
		
		if ChannelP == 30:
			return self.binaryToCalibratedAnalogTemperature(caliInfo, bytesVT)
		if hwver < 1.3:
			return self.binaryToCalibratedAnalogVoltage(CalibrationInfo, DAC1Enable, ChannelN, bytesVT)
		return self.binaryToCalibratedAnalogVoltage(CalibrationInfo, ChannelP, ChannelN, bytesVT)
	
	def eDAC(self, caliInfo, Channel, Voltage, Binary, ConfigIO = False):
		self.isCalibrationInfoValid(caliInfo)
		
		if Channel < 0 or Channel > 1:
			raise LabJackException(0, "eDAC error: Invalid DAC channel")
		
		if ConfigIO and Channel == 1 and caliInfo.hardwareVersion < 1.3:
			# Using ConfigIO to enable DAC1
			(
				__,           # outTimerCounterConfig
				DAC1Enabled,  # outDAC1Enable
				__,           # outFIOAnalog
				__,           # outEIOAnalog
			) = ehConfigIO(
				2,            # inWriteMask
				0,            # inTimerCounterConfig
				1,            # inDAC1Enable
				0,            # inFIOAnalog
				0,            # inEIOAnalog
			)
		
		# Setting up Feedback command to set DAC
		if caliInfo.hardwareVersion < 1.3:
			sendDataBuff = [0] * 2
			
			sendDataBuff[0] = 34 + Channel  # IOType is DAC0/1 (8 bit)
			
			sendDataBuff[1] = self.analogToCalibratedBinary8BitVoltage(caliInfo, Channel, Voltage) # Value
		else:
			sendDataBuff = [0] * 3
			
			sendDataBuff[0] = 38 + Channel  # IOType is DAC0/1 (16 bit)
			
			bytesV = self.analogToCalibratedBinary16BitVoltage(caliInfo, Channel, Voltage)
			
			sendDataBuff[1] = bytesV & 255            # Value LSB
			sendDataBuff[2] = (bytesV & 65280) / 256  # Value MSB
		
		self.ehFeedback(
			sendDataBuff, # inIOTypesDataBuff
			0,            # outDataSize
		)
	
	def ehDIO_Feedback(self, channel, newValue = None, ConfigIO = False):
		if channel < 0 or channel > 19:
			raise LabJackException(0, "DIO Feedback error: Invalid Channel")
		
		sendBuff = [0] * 4
		
		if ConfigIO and Channel <= 15:
			FIOAnalog = 255
			EIOAnalog = 255
			
			# Setting Channel to digital using FIOAnalog and EIOAnalog
			if Channel <= 7:
				FIOAnalog = 255 - pow(2, Channel)
			else:
				EIOAnalog = 255 - pow(2, (Channel - 8))
			
			# Using ConfigIO to get current FIOAnalog and EIOAnalog settings
			(
				curTCConfig,  # outTimerCounterConfig
				__,           # outDAC1Enable
				curFIOAnalog, # outFIOAnalog
				curEIOAnalog, # outEIOAnalog
			) = ehConfigIO(
				0,            # inWriteMask
				0,            # inTimerCounterConfig
				0,            # inDAC1Enable
				0,            # inFIOAnalog
				0,            # inEIOAnalog
			)
			
			if not (FIOAnalog == curFIOAnalog and EIOAnalog == curEIOAnalog):
				# Creating new FIOAnalog and EIOAnalog settings
				# Using ConfigIO to get current FIOAnalog and EIOAnalog settings
				FIOAnalog = FIOAnalog & curFIOAnalog
				EIOAnalog = EIOAnalog & curEIOAnalog
				
				# Using ConfigIO to set new FIOAnalog and EIOAnalog settings
				(
					__,           # outTimerCounterConfig
					__,           # outDAC1Enable
					curFIOAnalog, # outFIOAnalog
					curEIOAnalog, # outEIOAnalog
				) = ehConfigIO(
					12,           # inWriteMask
					curTCConfig,  # inTimerCounterConfig
					0,            # inDAC1Enable
					FIOAnalog,    # inFIOAnalog
					EIOAnalog,    # inEIOAnalog
				)
		
		# Setting up Feedback command to set digital Channel to ( input and to read from it / output and to set the state )
		sendBuff[0] = 13          # IOType is BitDirWrite
		sendBuff[1] = Channel     # IONumber(bits 0-4)
		
		sendBuff[2] = 10          # IOType is BitStateRead
		sendBuff[3] = Channel     # IONumber
		
		if not newValue is None:
			sendBuff[1]+= 128     # + Direction (bit 7)
			sendBuff[2] = 11      # IOType is BitStateWrite
			if newValue:
				sendBuff[3]+= 128 # + State (bit 7)
		
		recSize = 0
		if newValue is None: recSize = 1
		recBuff = self.ehFeedback(
			sendBuff,                    # inIOTypesDataBuff
			recSize,   # outDataSize
		)
		
		if newValue is None:
			return recBuff[0]
		return newValue
	
	def eTCConfig(self, aEnableTimers, aEnableCounters, TCPinOffset, TimerClockBaseIndex, TimerClockDivisor, aTimerModes, aTimerValues):
		if TCPinOffset < 0 and TCPinOffset > 8:
			raise LabJackException(0, "eTCConfig error: Invalid TimerCounterPinOffset")
		
		# ConfigTimerClock
		if TimerClockBaseIndex == LJ_tc2MHZ or TimerClockBaseIndex ==  LJ_tc6MHZ or TimerClockBaseIndex == LJ_tc24MHZ or TimerClockBaseIndex == LJ_tc500KHZ_DIV or TimerClockBaseIndex == LJ_tc2MHZ_DIV or TimerClockBaseIndex == LJ_tc6MHZ_DIV or TimerClockBaseIndex == LJ_tc24MHZ_DIV:
			TimerClockBaseIndex = TimerClockBaseIndex - 10;
		elif TimerClockBaseIndex == LJ_tc4MHZ or TimerClockBaseIndex ==  LJ_tc12MHZ or TimerClockBaseIndex == LJ_tc48MHZ or TimerClockBaseIndex == LJ_tc1MHZ_DIV or TimerClockBaseIndex == LJ_tc4MHZ_DIV or TimerClockBaseIndex == LJ_tc12MHZ_DIV or TimerClockBaseIndex == LJ_tc48MHZ_DIV:
			TimerClockBaseIndex = TimerClockBaseIndex - 20;
		
		(
			__,                        # outTimerClockConfig
			__,                        # outTimerClockDivisor
		) = self.ehConfigTimerClock(
			TimerClockBaseIndex + 128, # inTimerClockConfig
			TimerClockDivisor,         # inTimerClockDivisor
		)
		
		# Using ConfigIO to get current FIOAnalog and curEIOAnalog settings
		(
			__,           # outTimerCounterConfig
			__,           # outDAC1Enable
			curFIOAnalog, # outFIOAnalog
			curEIOAnalog, # outEIOAnalog
		) = ehConfigIO(
			0,            # inWriteMask
			0,            # inTimerCounterConfig
			0,            # inDAC1Enable
			0,            # inFIOAnalog
			0,            # inEIOAnalog
		)
		
		numTimers = 0
		numCounters = 0
		TimerCounterConfig = 0
		FIOAnalog = 255
		EIOAnalog = 255
		
		for i in range(2):
			if aEnableTimers[i]:
				++numTimers
			else:
				break
		
		for i in range(2):
			if aEnableCounters[i]:
				++numCounters
				TimerCounterConfig += pow(2, i + 2)
		
		TimerCounterConfig += numTimers + TCPinOffset * 16
		
		for i in range(numCounters + numTimers):
			if i + TCPinOffset < 8:
				FIOAnalog = FIOAnalog - pow(2, i + TCPinOffset)
			else:
				EIOAnalog = EIOAnalog - pow(2, i + TCPinOffset - 8)
		
		FIOAnalog = FIOAnalog & curFIOAnalog
		EIOAnalog = EIOAnalog & curEIOAnalog
		(
			curTimerCounterConfig, # outTimerCounterConfig
			__,                    # outDAC1Enable
			curFIOAnalog,          # outFIOAnalog
			curEIOAnalog,          # outEIOAnalog
		) = ehConfigIO(
			13,                    # inWriteMask
			TimerCounterConfig,    # inTimerCounterConfig
			0,                     # inDAC1Enable
			FIOAnalog,             # inFIOAnalog
			EIOAnalog,             # inEIOAnalog
		)
		
		if numTimers > 0:
			# Feedback
			sendBuff = [0] * 4 * numTimers
			
			for i in range(numTimers):
				sendBuff[    i * 4] = 43 + i * 2                        # TimerConfig
				sendBuff[1 + i * 4] = aTimerModes[i]                    # TimerMode
				sendBuff[2 + i * 4] =  aTimerValues[i] & 0x00ff         # Value LSB
				sendBuff[3 + i * 4] = (aTimerValues[i] & 0xff00) / 256  # Value MSB
			
			self.ehFeedback(
				sendBuff,     # inIOTypesDataBuff
				0,            # outDataSize
			)
	
	def eTCValues(self, aReadTimers, aUpdateResetTimers, aReadCounters, aResetCounters):
		sendBuff = [0] * 12
		recBuff  = [0] * 16
		
		# Feedback
		numTimers = 0
		dataCountCounter = 0
		dataCountTimer = 0
		sendBuffSize = 0
		recBuffSize = 0
		
		for i in range(2):
			if aReadTimers[i] or aUpdateResetTimers[i]:
				sendBuff[    sendDataBuffSize] = 42 + i * 2                        # Timer
				v = 0
				if aUpdateResetTimers[i]: v = 1
				sendBuff[1 + sendDataBuffSize] = v                                 # UpdateReset
				sendBuff[2 + sendDataBuffSize] =  aTimerValues[i] & 0x00ff         # Value LSB
				sendBuff[3 + sendDataBuffSize] = (aTimerValues[i] & 0xff00) / 256  # Value MSB
				sendBuffSize += 4
				recBuffSize  += 4
				++numTimers
		
		for i in range(2):
			if aReadCounters[i] or aResetCounters[i]:
				sendBuff[    sendDataBuffSize] = 54 + i                     # Counter
				v = 0
				if aResetCounters[i]: v = 1
				sendBuff[1 + sendDataBuffSize] = v       # Reset
				sendBuffSize += 2
				recBuffSize  += 4
		
		sendBuff = sendBuff[0:sendBuffSize]
		
		recBuff = self.ehFeedback(
			sendBuff,     # inIOTypesDataBuff
			recBuffSize,  # outDataSize
		)
		
		for i in range(2):
			aTimerValues[i] = 0
			if aReadTimers[i]:
				for j in range(4):
					aTimerValues[i] += recDataBuff[j + dataCountTimer * 4] * pow(2, 8 * j)
			if aReadTimers[i] or aUpdateResetTimers[i]:
				++dataCountTimer
			
			aCounterValues[i] = 0
			if aReadCounters[i]:
				for j in range(4):
					aCounterValues[i] += recDataBuff[j + numTimers * 4 + dataCountCounter * 4] * pow(2, 8 * j)
			if aReadCounters[i] or aResetCounters[i]:
				++dataCountCounter
	
	def ehConfigIO(self, inWriteMask, inTimerCounterConfig, inDAC1Enable, inFIOAnalog, inEIOAnalog):
		sendBuff = [0] * 12
		
		sendBuff[1] = 0xF8             # command byte
		sendBuff[2] = 3                # Number of data words
		sendBuff[3] = 0x0B             # Extended command number
		
		sendBuff[6] = inWriteMask      # Writemask
		
		sendBuff[7] = 0                # Reserved
		sendBuff[8] = inTimerCounterConfig # TimerCounterConfig
		sendBuff[9] = inDAC1Enable     # DAC1 enable : not enabling
		sendBuff[10]= inFIOAnalog      # FIOAnalog
		sendBuff[11]= inEIOAnalog      # EIOAnalog
		self.extendedChecksum(sendBuff)
		
		# Sending command to U3
		self._LJP.Write(self._LJ, sendBuff, len(sendBuff))
		
		# Reading response from U3
		recSize = 12
		(recChars, recBuff) = self._LJP.Read(self._LJ, False, recSize);
		if recChars < recSize:
			if recChars == 0:
				raise LabJackException(0, "Read failed");
			else:
				raise LabJackException(0, "Only read %d of %d bytes" % (recChars, recSize))
		
		chksum = (recBuff[0], recBuff[4], recBuff[5])
		self.extendedChecksum(recBuff)
		if chksum != (recBuff[0], recBuff[4], recBuff[5]):
			raise LabJackException(0, "Read buffer has bad checksum")
		if recBuff[1] != 0xF8 or recBuff[2] != 3 or recBuff[3] != 0x0B:
			raise LabJackException(0, "Read buffer has wrong command bytes")
		if recBuff[6]:
			raise LabJackException(0, "Read buffer received errorcode %d" % recBuff[6])
		
		return (
			recBuff[8],  # outTimerCounterConfig
			recBuff[9],  # outDAC1Enable
			recBuff[10], # outFIOAnalog
			recBuff[11], # outEIOAnalog
		)
	
	def ehConfigTimerClock(self, inTimerClockConfig, inTimerClockDivisor):
		sendBuff = [0] * 10
		recSize = 10
		
		sendBuff[1] = 0xF8                 # Command byte
		sendBuff[2] = 2                    # Number of data words
		sendBuff[3] = 0x0A                 # Extended command number
		
		sendBuff[8] = inTimerClockConfig   # TimerClockConfig
		sendBuff[9] = inTimerClockDivisor  # TimerClockDivisor
		self.extendedChecksum(sendBuff)
		
		# Sending command to U3
		self._LJP.Write(self._LJ, sendBuff, len(sendBuff))
		
		# Reading response from U3
		(recChars, recBuff) = self._LJP.Read(self._LJ, recSize);
		if recChars < recSize:
			if recChars == 0:
				raise LabJackException(0, "ehConfigTimerClock : read failed")
			else:
				raise LabJackException(0, "ehConfigTimerClock : did not read all of the buffer")
		
		chksum = (recBuff[0], recBuff[4], recBuff[5])
		self.extendedChecksum(recBuff)
		if chksum != (recBuff[0], recBuff[4], recBuff[5]):
			raise LabJackException(0, "Read buffer has bad checksum")
		if recBuff[1] != 0xF8 or recBuff[2] != 2 or recBuff[3] != 0x0A:
			raise LabJackException(0, "Read buffer has wrong command bytes")
		if recBuff[6]:
			raise LabJackException(0, "Read buffer received errorcode %d" % recBuff[6])
		
		return (
			recBuff[8],  # outTimerClockConfig
			recBuff[9],  # outTimerClockDivisor
		)
	
	def ehFeedback(self, inIOTypesDataBuff, outDataSize):
		ret = 0
		commandBytes = 6
		
		sendDWSize = len(inIOTypesDataBuff) + 1
		if sendDWSize % 2:
			++sendDWSize
		recDWSize =            outDataSize  + 3
		if  recDWSize % 2:
			++ recDWSize
		
		sendSize = commandBytes + sendDWSize
		sendBuff = [0] * sendSize
		recSize = commandBytes +  recDWSize
		recBuff  = [0] *  recSize
		
		# Setting up Feedback command
		sendBuff[1] = 0xF8             # Command byte
		sendBuff[2] = sendDWSize / 2   # Number of data words (.5 word for echo, 1.5
		                               # words for IOTypes)
		
		# TODO: optimize this with []+[]
		for i in range(inIOTypesDataSize):
			sendBuff[i + commandBytes + 1] = inIOTypesDataBuff[i]
		
		self.extendedChecksum(sendBuff)
		
		# Sending command to U3
		self._LJP.Write(self._LJ, sendBuff, sendSize)
		
		# Reading response from U3
		(recChars, recBuff) = self._LJP.Read(self._LJ, recSize);
		if recChars < recSize:
			if recChars == 0:
				raise LabJackException(0, "ehFeedback : read failed")
			elif recChars < 8:
				raise LabJackException(0, "ehFeedback : response buffer is too small")
			else:
				raise LabJackException(0, "ehFeedback : did not read all of the buffer")
		
		chksum = (recBuff[0], recBuff[4], recBuff[5])
		self.extendedChecksum(recBuff)
		if chksum != (recBuff[0], recBuff[4], recBuff[5]):
			raise LabJackException(0, "Read buffer has bad checksum")
		if recBuff[1] != 0xF8 or recBuff[3] != 0:
			raise LabJackException(0, "Read buffer has wrong command bytes")
		if recBuff[6]:
			raise LabJackException(0, "Read buffer received errorcode %d (frame: %d)" % (recBuff[6], recBuff[7]))
		
		outDataBuff = [0] * outDataSize
		for i in range(outDataSize):
			if not i + commandBytes + 3 < recChars:
				 break
			outDataBuff[i] = recBuff[i + commandBytes + 3];
		
		return outDataBuff

class U3_HV(U3):
	def binaryToCalibratedAnalogVoltage(self, caliInfo, positiveChannel, negChannel, bytesVoltage):
		self.isCalibrationInfoValid(caliInfo)
		
		if caliInfo.hardwareVersion < 1.3:
			raise LabJackException(0, "binaryToCalibratedAnalogVoltage error: cannot handle U3 hardware versions < 1.30 .  Please use U3 class.")
		
		analogVoltage = None
		
		if (negChannel >= 0 and negChannel <= 15) or negChannel == 30:
			if caliInfo.highVoltage == 0 or (caliInfo.highVoltage == 1 and positiveChannel >= 4 and negChannel >= 4):
				analogVoltage = caliInfo.ainDiffSlope * bytesVoltage + caliInfo.ainDiffOffset
			elif caliInfo.hardwareVersion >= 1.3 and caliInfo.highVoltage == 1:
				raise LabJackException(0, "binaryToCalibratedAnalogVoltage error: invalid negative channel for U3-HV.")
		elif negChannel == 31:
			if caliInfo.highVoltage == 1 and positiveChannel >= 0 and positiveChannel < 4:
				analogVoltage = caliInfo.hvAINSlope[positiveChannel] * bytesVoltage + caliInfo.hvAINOffset[positiveChannel]
			else:
				analogVoltage = caliInfo.ainSESlope * bytesVoltage + caliInfo.ainSEOffset;
		else:
			raise LabJackException(0, "binaryToCalibratedAnalogVoltage error: invalid negative channel.")
		
		return analogVoltage
	
	def binaryToUncalibratedAnalogVoltage(self, highVoltage, positiveChannel, negChannel, bytesVoltage):
		if (negChannel >= 0 and negChannel <= 15) or negChannel == 30:
			if highVoltage == 0 or (highVoltage == 1 and positiveChannel >= 4 and negChannel >= 4):
				analogVoltage = bytesVoltage * 0.000074463 - 2.44
			elif highVoltage == 1:
				LabJackException(0, "binaryToCalibratedAnalogVoltage_hw130 error: invalid negative channel for U3-HV.")
		elif negChannel == 31:
			if highVoltage == 1:
				analogVoltage = bytesVoltage * 0.000314 - 10.3
			else:
				analogVoltage = bytesVoltage * 0.000037231
		else:
			LabJackException(0, "binaryToCalibratedAnalogVoltage_hw130 error: invalid negative channel.")
		
		return analogVoltage

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
