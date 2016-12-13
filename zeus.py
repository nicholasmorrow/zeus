import can
import logging
import signal
import time
from time import sleep
from colorama import init, Fore, Back, Style

DEBUG = 0
INFO = 1
WARNING = 1
ERROR = 1
KICK_MASK = 0x0400
SENDER_ID_MASK = 0x03E0
RECEIVER_ID_MASK = 0x001F
EOM_MASK = 0x0080


def printMSG(type, msg):
    if (type == 'info' and INFO == 1):
        print(Fore.WHITE + "INFO: " + msg + Style.RESET_ALL)

    elif (type == 'debug' and DEBUG == 1):
        print(Fore.MAGENTA + "DEBUG: " + msg + Style.RESET_ALL)

    elif (type == 'warning' and WARNING == 1):
        print(Fore.YELLOW + "WARNING: " + msg + Style.RESET_ALL)

    elif (type == 'error' and ERROR == 1):
        print(Fore.RED + "ERROR: " + msg + Style.RESET_ALL)
        raise Exception(msg)


class ContainerGeometry(object):

    def __init__(self, index=0, diameter=0, bottomHeight=0, bottomSection=0,
                 bottomPosition=0, immersionDepth=0, leavingHeight=0, jetHeight=0,
                 startOfHeightBottomSearch=0, dispenseHeightAfterBottomSearch=0,
                 ):
        self.index = index
        self.diameter = diameter
        self.bottomHeight = bottomHeight
        self.bottomSection = bottomSection
        self.bottomPosition = bottomPosition
        self.immersionDepth = immersionDepth
        self.leavingHeight = leavingHeight
        self.jetHeight = jetHeight
        self.startOfHeightBottomSearch = startOfHeightBottomSearch
        self.dispenseHeightAfterBottomSearch = dispenseHeightAfterBottomSearch
    pass


class DeckGeometry(object):

    def __init__(self, index=None, endTraversePosition=0,
                 beginningofTipPickingPosition=0, positionofTipDepositProcess=0):
        if index is None:
            raise ValueError(
                "Cannot initialize DeckGeometry instance with unspecified index.")
        self.index = index
        self.endTraversePosition = endTraversePosition
        self.beginningofTipPickingPosition = beginningofTipPickingPosition
        self.positionofTipDepositProcess = positionofTipDepositProcess


class LiquidClass(object):

    def __init__(self, id=0, index=None, liquidClassForFilterTips=0,
                 aspirationMode=0, aspirationFlowRate=0, overAspiratedVolume=0,
                 aspirationTransportVolume=0, blowoutAirVolume=0, aspirationSwapSpeed=0,
                 aspirationSettlingTime=0, lld=0, clldSensitivity=0, plldSensitivity=0,
                 adc=0, dispensingMode=0, dispensingFlowRate=0, stopFlowRate=0,
                 stopBackVolume=0, dispensingTransportVolume=0, acceleration=0,
                 dispensingSwapSpeed=0, dispensingSettlingTime=0, flowRateTransportVolume=0):
        if index is None:
            raise ValueError(
                "Cannot initialize LiquidClass instance with unspecified index.")
        self.id = id
        self.index = index
        self.liquidClassForFilterTips = liquidClassForFilterTips
        self.aspirationMode = aspirationMode
        self.aspirationFlowRate = aspirationFlowRate
        self.overAspiratedVolume = overAspiratedVolume
        self.aspirationTransportVolume = aspirationTransportVolume
        self.blowoutAirVolume = blowoutAirVolume
        self.aspirationSwapSpeed = aspirationSwapSpeed
        self.aspirationSettlingTime = aspirationSettlingTime
        self.lld = lld
        self.clldSensitivity = clldSensitivity
        self.plldSensitivity = plldSensitivity
        self.adc = adc
        self.dispensingMode = dispensingMode
        self.dispensingFlowRate = dispensingFlowRate
        self.stopFlowRate = stopFlowRate
        self.stopBackVolume = stopBackVolume
        self.dispensingTransportVolume = dispensingTransportVolume
        self.acceleration = acceleration
        self.dispensingSwapSpeed = dispensingSwapSpeed
        self.dispensingSettlingTime = dispensingSettlingTime
        self.flowRateTransportVolume = flowRateTransportVolume


class remoteFrameListener(can.Listener):

    def __init__(self, p):
        self.parent = p
        self.remote_flag = 0
        self.kick_flag = 0
        self.data_flag = 0
        self.msg = ""
        self.msg_complete_flag = 0

    def on_message_received(self, msg):
        # Ignore the messages that we've sent out
        #  printMSG(
            #  "debug", "Rcvd msg with id = {}, sender id = {}, receiver id = {}, dlc = {}, rmt = {}".format(msg.arbitration_id,
                                                                                                          #  (msg.arbitration_id &
                                                                                                           #  SENDER_ID_MASK),
                                                                                                          #  (msg.arbitration_id &
                                                                                                           #  RECEIVER_ID_MASK),
                                                                                                          #  msg.dlc, msg.is_remote_frame))
        #  print(msg)
        #  if(((msg.arbitration_id & SENDER_ID_MASK) == 0) or (msg.arbitration_id == 401)):
            #  printMSG("debug", "Ignoring previous message.")
            #  return
            # Handle received remote frames
        if(msg.arbitration_id == 0x0401):
            return
        #  if(msg.arbitration_id == self.parent.id):
            #  return

        if(msg.is_remote_frame == True):
            if((msg.arbitration_id == 0x0000) or (msg.arbitration_id == 0x0020)):
                return
            if(self.remote_flag == 0):
                self.remote_flag = 1
                printMSG("info", "Received remote frame with ID = {}, DLC = {}.".format(
                    hex(msg.arbitration_id), msg.dlc))
                print(Fore.BLUE + "{}".format(msg) + Style.RESET_ALL)
            return

        # elif((msg.arbitration_id & KICK_MASK) > 0):
        elif(msg.arbitration_id == 0x0420):
            if(self.kick_flag == 0):
                printMSG("info", "Received Kick.")
                #  print(Fore.BLUE + "Received kick with masked ID = {}.".format(
                    #  (msg.arbitration_id & SENDER_ID_MASK)) + Style.RESET_ALL)
                print(Fore.BLUE + "{}".format(msg) + Style.RESET_ALL)
                self.kick_flag = 1
                printMSG('info', "Sending remote frame.")
                if self.parent.auto_response:
                    self.parent.sendRemoteFrame(1)
            return

        else:
            if(self.msg_complete_flag == 1):
                self.msg = ""

            self.msg += msg.data
            printMSG(
                "info", "Response received with ID = {}.".format(msg.arbitration_id))
            print(Fore.BLUE + "{}".format(msg) + Style.RESET_ALL)
            if(self.msg_is_last(msg) == 0):
                if self.parent.auto_repsonse:
                    self.parent.sendRemoteFrame(8)
            else:
                self.msg_complete_flag = 1
                if self.parent.auto_response:
                    self.parent.sendRemoteFrame(8)

                ret = self.parent.parseErrors(msg.data)
                if(ret != "NONE"):
                    return ret

    def remote_received(self):
        if(self.remote_flag == 1):
            self.remote_flag = 0
            return 1
        else:
            return 0

    def kick_received(self):
        if(self.kick_flag == 1):
            self.kick_flag = 0
            return 1
        else:
            return 0

    def data_received(self):
        if(self.data_flag == 1):
            self.data_flag = 0
            return 1
        else:
            return 0

    def msg_is_last(self, msg):
        size = len(msg.data)
        if(size > 0):
            #  printMSG(
                #  "warning", "control byte length = {}".format(len(msg.data)))
            control_byte = msg.data[(len(msg.data) - 1)]
            if((control_byte & EOM_MASK) > 0):
                return 1

        return 0


def split_by_n(seq, n):
    """A generator to divide a sequence into chunks of n units."""
    while seq:
        yield seq[:n]
        seq = seq[n:]


class ZeusModule(object):
    CANBus = None
    transmission_retries = 1
    remote_timeout = 0.5
    errorTable = {
        "20": "No communication to EEPROM.",
            "30": "Undefined command.",
            "31": "Undefined parameter.",
            "32": "Parameter out of range.",
            "35": "Voltage outside the permitted range.",
            "36": "Emergency stop is active or was sent during action.",
            "38": "Empty liquid class.",
            "39": "Liquid class write protected.",
            "40": "Parallel processes not permitted.",
            "50": "Initialization failed.",
            "51": "Pipetting drive not initialized.",
            "52": "Movement error on pipetting drive.",
            "53": "Maximum volume of the tip reached.",
            "54": "Maximum volume in pipetting drive reached.",
            "55": "Volume check failed.",
            "56": "Conductivity check failed.",
            "57": "Filter check failed.",
            "60": "Initialization failed.",
            "61": "Z-drive is not initialized.",
            "62": "Movement error on the z-drive.",
            "63": "Container bottom search failed.",
            "64": "Z-position not possible.",
            "65": "Z-position not possible.",
            "66": "Z-position not possible.",
            "67": "Z-position not possible.",
            "68": "Z-position not possible.",
            "69": "Z-position not possible.",
            "70": "Liquid level not detected.",
            "71": "Not enough liquid present.",
            "72": "Auto calibration of the pressure sensor not possible.",
            "74": "Early liquid level detection.",
            "75": "No tip picked up or no tip present.",
            "76": "Tip already picked up.",
            "77": "Tip not discarded.",
            "80": "Clot detected during aspiration.",
            "81": "Empty tube detected during aspiration.",
            "82": "Foam detected during aspiration.",
            "83": "Clot detected during dispensing.",
            "84": "Foam detected during dispensing.",
            "85": "No communication to the digital potentiometer.",
    }

    def __init__(self, id=None, init_module=True, auto_response=True):
        # colorama.init()
        init()
        self.id = id
        self.auto_response = auto_response
        if id is None:
            raise ValueError(
                "Cannot initialize ZeusModule instance with unspecified id.")
        elif id not in range(1, 32):
            raise ValueError(
                "Cannot initialize ZeusModule instance with out of range id."
                " Valid id range is [1-31]")
        self.initCANBus()
        self.pos = 0
        self.minZPosition = 0
        self.maxZPosition = 1800
        self.r = remoteFrameListener(self)
        self.remoteFrameNotifier = can.Notifier(self.CANBus, [self.r])
        # print("ZeusModule {}: initializing...".format(self.id))
        logging.info("ZeusModule {}: initializing...".format(self.id))
        if init_module:
            self.initZDrive()
            self.initDosingDrive()

    def setAutoResponse(self, auto):
        self.auto_response = auto

    def cmdHeader(self, command):
        return command + "id" + str(self.id).zfill(4)

    def assembleIdentifier(self, msg_type, master_id=0):
        identifier = (0 | self.id)
        identifier |= (master_id << 5)
        if msg_type is 'kick':
            identifier |= 1 << 10
        return identifier

    def sendRemoteFrame(self, dlc):
            # SEND REMOTE FRAME
        msg = can.Message(
            extended_id=False,
            is_remote_frame=True,
              arbitration_id=20)
        for i in range(0, dlc):
            msg.data.append("0")
        msg.dlc = dlc
        msg.arbitration_id = (0x0020)
        print(
            "INFO: ZeusModule {}: sending remote frame with dlc = {}...".format(self.id, msg.dlc))
        print(Fore.GREEN + "{}".format(msg) + Style.RESET_ALL)
        try:
            self.CANBus.send(msg)
        except can.canError:
            print(
                Fore.RED + "ERROR: Remote frame not sent!" + Style.RESET_ALL)
            # WAIT FOR REMOTE RESPONSE
            #  sleep(self.remote_timeout)
            #  s = time.time()
            #  c = time.time()
            # LOOP HERE UNTIL TIMEOUT EXPIRES
            #  while ((c - s) < self.remote_timeout):
                #  c = time.time()
                #  if(self.r.remote_received() == 1):
                    #  print(
                        #  Fore.MAGENTA + "DEBUG: ACK Received." +
                        #  Style.RESET_ALL)
                    #  n = self.transmission_retries
            #  if(n < self.transmission_retries):
                #  printMSG("warning", "Timeout waiting for kick response. Issuing retry {} of {}".format(
                    #  n + 1, self.transmission_retries))

            #  n += 1

    def sendKickFrame(self):
        n = 0
        msg = can.Message(
            extended_id=False,
                arbitration_id=self.assembleIdentifier('kick'),
                data=0
        )
        print(
            "INFO: ZeusModule {}: sending kick frame...".format(self.id))
        print(Fore.GREEN + "{}".format(msg) + Style.RESET_ALL)
        while (n < self.transmission_retries):
            try:
                self.CANBus.send(msg)
            except can.canError:
                printMSG("error", "Kick not sent!")

             # WAIT FOR REMOTE RESPONSE
            if(self.waitForRemoteResponse() == 1):
                return
            if(n < self.transmission_retries):
                printMSG("warning", "Timeout waiting for kick response. Issuing retry {} of {}".format(
                    n + 1, self.transmission_retries))

            n += 1

        # Should not get here unless no response is received.
        printMSG(
            "error", "No remote frame received. Check connection to Zeus module.")
        exit(1)

    def waitForRemoteResponse(self):
        # WAIT FOR REMOTE RESPONSE
        sleep(self.remote_timeout)
        s = time.time()
        c = time.time()
        # LOOP HERE UNTIL TIMEOUT EXPIRES
        while ((c - s) < self.remote_timeout):
            c = time.time()
            if(self.r.remote_received() == 1):
                #  printMSG("debug", "ACK Received.")
                return 1

    def waitForKickResponse(self):
        # WAIT FOR REMOTE RESPONSE
        sleep(self.remote_timeout)
        s = time.time()
        c = time.time()
        # LOOP HERE UNTIL TIMEOUT EXPIRES
        while ((c - s) < self.remote_timeout):
            c = time.time()
            if(self.r.kick_received() == 1):
                #  printMSG("debug", "ACK Received.")
                return 1

    def sendDataObject(self, i, cmd_len, data):
        byte = 0
        printMSG(
            "info", "ZeusModule {}: sending data frame {} of {}...".format(self.id, i + 1, cmd_len))
        printMSG(
            "debug", "data pre append = {}".format(data.encode('hex')))

        # PAD FRAME WITH ZEROES
        while(len(data) < 7):
            data += "0"

        printMSG('debug', "Outstring = {}".format(data))
        # Assemble the 8th (status) byte
        # Add EOM bit if this is the last frame of the message.
        if (i == (cmd_len - 1)):
            printMSG("debug", "appending EOM bit to byte")
            byte |= 1 << 7
        # Add number of data bytes
        byte |= (len(data[i]) << 4)
        printMSG("debug", "num data bytes = {}".format(len(data[i])))
        byte |= ((i + 1) % 31)
        printMSG("debug", "frame counter = {}".format(((i + 1) % 31)))
        printMSG("debug", "control byte = {0:b}".format(byte))
        data += chr(byte)
        printMSG(
            "debug", "data post append = {}".format(data.encode('hex')))
        msg = can.Message(
            extended_id=False,
            arbitration_id=self.assembleIdentifier('data'),
            data=data)
        try:
            self.CANBus.send(msg)
            print(Fore.GREEN + "{}".format(msg) + Style.RESET_ALL)
        except can.CanError:
            printMSG("error", "Message not sent!")

    def sendCommand(self, cmd):
        data = list(split_by_n(cmd, 7))
        cmd_len = len(data)
        printMSG(
            "info", "ZeusModule {}: sending packet {} in {} data frame(s)...".format(self.id, cmd, cmd_len))

        # Send kick frame and wait for remote response
        self.sendKickFrame()
        for i in range(0, cmd_len):
            n = 0
            # outstring = bytearray(data[i])
            outstring = data[i]
            # sendDataObject(outstring)
            while (n < self.transmission_retries):
                # SEND DATA FRAME
                self.sendDataObject(i, cmd_len, outstring)

                # WAIT FOR REMOTE RESPONSE
                if(self.waitForRemoteResponse() == 1):
                   # self.sendRemoteFrame()
                    printMSG("warning", "just set n=1.")
                    n = self.transmission_retries

                if(n < self.transmission_retries):
                    printMSG("warning", "Timeout waiting for remote response. Issuing retry {} of {}".format(
                        n + 1, self.transmission_retries))

                n += 1

    def initCANBus(self):
        print("ZeusModule {}: initializing CANBus...")
        logging.info("ZeusModule {}: initializing CANBus...")
        can.rc['interface'] = 'socketcan_ctypes'
        can.rc['channel'] = 'can0'
        self.CANBus = can.interface.Bus(receive_own_messages=False)
        # self.CANBus = can.interfaces.socketcan_ctypes.SocketcanCtypes_Bus(receive_own_messages=False)
        # self.remoteFrameListener = can.Listener()
        # self.remoteFrameNotifier = can.Notifier(self.CANBus, [self.r])

    def initDosingDrive(self):
        cmd = self.cmdHeader('DI')
        print("ZeusModule {}: initializing dosing drive...".format(self.id))
        self.sendCommand(cmd)

    def initZDrive(self):
        cmd = self.cmdHeader('ZI')
        print("ZeusModule {}: initializing z-drive...".format(self.id))
        self.pos = self.maxZPosition
        self.sendCommand(cmd)

    def moveZDrive(self, pos, speed):
        cmd = self.cmdHeader('GZ')
        if (pos > self.maxZPosition) or (pos < self.minZPosition):
            raise ValueError(
                "ZeusModule {}: requested z-position out of range. "
                " Valid range for z-position is between {} and {}"
                .format(self.id, self.minZPosition, self.maxZPosition))
        if (speed == "slow"):
            speed = 0
        elif (speed == "fast"):
            speed = 1
        else:
            raise ValueError(
                "ZeusModule {}: invalid z-axis drive speed specified."
                    " Accepted values for z-axis drive speed are \'slow\'"
                    " and \'fast\'.")
        print(
            "ZeusModule {}: moving z-drive from position {} to position {}."
            .format(self.id, self.pos, pos))
        cmd = cmd + 'gy' + str(pos).zfill(4) + 'gw' + str(speed)
        self.pos = pos
        self.sendCommand(cmd)

    def pickUpTip(self, tipTypeTableIndex, deckGeometryTableIndex):
        cmd = self.cmdHeader('GT')
        cmd = cmd + 'tt' + str(tipTypeTableIndex).zfill(
            2) + 'go' + str(deckGeometryTableIndex).zfill(2)

        self.sendCommand(cmd)

    def discardTip(self, deckGeometryTableIndex):
        cmd = self.cmdHeader('GU')
        cmd = cmd + 'go' + str(deckGeometryTableIndex).zfill(2)
        self.sendCommand(cmd)

    def sendString(self, string):
        # cmd = self.cmdHeader(string)
        self.sendCommand(string)

    def aspiration(self, aspirationVolume=0, containerGeometryTableIndex=0,
                   deckGeometryTableIndex=0, liquidClassTableIndex=0, qpm=0,
                   lld=0, lldSearchPosition=0, liquidSurface=0, mixVolume=0,
                   mixFlowRate=0, mixCycles=0):
        cmd = self.cmdHeader('GA')
        cmd = cmd + 'ai' + str(aspirationVolume).zfill(5) +\
            'ge' + str(containerGeometryTableIndex).zfill(2) +\
            'go' + str(deckGeometryTableIndex).zfill(2) +\
            'lq' + str(liquidClassTableIndex).zfill(2) +\
            'gq' + str(qpm) +\
            'lb' + str(lld) +\
            'zp' + str(lldSearchPosition).zfill(4) +\
            'cf' + str(liquidSurface).zfill(4) +\
            'ma' + str(mixVolume).zfill(5) +\
            'mb' + str(mixFlowRate).zfill(5) +\
            'dn' + str(mixCycles).zfill(2)
        self.sendCommand(cmd)

    def dispensing(self, dispensingVolume=0, containerGeometryTableIndex=0,
                   deckGeometryTableIndex=0, qpm=0, liquidClassTableIndex=0,
                   lld=0, lldSearchPosition=0, liquidSurface=0,
                   searchBottomMode=0, mixVolume=0, mixFlowRate=0, mixCycles=0):
        cmd = self.cmdHeader('GD')
        cmd = cmd + 'di' + str(dispensingVolume).zfill(4) +\
            'ge' + str(containerGeometryTableIndex).zfill(2) +\
            'go' + str(deckGeometryTableIndex).zfill(2) +\
            'gq' + str(qpm) +\
            'lq' + str(liquidClassTableIndex).zfill(2) +\
            'lb' + str(lld) +\
            'zp' + str(lldSearchPosition).zfill(4) +\
            'cf' + str(liquidSurface).zfill(4) +\
            'zm' + str(searchBottomMode) +\
            'ma' + str(mixVolume).zfill(5) +\
            'mb' + str(mixFlowRate).zfill(5) +\
            'dn' + str(mixCycles).zfill(2)
        self.sendCommand(cmd)

    def switchOff(self):
        cmd = self.cmdHeader('AV')
        self.sendCommand(cmd)

    def calculateContainerVolume(self, containerGeometryTableIndex=0,
                                 deckGeometryTableIndex=0,
                                 liquidClassTableIndex=0, lld=0,
                                 lldSearchPosition=0, liquidSurface=0):
        cmd = self.cmdHeader('GJ')
        cmd = cmd + 'ge' + str(containerGeometryTableIndex).zfill(2) +\
            'go' + str(deckGeometryTableIndex).zfill(2) +\
            'lq' + str(liquidClassTableIndex).zfill(2) +\
            'lb' + str(lld) +\
            'zp' + str(lldSearchPosition).zfill(4) +\
            'cf' + str(liquidSurface).zfill(3)
        self.sendCommand(cmd)

    def calculateContainerVolumeAfterPipetting(self):
        cmd = self.cmdHeader('GN')
        self.sendCommand(cmd)

    def getErrorCode(self):
        cmd = self.cmdHeader('RE')
        self.sendCommand(cmd)

    def getFirmwareVersion(self):
        # cmd = self.cmdHeader('RF')
        self.sendCommand("RFid001")
        # self.sendCommand(cmd)

    def getParameterValue(self, parameterName):
        if (len(parameterName) > 2):
            raise ValueError(
                "ZeusModule {}: Invalid parameter \'{}\' requested. "
                " Parameter format must be two lower-case letters."
                .format(parameterName))
        cmd = self.cmdHeader('RA')
        cmd = cmd + 'ra' + parameterName
        self.sendCommand(cmd)

    def getInstrumentInitializationStatus(self):
        cmd = self.cmdHeader('QW')
        self.sendCommand(cmd)

    def getNameofLastFaultyParameter(self):
        cmd = self.cmdHeader('VP')
        self.sendCommand(cmd)

    def getTipPresenceStatus(self):
        cmd = self.cmdHeader('RT')
        self.sendCommand(cmd)

    def getTechnicalStatus(self):
        cmd = self.cmdHeader('QT')
        self.sendCommand(cmd)

    def getAbsoluteZPosition(self):
        cmd = self.cmdHeader('RZ')
        self.sendCommand(cmd)

    def getCycleCounter(self):
        cmd = self.cmdHeader('RV')
        self.sendCommand(cmd)

    def getLifetimeCounter(self):
        cmd = self.cmdHeader('RY')
        self.sendCommand(cmd)

    def getInstrumentStatus(self):
        cmd = self.cmdHeader('RQ')
        self.sendCommand(cmd)
        pass

    def getLiquidClassRevision(self):
        cmd = self.cmdHeader('XB')
        self.sendCommand(cmd)
        pass

    def setEmergencyStop(self, state):
        if state in set([1, 'on', 'ON', 'True', 'true']):
            cmd = self.cmdHeader('AB')
        if state in set([0, 'off', 'OFF', 'False', 'false']):
            cmd = seld.cmdHeader('AW')
        self.sendCommand(cmd)

    def switchDigitalOutput(self, out1State, out2State):
        cmd = self.cmdHeader('OU')
        cmd += 'ou'
        if out1State in set([1, 'on', 'ON', 'True', 'true']):
            cmd += str(1)
        if out2State in set([0, 'off', 'OFF', 'False', 'false']):
            cmd += str(0)

        cmd += 'oy'
        if out2State in set([1, 'on', 'ON', 'True', 'true']):
            cmd += str(1)
        if out2State in set([0, 'off', 'OFF', 'False', 'false']):
            cmd += str(0)

    def switchLEDStatus(self, blueState, redState):
        if (blueState not in set([0.1])) or (redState not in set([0, 1])):
            raise ValueError(
                "ZeusModule {}: requested LED state out of range. "
                " Valid range for LED state is [0,1]".format(self.id))
        cmd = self.cmdHeader('SL')
        cmd += 'sl' + str(blueState) +\
            'sk' + str(redState)
        print("Switching status of blue LED to {} and red LED to {}"
              .format(blueState, redState))

    def testModeCommand(self, status):
        if (status not in set([0, 1])):
            raise ValueError(
                "ZeusModule {}: requested LED state out of range. "
                " Valid range for LED state is [0,1]".format(self.id))
        cmd = self.cmdHeader('AT')
        cmd += 'at' + str(status)
        self.sendCommand(cmd)

    def setDosingDriveInCleaningPosition(self):
        cmd = self.cmdHeader('GX')
        self.sendCommand(cmd)

    def setContainerGeometryParameters(self, containerGeometryParameters):
        cmd = self.cmdHeader('GC')
        cmd = cmd + 'ge' + str(containerGeometryParameters.index).zfill(2) +\
            'cb' + str(containerGeometryParameters.diameter).zfill(3) +\
            'bg' + str(containerGeometryParameters.bottomHeight).zfill(4) +\
            'gx' + str(ContainerGeometryParameters.bottomSection).zfill(5) +\
            'ce' + str(ContainerGeometryParameters.bottomPosition).zfill(4) +\
            'ie' + str(ContainerGeometryParameters.immersionDepth).zfill(4) +\
            'yq' + str(ContainerGeometryParameters.leavingHeight).zfill(4) +\
            'yr' + str(ContainerGeometryParameters.jetHeight).zfill(4) +\
            'ch' +\
            str(ContainerGeometryParameters.startOfHeightBottomSearch).zfill(4) +\
            'ci' +\
            str(ContainerGeometryParameters.dispenseHeightAfterBottomSearch).zfill(
            4)
        self.sendCommand(cmd)

    def getContainerGeometryParameters(self, index):
        if index is None:
            raise ValueError(
                "Please specify a valid container geometry table index.")
        cmd = self.cmdHeader('GB')
        cmd = cmd + 'ge' + str(index).zfill(2)
        ret = ContainerGeometry(index=index)
        # Request and fill class attributes here
        return ret

    def setDeckGeometryParameters(self, deckGeometryParameters):
        cmd = self.cmdHeader('GO')
        cmd = cmd + 'go' + str(deckGeometryParameters.index).zfill(2) +\
            'te' + str(deckGeometryParameters.endTraversePosition).zfill(4) +\
            'tm' +\
            str(deckGeometryParameters.beginningofTipPickingPosition).zfill(4) +\
            'tr' +\
            str(deckGeometryParameters.positionofTipDepositProcess).zfill(
            4)
        self.sendCommand(cmd)

    def getDeckGeometryParameters(self, index):
        if index is None:
            raise ValueError(
                "Please specify a valid deck geometry table index.")
        cmd = self.cmdHeader('GR')
        cmd = cmd + 'go' + str(index).zfill(2)
        self.sendCommmand(cmd)

    def setLiquidClassParameters(self, liquidClassParameters):
        if liquidClassParameters.index is None:
            raise ValueError(
                "Please specify a valid deck geometry table index.")
        cmd = self.cmdHeader('GL')
        cmd = cmd + 'id' + str(liquidClassParameters.id).zfill(4) +\
            'lq' + str(liquidClassParameters.index).zfill(2) +\
            'uu' + str(liquidClassParameters.liquidClassForFilterTips) +\
            str(liquidClassParameters.aspirationMode) +\
            str(liquidClassParameters.aspirationFlowRate).zfill(5) +\
            str(liquidClassParameters.overAspiratedVolume).zfill(4) +\
            str(liquidClassParameters.aspirationTransportVolume).zfill(5) +\
            str(liquidClassParameters.blowoutAirVolume).zfill(5) +\
            str(liquidClassParameters.aspirationSwapSpeed).zfill(4) +\
            str(liquidClassParameters.aspirationSettlingTime).zfill(3) +\
            str(liquidClassParameters.lld) +\
            str(liquidClassParameters.clldSensitivity) +\
            str(liquidClassParameters.plldSensitivity) +\
            str(liquidClassParameters.adc) +\
            str(liquidClassParameters.dispensingMode) +\
            str(liquidClassParameters.dispensingFlowRate).zfill(5) +\
            str(liquidClassParameters.stopFlowRate).zfill(5) +\
            str(liquidClassParameters.stopBackVolume).zfill(3) +\
            str(liquidClassParameters.dispensingTransportVolume).zfill(5) +\
            str(liquidClassParameters.acceleration).zfill(3) +\
            str(liquidClassParameters.dispensingSwapSpeed).zfill(4) +\
            str(liquidClassParameters.dispensingSettlingTime).zfill(3) +\
            str(liquidClassParameters.flowRateTransportVolume).zfill(5)
        self.sendCommand(cmd)

    def getLiquidClassParameters(self, id, index):
        cmd = self.cmdHeader('GM')
        cmd = cmd + 'id' + str(id).zfill(4) +\
            'iq' + str(index).zfill(2)
        self.sendCommand(cmd)

    def firmwareUpdate(self, filename):
        pass

    def parseErrors(self, errorString):
        if len(errorString.replace(" ", "")) == 0:
            return "NONE"
        if errorString == "":
            return "NONE"

        printMSG("debug", "ErrorString = {}".format(errorString))

        #  else:
            # print(Fore.MAGENTA + "DEBUG: Received error string '{}' with
            # length: {}".format(errorString, len(errorString.replace(" ",
            # ""))) + Style.RESET_ALL)

        cmd = str(errorString[:2])
        # print("cmd = {}".format(cmd))
        eidx = errorString.find("er")
        if(eidx == -1):
            return "NONE"

        ec = str(errorString[(eidx + 3): (eidx + 5)])
        if ec == '00':
            # NO ERROR
            return
        #  print(Fore.MAGENTA + "DEBUG: ec = {}".format(ec))

        defaultError = "Unknown error code returned."
        if cmd == 'DI':
            if ec in set(['00', '30', '35', '36', '40', '50', '52']):
                return self.errorTable[ec]
            else:
                return defaultError

        elif cmd == 'ZI':
            if ec in set(['00', '30', '35', '36', '40', '60', '62']):
                return self.errorTable[ec]
            else:
                return defaultError

        elif cmd == 'GZ':
            if ec in set(['00', '31', '32', '35', '36', '40', '61', '62', '64']):
                return self.errorTable[ec]
            else:
                return defaultError

        elif cmd == 'GT':
            if ec in set(['00', '31', '32', '35', '36', '40', '51', '52', '61',
                          '62', '65', '75', '76']):
                return self.errorTable[ec]
            else:
                return defaultError

        elif cmd == 'GU':
            if ec in set(['00', '30', '31', '32', '35', '36', '40', '51', '52',
                          '61', '62', '65', '69', '75', '77']):
                return self.errorTable[ec]
            else:
                return defaultError

        elif cmd == 'GA':
            if ec in set(['00', '30', '31', '32', '35', '36', '38', '40', '51',
                          '52', '53', '54', '55', '56', '57', '61', '62', '65',
                          '66', '67', '68', '70', '71', '72', '74', '75', '80',
                          '81', '82', '85']):
                return self.errorTable[ec]
            else:
                return defaultError

        elif cmd == 'GD':
            if ec in set(['00', '30', '31', '32', '35', '36', '38', '40', '51',
                          '52', '54', '55', '57', '61', '62', '63', '65', '66',
                          '67', '68', '70', '72', '74', '75', '83', '84', '85']):
                return self.errorTable[ec]
            else:
                return defaultError

        elif cmd == 'GJ':
            if ec in set(['00', '30', '31', '32', '35', '36', '38', '40', '51',
                          '52', '56', '57', '61', '62', '65', '66', '67', '68',
                          '70', '72', '74', '85']):
                return self.errorTable[ec]
            else:
                return defaultError

        elif cmd == 'AB':
            if ec in set(['00', '30']):
                return self.errorTable[ec]
            else:
                return defaultError
        elif cmd == 'AW':
            if ec in set(['00', '30']):
                return self.errorTable[ec]
            else:
                return defaultError
        elif cmd == 'XA':
            if ec in set(['00', '20', '30', '31', '32']):
                return self.errorTable[ec]
            else:
                return defaultError

        elif cmd == 'GK':
            if ec in set(['00', '30', '31', '32', '35', '51', '52', '54']):
                return self.errorTable[ec]
            else:
                return defaultError

        elif cmd == 'GC':
            if ec in set(['00', '20', '30', '31', '32']):
                return self.errorTable[ec]
            else:
                return defaultError

        elif cmd == 'GO':
            if ec in set(['00', '20', '30', '31', '32']):
                return self.errorTable[ec]
            else:
                return defaultError

        elif cmd == 'GB':
            if ec in set(['00', '20', '30', '31', '32']):
                return self.errorTable[ec]
            else:
                return defaultError

        elif cmd == 'GR':
            if ec in set(['00', '20', '30', '31', '32']):
                return self.errorTable[ec]
            else:
                return defaultError

        elif cmd == 'GL':
            if ec in set(['00', '20', '30', '31', '32', '39']):
                return self.errorTable[ec]
            else:
                return defaultError

        elif cmd == 'GM':
            if ec in set(['00', '20', '30', '31', '32']):
                return self.errorTable[ec]
            else:
                return defaultError

        elif cmd == 'GQ':
            if ec in set(['00', '20', '30', '31', '32']):
                return self.errorTable[ec]
            else:
                return defaultError

        elif cmd == 'GS':
            if ec in set(['00', '20', '30', '31', '32']):
                return self.errorTable[ec]
            else:
                return defaultError

        elif cmd == 'GV':
            if ec in set(['00', '20', '30', '31', '32']):
                return self.errorTable[ec]
            else:
                return defaultError

        elif cmd == 'GW':
            if ec in set(['00', '20', '30', '31', '32']):
                return self.errorTable[ec]
            else:
                return defaultError

        elif cmd == 'GG':
            if ec in set(['00', '20', '30', '31', '32']):
                return self.errorTable[ec]
            else:
                return defaultError

        elif cmd == 'GE':
            if ec in set(['00', '20', '30', '31', '32']):
                return self.errorTable[ec]
            else:
                return defaultError

        elif cmd == 'GH':
            if ec in set(['00', '20', '30', '31', '32']):
                return self.errorTable[ec]
            else:
                return defaultError

        elif cmd == 'GI':
            if ec in set(['00', '20', '30', '31', '32']):
                return self.errorTable[ec]
            else:
                return defaultError
        else:
            return "Error code returned '{}' corresponds to unknown command.".format(errorString)
