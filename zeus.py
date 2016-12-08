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

def printMSG(type, msg):
    if (type == 'info' and INFO == 1):
        print(Fore.WHITE + "INFO: " + msg + Style.RESET_ALL)

    elif (type == 'debug' and DEBUG == 1):
        print(Fore.MAGENTA + "DEBUG: " + msg + Style.RESET_ALL)

    elif (type == 'warning' and WARNING == 1):
        print(Fore.YELLOW + "WARNING: " + msg + Style.RESET_ALL)

    elif (type == 'error' and ERROR == 1):
        print(Fore.RED + "ERROR: " + msg + Style.RESET_ALL)


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
        self.flag = 0

    def on_message_received(self, msg):
        if(msg.is_remote_frame == True):
            print(Fore.BLUE + "Received remote frame with DLC={}.".format(
                msg.dlc) + Style.RESET_ALL)
            self.flag = 1
        elif(msg.dlc == 0):
            print(Fore.BLUE + "Received kick with DLC=0.".format(
                msg.dlc) + Style.RESET_ALL)
            printMSG('info', "Sending remote frame.")
            self.parent.sendRemoteFrame()
        else:
            print(Fore.BLUE + "{}".format(msg) + Style.RESET_ALL)
            ret = self.parent.parseErrors(msg.data)
            if(ret):
                print(Fore.RED + ret + Style.RESET_ALL)

    def remote_received(self):
        if(self.flag == 1):
            self.flag = 0
            return 1
        else:
            return 0


def split_by_n(seq, n):
    """A generator to divide a sequence into chunks of n units."""
    while seq:
        yield seq[:n]
        seq = seq[n:]


class ZeusModule(object):
    CANBus = None
    transmission_retries = 10
    remote_timeout = 0.1
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

    def __init__(self, id=None):
        # colorama.init()
        init()
        self.id = id
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
        self.initZDrive()
        self.initDosingDrive()

    def cmdHeader(self, command):
        return command + "id" + str(self.id).zfill(4)

    def assembleIdentifier(self, msg_type, master_id=0):
        identifier = (0 | self.id)
        identifier |= (master_id << 5)
        if msg_type is 'kick':
            identifier |= 1 << 10
        return identifier
    
    def sendRemoteFrame(self):
            # SEND REMOTE FRAME
            msg = can.Message(
                extended_id=False,
                is_remote_frame=True,
                    arbitration_id=self.assembleIdentifier('kick')
            )
            print(
                "INFO: ZeusModule {}: sending remote frame...".format(self.id))
            print(Fore.GREEN + "{}".format(msg) + Style.RESET_ALL)
            try:
                self.CANBus.send(msg)
            except can.canError:
                print(Fore.RED + "ERROR: Remote frame not sent!" + Style.RESET_ALL)
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
    
    def sendCommand(self, cmd):
        data = list(split_by_n(cmd, 7))
        cmd_len = len(data)
        print(
            "INFO: ZeusModule {}: sending packet {} in {} data frame(s)...".format(self.id, cmd, cmd_len))
        for i in range(0, cmd_len):
            n = 0
            byte = 0
            # RETRY UP TO 10 TIMES
            # SEND KICK FRAME
            while (n < self.transmission_retries):
                msg = can.Message(
                    extended_id=False,
                        arbitration_id=self.assembleIdentifier('kick'),
                        data=0
                )
                print(
                    "INFO: ZeusModule {}: sending kick frame...".format(self.id))
                print(Fore.GREEN + "{}".format(msg) + Style.RESET_ALL)
                try:
                    self.CANBus.send(msg)
                except can.canError:
                    print(Fore.RED + "ERROR: Kick not sent!" + Style.RESET_ALL)
                # WAIT FOR REMOTE RESPONSE
                sleep(self.remote_timeout)
                s = time.time()
                c = time.time()
                # LOOP HERE UNTIL TIMEOUT EXPIRES
                while ((c - s) < self.remote_timeout):
                    c = time.time()
                    if(self.r.remote_received() == 1):
                        #  print(
                            # Fore.MAGENTA + "DEBUG: ACK Received." +
                            # Style.RESET_ALL)
                        n = self.transmission_retries
                if(n < self.transmission_retries):
                    printMSG("warning", "Timeout waiting for kick response. Issuing retry {} of {}".format(
                        n + 1, self.transmission_retries))

                n += 1
            #  print(
                # Fore.MAGENTA + "DEBUG: Jumped out of while loop." +
                # Style.RESET_ALL)

            n = 0
            while (n < 10):
                # SEND DATA FRAME
                string_length = len(data[i])
                outstring = bytearray(data[i])
                # print("INFO: ZeusModule {}: sending data frame {} of
                # {}...".format(self.id, i+1, cmd_len))
                #  print(Fore.MAGENTA + "DEBUG: data pre append = {}".format(
                    #  outstring) + Style.RESET_ALL)
                printMSG('debug', "data pre append = {}".format(outstring))
                # Assemble the 8th (status) byte
                if (i == (cmd_len - 1)):
                    # Add EOM bit
                    printMSG("debug","appending EOM bit to byte")
                    byte |= 1 << 7
                # Add number of data bytes
                #byte |= (len(data[i]) << 4)
                byte |= (string_length << 4)
                printMSG("debug","num data bytes = {}".format(len(data[i])))
                byte |= ((i + 1) % 31)
                printMSG("debug","byte counter = {}".format(((i + 1) % 31)))
                printMSG("debug","byte = {0:b}".format(byte))
                outstring.append(byte)
                printMSG("debug","date post append = {}".format(outstring))
                msg = can.Message(
                    extended_id=False,
                    arbitration_id=self.assembleIdentifier('data'),
                    data=outstring)
                print("info","ZeusModule {}: sending data frame {} of {}...".format(self.id, i + 1, cmd_len))
                print(Fore.GREEN + "{}".format(msg) + Style.RESET_ALL)
                try:
                    self.CANBus.send(msg)
        #          print("Message")
                except can.canError:
                    printMSG("error","Message not sent!")

                # WAIT FOR REMOTE RESPONSE
                s = time.time()
                c = time.time()
                # LOOP HERE UNTIL TIMEOUT EXPIRES
                while ((c - s) < self.remote_timeout):
                    c = time.time()
                    if(self.r.remote_received() == 1):
                        #  print(
                            # Fore.MAGENTA + "DEBUG: ACK Received." +
                            # Style.RESET_ALL)
                        n = self.transmission_retries
                if(n < self.transmission_retries):
                    print (Fore.YELLOW + "WARNING: Timeout waiting for kick response. Issuing retry {} of {}".format(
                        n + 1, self.transmission_retries) + Style.RESET_ALL)

                n += 1

    def initCANBus(self):
        print("ZeusModule {}: initializing CANBus...")
        logging.info("ZeusModule {}: initializing CANBus...")
        can.rc['interface'] = 'socketcan_ctypes'
        can.rc['channel'] = 'can0'
        self.CANBus = can.interface.Bus()
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
        #cmd = self.cmdHeader(string)
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
        #cmd = self.cmdHeader('RF')
        self.sendCommand("RFid001")
        #self.sendCommand(cmd)

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
            'ch' + str(ContainerGeometryParameters.startOfHeightBottomSearch).zfill(4) +\
            'ci' + \
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
            'tm' + str(deckGeometryParameters.beginningofTipPickingPosition).zfill(4) +\
            'tr' + \
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
        #  print(Fore.MAGENTA + "DEBUG: ErrorString = {}".format(errorString))
        #  if len(errorString.replace(" ", "")) == 0:
            #  return
        if errorString == "":
            return

        #  else:
            # print(Fore.MAGENTA + "DEBUG: Received error string '{}' with
            # length: {}".format(errorString, len(errorString.replace(" ",
            # ""))) + Style.RESET_ALL)

        cmd = str(errorString[:2])
        # print("cmd = {}".format(cmd))
        eidx = errorString.find("er")
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
