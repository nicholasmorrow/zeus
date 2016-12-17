import zeus
import ezstepper
from time import sleep

zm = zeus.ZeusModule(1)
ez = ezstepper.EZStepper(1)


def demo():
    ez.setVelocity(3)
    zm.moveZDrive(0, "fast")
    ez.moveAbsolute(0)
    sleep(1)
    zm.moveZDrive(1000, "fast")
    for i in range(0, 6):
        zm.moveZDrive(1400, "fast")
        zm.moveZDrive(1500, "slow")
        zm.moveZDrive(1000, "fast")
        ez.moveRelative(14.5)
        sleep(1)
    zm.moveZDrive(1400, "fast")
    zm.moveZDrive(1500, "slow")
    zm.moveZDrive(1000, "fast")
    ez.moveAbsolute(0)
    zm.moveZDrive(0, "slow")

if __name__ == '__main__':
    demo()
