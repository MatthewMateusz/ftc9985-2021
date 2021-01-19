package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Main Autonomous", group="INDEV")
public class MainAutonomous extends Automation {

    RingState ringState = RingState.NONE;

    VuforiaDetection vuforia = new VuforiaDetection();

    public void instruction() throws InterruptedException {
        vuforia.start();
        setWobbleGrabber(WobbleGrabber.CLOSE);
        setYeeter(0.65);
        sleep(2000);
        setGate(GatePosition.OPEN);
        sleep(500);
        setConveyor(ConveyorState.ON, 0.8);
        sleep(200);
        rotate(-5, 1, 2, false);
        sleep(10);
        rotate(-5, 1, 2, true);
        sleep(500);
        setYeeter(0);
        setConveyor(ConveyorState.OFF, 0.05);
        setGate(GatePosition.CLOSE);
        rotate(10, 1, 2, true);
        vuforia.join();

        switch (ringState) {
            case NONE:
                driveUntilCondition(new LineDrive(LineColor.WHITE), 1, -41.5, 10, true);
                setWobbleGrabber(WobbleGrabber.OPEN);
                driveUntilCondition(new TimeDrive(0.1), 0.75, 0, 3, true);
                driveUntilCondition(new TimeDrive(1), 0.75, -180, 3, true);
                driveUntilCondition(new LineDrive(LineColor.WHITE), 1, 60, 5, true);
                break;

            case ONE:
                driveUntilCondition(new LineDrive(LineColor.WHITE), 1, -22, 5, false);
                driveUntilCondition(new TimeDrive(1), 1, -22, 5, true);
                setWobbleGrabber(WobbleGrabber.OPEN);
                driveUntilCondition(new TimeDrive(0.1), 0.75, 0, 3, true);
                driveUntilCondition(new TimeDrive(1), 1, -180, 5, true);
                break;

            case QUAD:
                driveUntilCondition(new LineDrive(LineColor.WHITE), 0.75, -20, 5, false);
                driveUntilCondition(new TimeDrive(4), 1, -50, 5, false);
                driveUntilCondition(new TimeDrive(0.5), 0.5, 0, 2.5, true);
                driveUntilCondition(new TimeDrive(0.5), 0.25, -90, 2.5, true);
                setWobbleGrabber(WobbleGrabber.OPEN);
                driveUntilCondition(new TimeDrive(0.1), 0.75, 0, 3, true);
                driveUntilCondition(new LineDrive(LineColor.WHITE), 0.75, -180, 2.5, true);
                break;
        }
    }

    public class VuforiaDetection extends Thread {
        public void run() {
            ringState = detectRingState(3);
        }
    }
}
