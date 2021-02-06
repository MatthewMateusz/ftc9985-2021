package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Main Autonomous", group="INDEV")
public class BetterMainAutonomous extends Automation {

    RingState ringState = RingState.NONE;

    moveRight moveRight1 = new moveRight();
    moveRight moveRight2 = new moveRight();

    public void auto_init() {
        ringState = detectRingState(3);
    }

    public void instruction() throws InterruptedException {
        setWobbleGrabber(WobbleGrabber.CLOSE);

        setYeeter(0.62);
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

        driveUntilCondition(new LineDrive(LineColor.WHITE), 1, 0, 10, true);
        setYeeter(0.8);
        sleep(2000);

        //Power shot #1
        setGate(GatePosition.OPEN);
        sleep(500);
        setConveyor(ConveyorState.ON, 0.8);
        sleep(200);
        setConveyor(ConveyorState.OFF, 0.05);
        setGate(GatePosition.CLOSE);

        //Power shot #2
        moveRight1.run();
        sleep(200);
        setConveyor(ConveyorState.ON, 0.8);
        sleep(500);
        setConveyor(ConveyorState.OFF, 0.05);
        moveRight1.join();
        setGate(GatePosition.OPEN);
        sleep(500);
        setConveyor(ConveyorState.ON, 0.8);
        sleep(200);
        setConveyor(ConveyorState.OFF, 0.05);
        setGate(GatePosition.CLOSE);

        //Power shot #3
        moveRight2.run();
        sleep(200);
        setConveyor(ConveyorState.ON, 0.8);
        sleep(500);
        setConveyor(ConveyorState.OFF, 0.05);
        moveRight2.join();
        setGate(GatePosition.OPEN);
        sleep(500);
        setConveyor(ConveyorState.ON, 0.8);
        sleep(200);
        setConveyor(ConveyorState.OFF, 0.05);
        setGate(GatePosition.CLOSE);



        switch (ringState) {
            case NONE:
                driveUntilCondition(new TimeDrive(3), 1, -90, 5, true);
                setWobbleGrabber(WobbleGrabber.OPEN);
                driveUntilCondition(new TimeDrive(0.1), 0.75, 0, 3, true);
                driveUntilCondition(new TimeDrive(1), 0.75, -180, 3, true);
                driveUntilCondition(new LineDrive(LineColor.WHITE), 1, 60, 5, true);
                break;

            case ONE:
                driveUntilCondition(new TimeDrive(1), 1, -45, 5, true);
                setWobbleGrabber(WobbleGrabber.OPEN);
                driveUntilCondition(new TimeDrive(0.1), 0.75, 0, 3, true);
                driveUntilCondition(new TimeDrive(1), 1, -180, 5, true);
                break;

            case QUAD:
                driveUntilCondition(new TimeDrive(2), 1, -45, 2.5, true);
                setWobbleGrabber(WobbleGrabber.OPEN);
                driveUntilCondition(new TimeDrive(0.1), 0.75, 0, 3, true);
                driveUntilCondition(new LineDrive(LineColor.WHITE), 0.75, -180, 2.5, true);
                break;
        }
    }

    public class moveRight extends Thread {
        public void run() {
            driveUntilCondition(new TimeDrive(0.5), 0.7, 90, 2, true);
        }
    }
}
