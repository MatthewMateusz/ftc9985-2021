package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Main Autonomous", group="INDEV")
public class MainAutonomous extends Automation {

    public void instruction() throws InterruptedException {
        setYeeter(0.65);
        sleep(2000);
        setGate(GatePosition.OPEN);
        sleep(500);
        setConveyor(ConveyorState.ON, 0.8);
        sleep(100);
        rotate(-5, 1, 2, false);
        sleep(10);
        rotate(-5, 1, 2, true);
        sleep(500);
        setYeeter(0);

        setConveyor(ConveyorState.OFF, 0.05);
        setGate(GatePosition.CLOSE);
        rotate(10, 1, 2, true);
        /*driveUntilCondition(new LineDrive(LineColor.WHITE), 1, -41.5, 10, true);
        driveUntilCondition(new TimeDrive(1), 0.75, -180, 3, true);
        driveUntilCondition(new LineDrive(LineColor.WHITE), 1, 60, 5, true);*/

        /*driveUntilCondition(new LineDrive(LineColor.WHITE), 1, -22, 5, false);
        driveUntilCondition(new TimeDrive(1), 1, -22, 5, true);
        driveUntilCondition(new TimeDrive(1), 1, -180, 5, true);*/

        driveUntilCondition(new LineDrive(LineColor.WHITE), 1, -30, 10, false);
        driveUntilCondition(new TimeDrive(4), 1, -50, 10, true);
        driveUntilCondition(new TimeDrive(4), 1, -180, 10, true);
    }

    public class VuforiaDetection implements Runnable {
        public void run() {

        }
    }
}
