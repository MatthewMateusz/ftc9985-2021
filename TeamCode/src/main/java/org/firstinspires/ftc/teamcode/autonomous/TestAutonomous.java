package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Test Autonomous", group="INDEV")
public class TestAutonomous extends Automation {

    public void instruction() throws InterruptedException {


        /*setYeeter(0.65);
        sleep(2000);
        setGate(GatePosition.OPEN);
        sleep(500);
        setConveyor(ConveyorState.ON, 0.8);
        sleep(300);
        rotate(-5, 1, 2, false);
        sleep(10);
        rotate(-5, 1, 2, true);
        sleep(500);
        setYeeter(0);
        sleep(1000);*/


        driveUntilCondition(new IMUDistance(100), 0.5, 0, 10, true);

        //driveUntilCondition(new IMUDistance(609.6), 0.5, 0, 10, true);



        /*rotate(90,0.5, 5, true);*/
        /*setConveyor(ConveyorState.ON);
        sleep(1500);
        setConveyor(ConveyorState.OFF);
        sleep(1500);
        driveUntilCondition(new IMUDistance(609.6), 0.5, 0 , 10, true);
        sleep(2000);*/
        /*driveUntilCondition(new IMUDistance(609.6), 0.5, 90, 10,  true);*/
        /*driveUntilCondition(new LineDrive(LineColor.RED), 0.5, 0, 10, true);*/

//        setDriveMotorSpeed(0.25);
//        sleep(1000);
//        setDriveMotorSpeed(0);d
    }
}
