package org.firstinspires.ftc.teamcode.autonomous;

import android.graphics.drawable.GradientDrawable;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name="Test Autonomous", group="INDEV")
public class TestAutonomous extends Automation {

    public void instruction() throws InterruptedException {

        //rotate(180, 1, 5, true);
        driveUntilCondition(new IMUDistance(609.6), 0, 0.5, 10, true);
        sleep(2000);
        driveUntilCondition(new IMUDistance(609.6), 90, 0.5, 10,  true);
        driveUntilCondition(new LineDrive(LineColor.RED), 0, 0.5, 10, true);
        /*driveUntilCondition(new IMUDistance(250), -180, 0.5, 10, true);
        driveUntilCondition(new IMUDistance(250), -90, 0.5, 10, true);*/

//        setDriveMotorSpeed(0.25);
//        sleep(1000);
//        setDriveMotorSpeed(0);
    }
}
