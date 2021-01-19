package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Pushbot: Teleop Tank", group="Pushbot")
//@Disabled
public class Telop extends OpMode {
    /* Declare OpMode members. */
    Hardware robot = new Hardware(); // use the class created to define a Pushbot's hardware
    boolean LauncherChange = false; //Luancher
    boolean LauncherChangeSlow = false;
    boolean MotorChange = false; //motor collector
    boolean LaunchChange = false;
    boolean GateChange = false;
    boolean ConveyorChange = false;


    //final double    servoSpeedH   = 0.004;
    //final double    servoSpeedV   = 0.004;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "No one out pizzas the hut, good luck"); //
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }
    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */


    @Override
    public void loop() {
        double leftFrontSpeed;
        double rightFrontSpeed;
        double leftRearSpeed;
        double rightRearSpeed;

//add this vairable.  "CC" stands for controller coefficient and is used with mecanum wheels so the x and y sticks don't add up to more than 1 for a pair of motors (this would be bad since the motor value is limited to 1.  The motors moving in the opposing direction would not be limited
        double CC = .7;

        //if (gamepad1.a)
          //  robot.motor_collector.setPower(.7);
        //else if (gamepad1.y)
          //  robot.motor_collector.setPower(0);
        //Collector
        if(gamepad1.a && !MotorChange) {
            if(robot.motor_collector.getPower() == 0) {
                robot.motor_collector.setPower(1);
                robot.motor_conveyor.setPower(1);
                robot.servo_Advancer.setPower(0.5);
            }else {
                robot.motor_collector.setPower(0);
                robot.motor_conveyor.setPower(1);
                robot.servo_Advancer.setPower(0.05);
            }
            MotorChange = true;
        } else if(!gamepad1.a) MotorChange = false;

        if(gamepad1.b && !MotorChange) {
            if(robot.motor_collector.getPower() == 0) {
                robot.motor_collector.setPower(-1);
                robot.motor_conveyor.setPower(-1);
                robot.servo_Advancer.setPower(-0.5);
            }else {
                robot.motor_collector.setPower(0);
                robot.motor_conveyor.setPower(-1);
                robot.servo_Advancer.setPower(-0.05);
            }
            MotorChange = true;
        } else if(!gamepad1.b) MotorChange = false;

        if(gamepad1.y && !MotorChange) {
            if(robot.motor_collector.getPower() == 0) {
                robot.motor_collector.setPower(0);
                robot.motor_conveyor.setPower(0);
                robot.servo_Advancer.setPower(-0);
            }else {
                robot.motor_collector.setPower(0);
                robot.motor_conveyor.setPower(0);
                robot.servo_Advancer.setPower(0);
            }
            MotorChange = true;
        } else if(!gamepad1.y) MotorChange = false;


        if(gamepad2.a && !GateChange) {
            if(robot.servo_gate.getPosition() == 1) {
                robot.servo_gate.setPosition(.5);
            } else {
                robot.servo_gate.setPosition(1);
            }
            GateChange = true;
        } else if(!gamepad2.a) GateChange = false;

        if (gamepad2.b)
            robot.servo_grabber.setPosition(1);
        else
            robot.servo_grabber.setPosition(0);


        if(gamepad1.right_bumper && !LauncherChange) {
            if(robot.motor_launch.getPower() == 0)
                robot.motor_launch.setPower(1);

            else
                robot.motor_launch.setPower(0);
            LauncherChange = true;
        } else if(!gamepad1.right_bumper) LauncherChange = false;

        if(gamepad1.left_bumper && !LauncherChangeSlow) {
            if(robot.motor_launch.getPower() == 0)
                robot.motor_launch.setPower(.60);

            else
                robot.motor_launch.setPower(0);
            LauncherChangeSlow = true;
        } else if(!gamepad1.left_bumper) LauncherChangeSlow = false;


        if (gamepad2.dpad_up && !robot.pressed(robot.touch_lift_up))
            robot.motor_lift.setPower(.6);
        else if (gamepad2.dpad_down && !robot.pressed(robot.touch_lift_down))
            robot.motor_lift.setPower(-.6);


        // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)

//This code is for mecanum wheels mounted out the front and rear of robot (team 9985)
        leftFrontSpeed = (-gamepad1.left_stick_y*(CC) + gamepad1.left_stick_x*(CC) - gamepad1.left_trigger*(CC) + gamepad1.right_trigger*(CC));
        rightFrontSpeed = (-gamepad1.left_stick_y*(CC) - gamepad1.left_stick_x*(CC) + gamepad1.left_trigger*(CC) - gamepad1.right_trigger*(CC));
        leftRearSpeed = (-gamepad1.left_stick_y*(CC) - gamepad1.left_stick_x*(CC) - gamepad1.left_trigger*(CC) + gamepad1.right_trigger*(CC));
        rightRearSpeed = (-gamepad1.left_stick_y*(CC) + gamepad1.left_stick_x*(CC) + gamepad1.left_trigger*(CC) - gamepad1.right_trigger*(CC));


        robot.motor_frontLeft.setPower(leftFrontSpeed);
        robot.motor_frontRight.setPower(rightFrontSpeed);
        robot.motor_rearLeft.setPower(leftRearSpeed);
        robot.motor_rearRight.setPower(rightRearSpeed);

        float hsvValues_RF[] = {0F, 0F, 0F};
        float hsvValues_LF[] = {0F, 0F, 0F};
//
//        // values is a reference to the hsvValues array.
        final float values_RF[] = hsvValues_RF;
        final float values_LF[] = hsvValues_LF;
//         sometimes it helps to multiply the raw RGB values with a scale factor
//         to amplify/attentuate the measured values.
        final double SCALE_FACTOR = 255;
        Color.RGBToHSV((int) (robot.colorSensor_Down.red() * SCALE_FACTOR),
                (int) (robot.colorSensor_Down.green() * SCALE_FACTOR),
                (int) (robot.colorSensor_Down.blue() * SCALE_FACTOR),hsvValues_RF);


        telemetry.addData("LeftFront:",leftFrontSpeed);
        telemetry.addData("RightFront:",rightFrontSpeed);
        telemetry.addData("LeftRear:",leftRearSpeed);
        telemetry.addData("RightRear:",rightRearSpeed);
        telemetry.addData("Color Sensor B:", robot.colorSensor_Down.blue());
        telemetry.addData("Color Sensor R:", robot.colorSensor_Down.red());
        telemetry.addData("Color Sensor G:", robot.colorSensor_Down.green());
        telemetry.update();
//This code is for mecanum wheels mounted out the sides of robot (team 11283)



        // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
        //if (gamepad1.left_stick_y < 0) {
        // leftFrontSpeed = (gamepad1.left_stick_y + gamepad1.left_trigger - gamepad1.right_trigger);
        //rightFrontSpeed = (-gamepad1.left_stick_y + gamepad1.left_trigger - gamepad1.right_trigger);
        //leftRearSpeed = (gamepad1.left_stick_y - gamepad1.left_trigger + gamepad1.right_trigger);
        //rightRearSpeed = (-gamepad1.left_stick_y - gamepad1.left_trigger + gamepad1.right_trigger);

        //robot.servo_frontLeft.setPosition(.5);
        //robot.servo_frontRight.setPosition(.5);
        //robot.servo_rearRight.setPosition(.5);
        //robot.servo_rearLeft.setPosition(.5);
    }
    //else if (gamepad1.left_stick_y > 0) {
    //leftFrontSpeed = (gamepad1.left_stick_y + gamepad1.left_trigger - gamepad1.right_trigger);
    //rightFrontSpeed = (-gamepad1.left_stick_y + gamepad1.left_trigger - gamepad1.right_trigger);
    //leftRearSpeed = (gamepad1.left_stick_y - gamepad1.left_trigger + gamepad1.right_trigger);
    //rightRearSpeed = (-gamepad1.left_stick_y - gamepad1.left_trigger + gamepad1.right_trigger);

    //robot.servo_frontLeft.setPosition(.5);
    //robot.servo_frontRight.setPosition(.5);
    //robot.servo_rearRight.setPosition(.5);
    //robot.servo_rearLeft.setPosition(.5);
    //}
    //else if (gamepad1.left_stick_x > 0) {
    //leftFrontSpeed = (-gamepad1.left_stick_x + gamepad1.left_trigger - gamepad1.right_trigger);
    //rightFrontSpeed = (-gamepad1.left_stick_x + gamepad1.left_trigger - gamepad1.right_trigger);
    //leftRearSpeed = (gamepad1.left_stick_x + gamepad1.left_trigger - gamepad1.right_trigger);
    //rightRearSpeed = (gamepad1.left_stick_x + gamepad1.left_trigger - gamepad1.right_trigger);

    //}

    //else if (gamepad1.left_stick_x < 0) {
    //  leftFrontSpeed = (-gamepad1.left_stick_x + gamepad1.left_trigger - gamepad1.right_trigger);
    //rightFrontSpeed = (-gamepad1.left_stick_x + gamepad1.left_trigger - gamepad1.right_trigger);
    //leftRearSpeed = (gamepad1.left_stick_x + gamepad1.left_trigger - gamepad1.right_trigger);
    //rightRearSpeed = (gamepad1.left_stick_x + gamepad1.left_trigger - gamepad1.right_trigger);

    //}
    //else {
    //leftFrontSpeed = (gamepad1.left_trigger - gamepad1.right_trigger);
    //rightFrontSpeed = (gamepad1.left_trigger - gamepad1.right_trigger);
    //leftRearSpeed = (gamepad1.left_trigger - gamepad1.right_trigger);
    //rightRearSpeed = (gamepad1.left_trigger - gamepad1.right_trigger);

    //robot.servo_frontLeft.setPosition(1);
    //robot.servo_frontRight.setPosition(0);
    //robot.servo_rearRight.setPosition(0);
    //robot.servo_rearLeft.setPosition(1);
    //}






    //Servo position 1 is left and servo position right is 0 and servo straight is 0.5
    //if (gamepad1.x){
    //  robot.leftRearServo.setPosition(0);
    //}   else if (gamepad1.y) {
    //  robot.leftRearServo.setPosition(0.1);

    //}
    //else {

    //robot.motor_frontLeft.setPower(leftFrontSpeed);
    //robot.motor_frontRight.setPower(rightFrontSpeed);
    //robot.motor_rearLeft.setPower(leftRearSpeed);
    //robot.motor_rearRight.setPower(rightRearSpeed);

        /*robot.servo_frontLeft.setPosition(-.5*gamepad1.right_stick_x+.5);
        robot.servo_rearRight.setPosition(-.5*gamepad1.right_stick_x+.5);
        robot.servo_rearLeft.setPosition(-.5*gamepad1.right_stick_x+.5);
        robot.servo_frontRight.setPosition(-.5*gamepad1.right_stick_x+.5);*/

    //Moves front servos
    //if (gamepad2.dpad_left)
    //  servoOffsetH += servoSpeedH;
    //else if (gamepad2.dpad_right)
    //  servoOffsetH -= servoSpeedH;
    //if (gamepad2.dpad_up)
    //  servoOffsetV += servoSpeedV;
    //else if (gamepad2.dpad_down)
    //servoOffsetV -= servoSpeedV;

    // Move both servos to new position.  Assume servos are mirror image of each other.
    //servoOffsetH = Range.clip(servoOffsetH, -0.5, 0.5);
    //servoOffsetV = Range.clip(servoOffsetV, -.5, 0.5);

    //robot.grabberVertServo.setPosition(robot.steeringstriaght - servoOffset);
//}

    // Send telemetry message to signify robot running;
    // telemetry.addData("vert Servo",  "position = %.2f",robot.steeringstriaght + servoOffsetV);
    //telemetry.addData("Servo Offset H","Offset H = %.2f", servoOffsetH);
    //telemetry.addData("Servo Offset V","Offset V = %.2f", servoOffsetV);

    //telemetry.addData("left",  "%.2f", left);
    //telemetry.update();

    //}

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}