package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.Hardware;

public abstract class Automation extends LinearOpMode {


    //NeveRest 40 Gearbox
    private static final int encoder_tick_per_revolution = 280;

    private static final double adjust = 1; //Adjust this variable if one centimeter distance travel is not one centimeter
    static final double encoder_cm = (encoder_tick_per_revolution / (7.62 * 3.14159265358979)) * adjust;


    static final double inch_in_cm = 2.54;
    static final double one_tile = 24 * inch_in_cm;
    static final long min_delay = 50;

    //Speed variables
    static final double speed_full  = 1;
    static final double speed_half  = 0.5;
    static final double speed_slow  = 0.3;
    static final double speed_still = 0.1;

    //timeout
    static final double timeout_short  = 3;
    static final double timeout_medium = 5;
    static final double timeout_long   = 10;

    //Delays
    static final long rotate_delay = 100;
    static final long drive_delay = 100;

    //Rotation stuff
    AxesOrder rotationalAxes = AxesOrder.ZYX; //The first angle is the one that is used for computation.
    Orientation reference = new Orientation();

    //Drive straight stuff
    PID controlRotate = new PID(0.01, 0, 0);
    PID controlDrive = new PID(0.05, 0, 0);


    private ElapsedTime runtime = new ElapsedTime();

    Hardware hardware = new Hardware();


    @Override
    public final void runOpMode() throws InterruptedException {
        //Do universal init stuff
        hardware.init(hardwareMap);
        initIMU();

        //specific autonomous init code
        auto_init();

        telemetry.addData("Autonomous mode initialized", "Waiting for user start");
        telemetry.update();
        while(!isStarted() && !isStopRequested()) {
            idle();
        }

        //Run specific autonomous code
        instruction();
    }

    /*
     ** This method is where the user puts its instructions for autonomous mode.
     ** It is required since there is not point to an autonomous with no instructions
     */
    public abstract void instruction() throws InterruptedException;

    /*
     ** This method is where the user puts its custom init code.
     ** It also holds init code that is not required for each autonomous for example:
     ** Vuforia or neural networks
     */
    public void auto_init() {};

    private void initIMU() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        hardware.imu.initialize(parameters);

        while (!hardware.imu.isGyroCalibrated() && !hardware.imu.isAccelerometerCalibrated()) {
            idle();
        }
    }

    private void resetAngle() {
        reference = hardware.imu.getAngularOrientation(AxesReference.INTRINSIC, rotationalAxes, AngleUnit.DEGREES);
    }

    private double getAngle() {
        Orientation current = hardware.imu.getAngularOrientation(AxesReference.INTRINSIC, rotationalAxes, AngleUnit.DEGREES);
        return current.firstAngle -  reference.firstAngle;
    }



    /*
     **
     */
    void rotate(double angle, double power, double timeout, boolean brake) {
        resetAngle();

        controlRotate.setPoint(angle);
        controlRotate.setTolerance(0.01);
        controlRotate.setDeltaT(5);

        setBrakeMode(brake);
        if (opModeIsActive()) {
            runtime.reset();
            if (angle < 0) {
                do {
                    power = Range.clip(controlRotate.doPID(getAngle()), -1.0, 1.0) * power;
                    hardware.motor_frontLeft.setPower(power);
                    hardware.motor_frontRight.setPower(power);
                    hardware.motor_rearLeft.setPower(-power);
                    hardware.motor_rearRight.setPower(-power);
                    telemetry.addData("TEst", power);
                    telemetry.addData("TEst", getAngle());
                    telemetry.update();
                    sleep((long) controlRotate.getDeltaT());
                } while (opModeIsActive() && !controlRotate.atTarget(getAngle()) && runtime.seconds() < timeout);
            } else {
                do {
                    power = Range.clip(controlRotate.doPID(getAngle()), -1.0, 1.0) * power;
                    hardware.motor_frontLeft.setPower(-power);
                    hardware.motor_frontRight.setPower(-power);
                    hardware.motor_rearLeft.setPower(power);
                    hardware.motor_rearRight.setPower(power);
                    telemetry.addData("TEst", power);
                    telemetry.addData("TEst", getAngle());
                    telemetry.update();
                    sleep((long) controlRotate.getDeltaT());
                } while (opModeIsActive() && !controlRotate.atTarget(getAngle()) && runtime.seconds() < timeout);
            }
            setDriveMotorSpeed(0);

            sleep(rotate_delay);
        }
    }


    /*
     **
     */
    void driveIMUDistance(double distance, double angle, double power, double timeout, boolean brake) {
        angle = Range.clip(angle, -180, 180);
        power = Range.clip(Math.abs(power), 0.0, 1.0);
        Position start = hardware.imu.getPosition();

        resetAngle();

        controlDrive.setPoint(0.0);
        controlDrive.setTolerance(0.01);
        controlDrive.setDeltaT(1);

        setBrakeMode(brake);
        if (opModeIsActive()) {
            double frontLeft;
            double frontRight;
            double rearLeft;
            double rearRight;

            if (angle <= -90.0) {
                frontLeft  = -power;
                frontRight = ((-1/45) * angle - 3) * power;
                rearLeft   = ((-1/45) * angle - 3) * power;
                rearRight  = -power;
            } else if (-90.0 <= angle && angle <= 0) {
                frontLeft = ((1/45) * angle + 1) * power;
                frontRight = -power;
                rearLeft = -power;
                rearRight = ((1/45) * angle + 1) * power;
            } else if (0 <= angle && angle <= 90) {
                frontLeft = power;
                frontRight = ((1/45) * angle - 1) * power;
                rearLeft = ((1/45) * angle - 1) * power;
                rearRight = power;
            } else /*because of clip -> 90 <= angle <= 180*/{
                frontLeft = ((-1/45) * angle + 3) * power;
                frontRight = power;
                rearLeft = power;
                rearRight = ((-1/45) * angle + 3) * power;
            }

            runtime.reset();
            do {
                double correction = controlDrive.doPID(getAngle());
                hardware.motor_frontLeft.setPower(frontLeft + correction);
                hardware.motor_frontRight.setPower(frontRight + correction);
                hardware.motor_rearLeft.setPower(rearLeft - correction);
                hardware.motor_rearRight.setPower(rearRight - correction);
                sleep((long) controlDrive.getDeltaT());
            } while (opModeIsActive() && runtime.seconds() < timeout && distance(start, hardware.imu.getPosition()) <= distance);
        }
        setDriveMotorSpeed(0);

        resetAngle();
        sleep(drive_delay);
    }

    private double distance(Position start, Position end) {
        start.toUnit(DistanceUnit.CM);
        end.toUnit(DistanceUnit.CM);
        return Math.sqrt(((end.x - start.x) * (end.x - start.x))+((end.y - start.y) * (end.y - start.y))+((end.z - start.z) * (end.z - start.z)));
    }

    private void setDriveMotorMode(RunMode mode) {
        hardware.motor_frontLeft.setMode(mode);
        hardware.motor_frontRight.setMode(mode);
        hardware.motor_rearLeft.setMode(mode);
        hardware.motor_rearRight.setMode(mode);
    }

    private void setDriveMotorSpeed(double speed) {
        hardware.motor_frontLeft.setPower(speed);
        hardware.motor_frontRight.setPower(speed);
        hardware.motor_rearLeft.setPower(speed);
        hardware.motor_rearRight.setPower(speed);
    }

    void setBrakeMode(boolean brake) {
        if (brake) {
            hardware.motor_frontLeft.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
            hardware.motor_frontRight.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
            hardware.motor_rearLeft.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
            hardware.motor_rearRight.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        } else {
            hardware.motor_frontLeft.setZeroPowerBehavior(ZeroPowerBehavior.FLOAT);
            hardware.motor_frontRight.setZeroPowerBehavior(ZeroPowerBehavior.FLOAT);
            hardware.motor_rearLeft.setZeroPowerBehavior(ZeroPowerBehavior.FLOAT);
            hardware.motor_rearRight.setZeroPowerBehavior(ZeroPowerBehavior.FLOAT);
        }
    }

}
