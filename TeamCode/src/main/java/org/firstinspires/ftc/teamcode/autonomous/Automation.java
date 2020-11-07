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
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.Hardware;

public abstract class Automation extends LinearOpMode {


    //NeveRest 40 Gearbox
    private static final int encoder_tick_per_revolution = 280;

    private static final double adjust = 1; //Adjust this variable if one centimeter distance travel is not one centimeter
    static final double encoder_cm = (encoder_tick_per_revolution / (7.62 * 3.14159265358979)) * adjust;


    static final double inch_in_cm = 2.54;
    static final double one_tile_i = 24 * inch_in_cm;
    static final long min_delay = 50;
    static final double cm_to_mm = 10;
    static final double one_tile = 24 * inch_in_cm * cm_to_mm;
    static final double mmToInch = cm_to_mm * inch_in_cm;

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
    PID controlRotate = new PID(0.025, 0.01, 0); //p 0.2 | 0.00004, 0
    PID controlDrive = new PID(0.05, 0.01, 0);
    PID controlDistance = new PID(0.05, 0, 0);

    private ElapsedTime runtime = new ElapsedTime();

    Hardware hardware = new Hardware();


    //Do universal init stuff
    @Override
    public final void runOpMode() throws InterruptedException {
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

        while (!isStarted() && !isStopRequested() && !hardware.imu.isSystemCalibrated()) {
            idle();
        }
    }

    private void resetAngle() {
        reference = hardware.imu.getAngularOrientation(AxesReference.EXTRINSIC, rotationalAxes, AngleUnit.DEGREES);
    }

    private double getAngle() {
        Orientation current = hardware.imu.getAngularOrientation(AxesReference.EXTRINSIC, rotationalAxes, AngleUnit.DEGREES);
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
                    power = controlRotate.doPID(getAngle());
                    hardware.motor_frontLeft.setPower(power);
                    hardware.motor_frontRight.setPower(-power);
                    hardware.motor_rearLeft.setPower(power);
                    hardware.motor_rearRight.setPower(-power);
                    sleep((long) controlRotate.getDeltaT());
                } while (opModeIsActive() && !controlRotate.atTarget(getAngle()) && runtime.seconds() < timeout);
            } else {
                do {
                    power = controlRotate.doPID(getAngle());
                    hardware.motor_frontLeft.setPower(-power);
                    hardware.motor_rearLeft.setPower(-power);
                    hardware.motor_frontRight.setPower(power);
                    hardware.motor_rearRight.setPower(power);
                    sleep((long) controlRotate.getDeltaT());
                } while (opModeIsActive() && !controlRotate.atTarget(getAngle()) && runtime.seconds() < timeout);
            }
            setDriveMotorSpeed(0);

            sleep(rotate_delay);
        }
    }

    /* Custom condition to drive to instead of creating a new function every single time */
    class IMUDistance implements Condition {
        Position start;
        double targetDistance;
        double tolerance = 0.02;

        public IMUDistance(double distance) {
            targetDistance = distance;
        }

        public void start() {
            hardware.imu.startAccelerationIntegration(hardware.imu.getPosition(), new Velocity(), 5);
            start = hardware.imu.getPosition();

            controlDistance.setPoint(targetDistance);
            controlDistance.setDeltaT(5);
        }

        public boolean atCondition() {
            return (distance(start, hardware.imu.getPosition()) - targetDistance)/1000 <= tolerance;
        }

        public double correction() {
            return controlDistance.doPID(distance(start, hardware.imu.getPosition()));
        }

        public void end() {
            hardware.imu.stopAccelerationIntegration();
        }

        private double distance(Position start, Position end) {
            start.toUnit(DistanceUnit.CM);
            end.toUnit(DistanceUnit.CM);
            return Math.sqrt(((end.x - start.x) * (end.x - start.x))+((end.y - start.y) * (end.y - start.y))+((end.z - start.z) * (end.z - start.z)));
        }
    }

    enum LineColor {
        RED,
        BLUE,
        WHITE
    }

    class LineDrive implements Condition {
        LineColor target;
        static final int red = 400;
        static final int green = 400;
        static final int blue = 400;
        static final float ratio = 1.2f;


        public LineDrive(LineColor lineColor) {
            target = lineColor;
        }

        public void start() {

        }

        public boolean atCondition() {
            switch(target) {
                case RED:
                    return hardware.colorSensor_Down.red() >= red &&
                            ( (float) hardware.colorSensor_Down.red() / (float) hardware.colorSensor_Down.green()) > ratio &&
                            ( (float) hardware.colorSensor_Down.red() / (float) hardware.colorSensor_Down.blue()) > ratio;

                case BLUE:
                    return hardware.colorSensor_Down.blue() >= blue &&
                            ( (float) hardware.colorSensor_Down.blue() / (float) hardware.colorSensor_Down.red()) > ratio &&
                            ( (float) hardware.colorSensor_Down.blue() / (float) hardware.colorSensor_Down.green()) > ratio;

                case WHITE:
                    return hardware.colorSensor_Down.red() >= red &&
                            hardware.colorSensor_Down.green() >= green &&
                            hardware.colorSensor_Down.blue() >= blue;

                default:
                    return true;
            }
        }

        public double correction() {
            return 0;
        }

        public void end() {

        }
    }



    /*
    ** Drive until meet a condition
    */
    interface Condition {
        void start();
        boolean atCondition();
        void end();
        double correction();
    }
    void driveUntilCondition(Condition condition, double angle, double power, double timeout, boolean brake) {
        angle = Range.clip(angle, -180, 180);
        power = Range.clip(Math.abs(power), 0.0, 1.0);
        condition.start();

        resetAngle();

        controlDrive.setPoint(0.0);
        controlDrive.setTolerance(0.01);
        controlDrive.setDeltaT(5);

        setBrakeMode(brake);
        if (opModeIsActive()) {
            double frontLeft;
            double frontRight;
            double rearLeft;
            double rearRight;

            if (angle <= -90.0) {
                frontLeft = -power;
                frontRight = ((1./45.) * angle + 3.) * power;
                rearLeft = ((1./45.) * angle + 3.) * power;
                rearRight = -power;

                /*frontLeft  = ((1./45.) * angle + 3.) * power;
                frontRight = -power;
                rearLeft   = ((1./45.) * angle + 3.) * power;
                rearRight  = -power;*/
            } else if (angle <= 0) {
                frontLeft = ((1./45.) * angle + 1.) * power;
                frontRight = power;
                rearLeft = power;
                rearRight = ((1./45.) * angle + 1.) * power;

                /*frontLeft = power;
                frontRight = ((1./45.) * angle + 1.) * power;
                rearLeft = power;
                rearRight = ((1./45.) * angle + 1.) * power;*/
            } else if (angle <= 90) {
                frontLeft = power;
                frontRight = ((-1./45.) * angle + 1) * power;
                rearLeft =  ((-1./45.) * angle + 1) * power;
                rearRight = power;

                /*frontLeft = ((-1./45.) * angle + 1.) * power;
                frontRight = power;
                rearLeft = ((-1./45.) * angle + 1.) * power;
                rearRight = power;*/
            } else /*because of clip angle <= 180*/{
                frontLeft = ((-1./45.) * angle + 3) * power;
                frontRight = -power;
                rearLeft = -power;
                rearRight = ((-1./45.) * angle + 3) * power;

                /*frontLeft = -power;
                frontRight = ((-1./45.) * angle + 3.) * power;
                rearLeft = -power;
                rearRight = ((-1./45.) * angle + 3.) * power;*/
            }

            runtime.reset();
            do {
                double correction = controlDrive.doPID(getAngle());
                hardware.motor_frontLeft.setPower(frontLeft + correction + condition.correction());
                hardware.motor_frontRight.setPower(frontRight - correction + condition.correction());
                hardware.motor_rearLeft.setPower(rearLeft + correction + condition.correction());
                hardware.motor_rearRight.setPower(rearRight - correction + condition.correction());
                sleep((long) controlDrive.getDeltaT());
            } while (opModeIsActive() && runtime.seconds() < timeout && condition.atCondition());
        }
        setDriveMotorSpeed(0.0);
        condition.end();

        resetAngle();
        sleep(drive_delay);
    }


    private void setDriveMotorMode(RunMode mode) {
        hardware.motor_frontLeft.setMode(mode);
        hardware.motor_frontRight.setMode(mode);
        hardware.motor_rearLeft.setMode(mode);
        hardware.motor_rearRight.setMode(mode);
    }

    void setDriveMotorSpeed(double speed) {
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


/*
 **
 *//*
    void driveIMUDistance(double distance, double angle, double power, double timeout, boolean brake) {
        angle = Range.clip(angle, -180, 180);
        power = Range.clip(Math.abs(power), 0.0, 1.0);
        hardware.imu.startAccelerationIntegration(hardware.imu.getPosition(), new Velocity(), 5);
        Position start = hardware.imu.getPosition();

        resetAngle();

        controlDrive.setPoint(0.0);
        controlDrive.setTolerance(0.01);
        controlDrive.setDeltaT(5);

        setBrakeMode(brake);
        if (opModeIsActive()) {
            double frontLeft;
            double frontRight;
            double rearLeft;
            double rearRight;

            if (angle <= -90.0) {
                frontLeft = -power;
                frontRight = ((1./45.) * angle + 3.) * power;
                rearLeft = ((1./45.) * angle + 3.) * power;
                rearRight = -power;

                *//*frontLeft  = ((1./45.) * angle + 3.) * power;
                frontRight = -power;
                rearLeft   = ((1./45.) * angle + 3.) * power;
                rearRight  = -power;*//*
            } else if (angle <= 0) {
                frontLeft = ((1./45.) * angle + 1.) * power;
                frontRight = power;
                rearLeft = power;
                rearRight = ((1./45.) * angle + 1.) * power;

                *//*frontLeft = power;
                frontRight = ((1./45.) * angle + 1.) * power;
                rearLeft = power;
                rearRight = ((1./45.) * angle + 1.) * power;*//*
            } else if (angle <= 90) {
                frontLeft = power;
                frontRight = ((-1./45.) * angle + 1) * power;
                rearLeft =  ((-1./45.) * angle + 1) * power;
                rearRight = power;

                *//*frontLeft = ((-1./45.) * angle + 1.) * power;
                frontRight = power;
                rearLeft = ((-1./45.) * angle + 1.) * power;
                rearRight = power;*//*
            } else *//*because of clip angle <= 180*//*{
                frontLeft = ((-1./45.) * angle + 3) * power;
                frontRight = -power;
                rearLeft = -power;
                rearRight = ((-1./45.) * angle + 3) * power;

                *//*frontLeft = -power;
                frontRight = ((-1./45.) * angle + 3.) * power;
                rearLeft = -power;
                rearRight = ((-1./45.) * angle + 3.) * power;*//*
            }

            runtime.reset();
            do {
                double correction = controlDrive.doPID(getAngle());
                hardware.motor_frontLeft.setPower(frontLeft + correction);
                hardware.motor_frontRight.setPower(frontRight - correction);
                hardware.motor_rearLeft.setPower(rearLeft + correction);
                hardware.motor_rearRight.setPower(rearRight - correction);
                sleep((long) controlDrive.getDeltaT());
            } while (opModeIsActive() && runtime.seconds() < timeout && distance(start, hardware.imu.getPosition()) <= distance);
        }
        setDriveMotorSpeed(0);
        hardware.imu.stopAccelerationIntegration();

        resetAngle();
        sleep(drive_delay);
    }*/
