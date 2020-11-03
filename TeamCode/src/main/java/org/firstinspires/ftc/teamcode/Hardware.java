package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Hardware {

    //Motors
    public DcMotor motor_frontLeft = null;
    public DcMotor motor_frontRight = null;
    public DcMotor motor_rearLeft = null;
    public DcMotor motor_rearRight = null;
    public DcMotor motor_collector = null;
    public DcMotor motor_lift = null;
    public DcMotor motor_launch = null;
    public DcMotor motor_conveyor = null;

    //Servos
    public CRServo servo_left_cont = null;
    public Servo servo_grabber = null;
    public Servo servo_gate = null;
    public Servo grabber_elbow = null;
    //Sensors
    public BNO055IMU imu;

    public DigitalChannel touch_lift_down = null;
    public DigitalChannel touch_lift_up = null;
    public ColorSensor colorSensor_Down = null;
    public void init(HardwareMap hwMap) {
        motor_frontLeft = setupMotor(hwMap, "motor_frontLeft", Direction.FORWARD, ZeroPowerBehavior.BRAKE);
        motor_frontRight = setupMotor(hwMap, "motor_frontRight", Direction.FORWARD, ZeroPowerBehavior.BRAKE);
        motor_rearLeft = setupMotor(hwMap, "motor_rearLeft", Direction.REVERSE, ZeroPowerBehavior.BRAKE);
        motor_rearRight = setupMotor(hwMap, "motor_rearRight", Direction.REVERSE, ZeroPowerBehavior.BRAKE);
        motor_collector = setupMotor(hwMap, "motor_Collector", Direction.REVERSE, ZeroPowerBehavior.BRAKE);
        motor_lift = setupMotor(hwMap, "motor_Lift", Direction.REVERSE, ZeroPowerBehavior.BRAKE);
        motor_launch = setupMotor(hwMap, "motor_Launch", Direction.REVERSE, ZeroPowerBehavior.BRAKE);
        motor_conveyor = setupMotor(hwMap, "motor_Conveyor", Direction.REVERSE, ZeroPowerBehavior.BRAKE);

        servo_left_cont = setupContinuousServo(hwMap, "servo_left_cont", -0.5);
        grabber_elbow = setupServo(hwMap,"servo_grabberElbow",.5);
        servo_gate= setupServo(hwMap,"servo_Gate",.5);
        servo_grabber = setupServo(hwMap,"servo_Grabber",.5);

        touch_lift_down = hwMap.get(DigitalChannel.class, "touch_front");
        touch_lift_up = hwMap.get(DigitalChannel.class, "touch_rear");
        colorSensor_Down = hwMap.get(ColorSensor.class,"color_Down");

        imu = hwMap.get(BNO055IMU.class, "imu");
    }

    private DcMotor setupMotor(HardwareMap hwMap, String phoneName, DcMotor.Direction motorDirection, DcMotor.ZeroPowerBehavior zeroPower) {
        DcMotor motor = hwMap.get(DcMotor.class, phoneName);
        motor.setDirection(motorDirection);
        motor.setPower(0);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(zeroPower);
        return motor;
    }

    private Servo setupServo(HardwareMap hwMap, String phoneName, double position) {
        Servo servo = hwMap.get(Servo.class, phoneName);
        servo.setPosition(position);
        return servo;

    }

    private CRServo setupContinuousServo(HardwareMap hwMap, String phoneName, double speed){
        CRServo servo = hwMap.crservo.get(phoneName);
        servo.setPower(speed);
        return servo;
    }

    public boolean pressed(DigitalChannel touchsensor) {
        return !touchsensor.getState();
    }
}
