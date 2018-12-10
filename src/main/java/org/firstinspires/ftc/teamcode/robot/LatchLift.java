package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.robot.DriveTrain.GEAR_RATIO;
import static org.firstinspires.ftc.teamcode.robot.DriveTrain.INCHES_PER_WHEEL_ROTATION;
import static org.firstinspires.ftc.teamcode.robot.DriveTrain.PULSES_PER_MTR_ROTATION;

/**
 * Created by Dravid-C on 10/7/2018.
 *
 */
public class LatchLift {
    private Robot robot = null;
    private ElapsedTime runtime = new ElapsedTime();

    public DcMotor liftMotor = null;


    public Servo liftServo = null;

    public static final int PULSES_PER_MTR_ROTATION = 288;
    public static final double INCHES_PER_WHEEL_ROTATION = 4.32;
    public static final double GEAR_RATIO = 1.0; // Motor to Wheel

    public LatchLift(Robot inRobot) {


        robot = inRobot;
        liftMotor = robot.hardwareMap.get(DcMotor.class, "lift_motor");
        liftServo = robot.hardwareMap.get(Servo.class, "lift_servo");

        liftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void liftUp() {
        liftMotor.setPower(1.0);
    }

    public void liftDown() {
        liftMotor.setPower(-1.0);
    }

    public void liftStop() {
        liftMotor.setPower(0.0);
    }

    public void raiseRobot() {

    }

    public void lowerRobot() {
    }

    /**
     * Prevents the lift from lowering under the weight of the robot
     */
    public void lockLift() {
    }

    /**
     * Allows the lift to lower the robot
     */
    public void unlockLift() {

    }

    /**
     * Closes the latch that connects to the lander
     */
    public void latch() {
        liftServo.setPosition(0.0);
    }

    /**
     * Opens the latch that connects to the lander
     */
    public void unlatch() {
        liftServo.setPosition(0.85);
    }

    private void liftMotorEncoder (double distance, double power, double timeoutSec){
        double numMtrRotations = distance / (INCHES_PER_WHEEL_ROTATION * GEAR_RATIO);
        int pulses = (int) (numMtrRotations * PULSES_PER_MTR_ROTATION);

        DcMotor.RunMode prevLiftMode = liftMotor.getMode();

        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        liftMotor.setTargetPosition(pulses);

        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        runtime.reset();
        liftMotor.setPower(Math.abs(power));
        while (robot.opModeIsActive() &&
                (runtime.seconds() < timeoutSec) &&
                (liftMotor.isBusy())) {

        }

        liftMotor.setPower(0);

        liftMotor.setMode(prevLiftMode);

    }

    public void raiseRobotToReleasePin(double timeoutSec) {
        liftMotorEncoder(
                Constants.DISTANCE_LIFT_TO_DROP_PIN,
                1.0,
                timeoutSec
        );
    }

    public void lowerRobotToGround(double timeoutSec){
        liftMotorEncoder(
                Constants.DISTANCE_LIFT_TO_GROUND,
                1.0,
                timeoutSec
        );

    }

    public void retractLift(double timeoutSec){
        liftMotorEncoder(
                -Constants.DISTANCE_LIFT_TO_GROUND,
                1.0,
                timeoutSec
        );
    }

}