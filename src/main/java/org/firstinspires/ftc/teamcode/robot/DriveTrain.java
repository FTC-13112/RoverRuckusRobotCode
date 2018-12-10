package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Dravid-C on 10/7/2018.
 *
 */
public class DriveTrain {
    private Robot robot = null;
    private ElapsedTime runtime = new ElapsedTime();

    public DcMotor leftDrive = null;
    public DcMotor rightDrive = null;

    public static final int PULSES_PER_MTR_ROTATION = 1120;
    public static final double INCHES_PER_WHEEL_ROTATION = 12.566;
    public static final double GEAR_RATIO = 1.0; // Motor to Wheel
    private static final boolean INVERT_DIRECTION_SIGN = true;

    private static final double GYRO_TURN_KP = 12.5 / 1000.0;
    private static final double GYRO_TURN_KI = 0.35 / 1000.0;
    private static final double GYRO_TURN_KD = 0.0 / 1000.0;

    private static final double PIXY_TURN_KP = 2000.0 / 1000.0;
    private static final double PIXY_TURN_KI = 0.0 / 1000.0;
    private static final double PIXY_TURN_KD = 0.0 / 1000.0;

//kp = 7.0, ki = 0.3, kd =0.5
    public DriveTrain(Robot inRobot){
        robot = inRobot;

        leftDrive = robot.hardwareMap.get(DcMotor.class,"left_drive");
        rightDrive = robot.hardwareMap.get(DcMotor.class, "right_drive");

        leftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    /**
     * This is the message about what the funciton does.
     * You can have multiple lines and it won't care, though
     * you should have all of your parameters and your return value
     * listed below.
     *
     * @param targetAngle This is something
     * @param timeoutSec This is something else
     */
    public void gyroTurnByAngle (double targetAngle, double timeoutSec) {
        double targetHeading = robot.sensors.getHeading() + targetAngle;
        gyroTurnToHeading(-targetHeading, timeoutSec);
    }

    public void gyroTurnToHeading(double headingAngle, double timeoutSec) {
        runtime.reset();

        double currentAngle = 0;
        double prevError = 0;
        double error = 0;
        double lastTime = runtime.seconds();
        double currTime = lastTime;
        double proportional = 0;
        double integral = 0;
        double derivative = 0;

        double bounded_power = 0;

        ActiveTimeout inRangeConfirm = new ActiveTimeout(0.25);

        while((robot.isRunningAutonomous() && robot.currentOpMode.opModeIsActive())
                && runtime.seconds() < timeoutSec
                && !inRangeConfirm.checkValid(robot.valueInRange(headingAngle, 0.5, currentAngle))) {

            currTime = runtime.seconds();
            currentAngle = robot.sensors.getHeading();
            error = headingAngle - currentAngle;

            proportional = GYRO_TURN_KP * error;
            integral += GYRO_TURN_KI * (error *  (currTime - lastTime));
            derivative = GYRO_TURN_KD * (error - prevError) / (currTime - lastTime);

            bounded_power = robot.boundValue(0.5, -0.5, proportional + integral + derivative);

            leftDrive.setPower(bounded_power);
            rightDrive.setPower(-bounded_power);
        }

        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }

    /**
     * Use vision to find gold mineral angle and turn to it
     * @param timeoutSec
     */
    public MineralLocation turnToGoldMineral(double timeoutSec){
        runtime.reset();

        MineralLocation startingLocation = robot.vision.determineMineralLocation();
        double currentValue = robot.vision.getPixyXLocation();
        double prevError = 0;
        double error = 0;
        double lastTime = runtime.seconds();
        double currTime = lastTime;
        double proportional = 0;
        double integral = 0;
        double derivative = 0;

        double bounded_power = 0;

        ActiveTimeout inRangeConfirm = new ActiveTimeout(0.25);

        while((robot.isRunningAutonomous() && robot.currentOpMode.opModeIsActive())
                && runtime.seconds() < timeoutSec
                && !inRangeConfirm.checkValid(robot.valueInRange(Constants.PIXY_MIDDLE, Constants.PIXY_DEBOUNCE, currentValue))) {

            currTime = runtime.seconds();
            currentValue = robot.vision.getPixyXLocation();
            error = Constants.PIXY_MIDDLE - currentValue;

            proportional = PIXY_TURN_KP * error;
            integral += PIXY_TURN_KI * (error *  (currTime - lastTime));
            derivative = PIXY_TURN_KD * (error - prevError) / (currTime - lastTime);

            bounded_power = robot.boundValue(0.25, -0.25, proportional + integral + derivative);

            if(robot.vision.isMarkerVisible()) {
                leftDrive.setPower(bounded_power);
                rightDrive.setPower(-bounded_power);
            } else if (runtime.seconds() >= 2 && leftDrive.getPower() == 0 && rightDrive.getPower() == 0) {
                startingLocation = MineralLocation.MIDDLE;
                break;
            }
        }

        leftDrive.setPower(0);
        rightDrive.setPower(0);

        return startingLocation;
    }

    /**
     * Uses encoders on both wheels to drive a certain distance in inches
     * @param distance IN INCHES
     * @param power 0 to 1
     * @param timeoutSec
     */
    public void driveForwardEncoder(double distance, double power, double timeoutSec) {
        double numMtrRotations = distance / (INCHES_PER_WHEEL_ROTATION * GEAR_RATIO);
        int pulses =  (int) (numMtrRotations * PULSES_PER_MTR_ROTATION);

        if (INVERT_DIRECTION_SIGN) {
            pulses = -pulses;
        }

        DcMotor.RunMode prevLeftMode = leftDrive.getMode();
        DcMotor.RunMode prevRightMode = rightDrive.getMode();

        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftDrive.setTargetPosition(pulses);
        rightDrive.setTargetPosition(pulses);

        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        runtime.reset();
        leftDrive.setPower(Math.abs(power));
        rightDrive.setPower(Math.abs(power));

        while (robot.opModeIsActive() &&
                (runtime.seconds() < timeoutSec) &&
                (leftDrive.isBusy() && rightDrive.isBusy())) {
            robot.telemetry.addData("Autonomous",  "Autonomous: %s", Boolean.toString(robot.isRunningAutonomous()));
            robot.telemetry.addData("Path1",  "Running to %7d :%7d", pulses,  pulses);
            robot.telemetry.addData("Path2",  "Running at %7d :%7d",
                    leftDrive.getCurrentPosition(),
                    rightDrive.getCurrentPosition());
            robot.telemetry.update();
        }

        leftDrive.setPower(0);
        rightDrive.setPower(0);

        leftDrive.setMode(prevLeftMode);
        rightDrive.setMode(prevRightMode);
    }

    public void driveBackwardEncoder(double distance, double power, double timeoutSec) {
        driveForwardEncoder(-distance, power, timeoutSec);
    }

    public void driveForwardUntilTilted(double power, double timeoutSec){
        if (INVERT_DIRECTION_SIGN) {
            power = -Math.abs(power);
        }

        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        runtime.reset();

        leftDrive.setPower(power);
        rightDrive.setPower(power);

        while (robot.opModeIsActive() &&
                (runtime.seconds() < timeoutSec) &&
                (runtime.seconds() < 3 || !robot.sensors.robotIsTilted())) {
        }

        leftDrive.setPower(0);
        rightDrive.setPower(0);

    }
}
