package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

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
    public static final double GEAR_RATIO = 0.769; // Motor to Wheel
    private static final boolean INVERT_DIRECTION_SIGN = true;

    private static final double GYRO_TURN_KP = 19.0 / 1000.0;
    private static final double GYRO_TURN_KI = 1.1 / 1000.0;
    private static final double GYRO_TURN_KD = 1.8 / 1000.0;

    public DriveTrain(Robot inRobot){
        robot = inRobot;

        leftDrive = robot.hardwareMap.get(DcMotor.class,"left_drive");
        rightDrive = robot.hardwareMap.get(DcMotor.class, "right_drive");

        leftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    public void gyroTurnKavin (double targetAngle, double timeoutSec) {
        BNO055IMU imu = robot.sensors.imu;
        runtime.reset();

        float startingHeading = getHeading(imu);
        double currentAngle = 0;
        double prevError = 0;
        double error = 0;
        double lastTime = runtime.seconds();
        double currTime = lastTime;
        double proportional = 0;
        double integral = 0;
        double derivative = 0;

        double bounded_power = 0;

        while((robot.isRunningAutonomous() && robot.currentOpMode.opModeIsActive())
                && runtime.seconds() < timeoutSec
                && !robot.valueInRange(targetAngle, 0.05, currentAngle)) {

            currTime = runtime.seconds();
            currentAngle = getHeading(imu) - startingHeading;
            error = targetAngle - currentAngle;

            proportional = GYRO_TURN_KP * error;
            integral += GYRO_TURN_KI * (error *  (currTime - lastTime));
            derivative = GYRO_TURN_KD * (error - prevError) / (currTime - lastTime);

            bounded_power = robot.boundValue(0.7, -0.7, proportional + integral + derivative);

            leftDrive.setPower(bounded_power);
            rightDrive.setPower(-bounded_power);
        }

        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }

    private float getHeading(BNO055IMU imu) {
        Orientation angles = robot.sensors.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        float unnormalized = AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle);
        return AngleUnit.DEGREES.normalize(unnormalized);
    }


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

        while (opModeIsActive() &&
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

    public void driveBackwardEncoder(double distance, double power, double timeoutSec){
        driveForwardEncoder(-distance,power , timeoutSec);
    }
    private boolean opModeIsActive() {
        if (robot.isRunningAutonomous()) {
            return robot.currentOpMode.opModeIsActive();
        }
        return true;
    }
}
