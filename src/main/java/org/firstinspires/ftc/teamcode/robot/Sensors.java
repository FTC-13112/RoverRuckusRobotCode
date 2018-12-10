package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

/**
 * Created by Dravid-C on 10/7/2018.
 *
 */
public class Sensors {
    private Robot robot = null;

    public BNO055IMU imu = null;
    public AnalogInput pixy_x_location = null;
    public DigitalChannel pixy_visible = null;
    Acceleration gravity;



    public Sensors(Robot inRobot){
        robot = inRobot;
        imu = robot.hardwareMap.get(BNO055IMU.class, "imu");
        initializeIMU();
        pixy_x_location = robot.hardwareMap.get(AnalogInput.class, "pixy");
        pixy_visible = robot.hardwareMap.get(DigitalChannel.class, "pixy_visible");
        pixy_visible.setMode(DigitalChannel.Mode.INPUT);

    }
    private void initializeIMU () {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu.initialize(parameters);
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

    }

    public boolean robotIsTilted(){
        return Math.abs(imu.getGravity().yAccel) > 0.8;
    }
    public float getHeading() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        float unnormalized = AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle);
        return AngleUnit.DEGREES.normalize(unnormalized);
    }

    public double addToHeading(double degrees) {
        // Everything is backwards
        return getHeading() - degrees;
    }

    public double addToHeading(double startingHeading, double degrees) {
        // Everything is backwards
        return startingHeading   - degrees;
    }


}

