package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.robot.Robot;

/**
 * Created by Dravid-C on 11/25/2018.
 */
@TeleOp(name = "pixy test")
public class pixy_test extends LinearOpMode {

    public Robot robot = null;

    private double pixyValue = 0;

    private boolean pixyVisible = false;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(this, hardwareMap, telemetry);

        composeTelemetry();

        waitForStart();

        while(opModeIsActive()){
            telemetry.update();

        }

    }

    private void composeTelemetry(){
        telemetry.addAction(new Runnable() {
            @Override public void run() {
                // Acquiring the angles is relatively expensive; we don't want
                // to do that in each of the three items that need that info, as that's
                // three times the necessary expense.
                pixyValue = robot.sensors.pixy_x_location.getVoltage();
                pixyVisible = robot.vision.isMarkerVisible();

            }
        });

        telemetry.addLine()
                .addData("pixy X voltage", new Func<String>() {
                    @Override public String value() {
                        return Double.toString(pixyValue);
                    }
                });

        telemetry.addLine()
                .addData("pixy marker visible", new Func<String>() {
                    @Override public String value() {
                        return Boolean.toString(pixyVisible);
                    }
                });

    }
}
