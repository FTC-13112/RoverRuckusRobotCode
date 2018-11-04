package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.Robot;

/**
 * Created by Dravid-C on 10/7/2018.
 *
 */
@Autonomous(name="Basic Autonomous", group="Testing")
public class BasicAutonomous extends LinearOpMode {
    private Robot robot = null;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(this, hardwareMap, telemetry);
        robot.output.holdMarker();

        waitForStart();
        robot.driveTrain.driveForwardEncoder(48.0, 0.5, 10.0);
        robot.output.dropMarker();
        robot.output.holdMarker();
    }
}
