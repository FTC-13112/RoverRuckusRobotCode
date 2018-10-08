package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.Robot;

/**
 * Created by Dravid-C on 10/7/2018.
 * lol lets get it
 */
@Autonomous(name="Basic Autonomous", group="Testing")
public class BasicAutonomous extends LinearOpMode {
    private Robot robot = null;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(this, hardwareMap, telemetry);
        waitForStart();
        robot.driveTrain.driveForwardEncoder(20.0, 0.25, 5.0);
        robot.driveTrain.driveBackwardEncoder(30.0, 0.25, 10.0);
    }
}
