package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Robot;

/**
 * Created by Dravid-C on 10/7/2018.
 * lol lets get it
 */
@TeleOp(name="Basic Teleop", group="Testing")
public class BasicTeleop extends OpMode {
    private Robot robot = null;

    @Override
    public void init() {
        robot = new Robot(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        robot.driveTrain.leftDrive.setPower(gamepad1.left_stick_y);
        robot.driveTrain.rightDrive.setPower(gamepad1.right_stick_y);

        robot.telemetry.addData("LeftY",  "LeftStick :%7f", gamepad1.left_stick_y);
    }
}
