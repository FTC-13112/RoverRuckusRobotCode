package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Dravid-C on 10/7/2018.
 *
 */
public class DriveTrain {
    private Robot robot = null;
    public DcMotor leftDrive = null;
    public DcMotor rightDrive = null;

    public DriveTrain(Robot inRobot){
        robot = inRobot;

        leftDrive = robot.hardwareMap.get(DcMotor.class,"left_drive");
        rightDrive = robot.hardwareMap.get(DcMotor.class, "right_drive");
    }
}
