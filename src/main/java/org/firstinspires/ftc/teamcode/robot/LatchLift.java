package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Dravid-C on 10/7/2018.
 *
 */
public class LatchLift {
    private Robot robot = null;
    private ElapsedTime runtime = new ElapsedTime();

    public DcMotor liftMotor = null;

    public CRServo liftServo = null;

    public LatchLift(Robot inRobot){

        robot = inRobot;
        liftMotor = robot.hardwareMap.get(DcMotor.class, "lift_motor");
        liftServo = robot.hardwareMap.get(CRServo.class, "lift_servo");

        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        liftServo.setDirection(CRServo.Direction.FORWARD);

    }
}
