package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Intake{
    private Robot robot = null;

    public DcMotor hexIntake = null;

    public Intake(Robot inRobot) {
        robot = inRobot;

        hexIntake = robot.hardwareMap.get(DcMotor.class, "intake_motor");

        hexIntake.setDirection(DcMotorSimple.Direction.REVERSE);

    }
    public void infeed (){
        hexIntake.setPower(1.0);
    }
    public void outfeed(){
        hexIntake.setPower(-1.0);
    }
    public void stopIntake(){
        hexIntake.setPower(0.0);
    }
}

