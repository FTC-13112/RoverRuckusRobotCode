package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Intake{
    private Robot robot = null;

    public DcMotor hexIntake = null;
    public DcMotor xRailLift = null;
    public DcMotor xRailExtend = null;

    public Intake(Robot inRobot) {
        robot = inRobot;

        hexIntake = robot.hardwareMap.get(DcMotor.class, "intake_motor");
        xRailLift = robot.hardwareMap.get(DcMotor.class, "xrail_lift");
        xRailExtend = robot.hardwareMap.get(DcMotor.class, "xrail_extend");

        hexIntake.setDirection(DcMotorSimple.Direction.REVERSE);
        xRailLift.setDirection(DcMotorSimple.Direction.FORWARD);
        xRailExtend.setDirection(DcMotorSimple.Direction.REVERSE);
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
    public void raiseXRailToVisionPosition(){}
    public void lowerXRailToFloor(){}
}

