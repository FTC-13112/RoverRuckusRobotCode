package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.robot.Robot;

/**
 * Created by Dravid-C on 10/7/2018.
 */
@TeleOp(name="Basic Teleop", group="Testing")
public class BasicTeleop extends OpMode {
    private Robot robot = null;
    public CRServo intake_servo = null;

    public double linearPosition = 0;

    @Override
    public void init() {
        robot = new Robot(hardwareMap, telemetry);
        robot.output.holdMarker();
        intake_servo  = hardwareMap.get(CRServo.class, "intake_servo" );
        intake_servo.setPower(0.0);

    }

    @Override
    public void loop() {

        //For Intake Motor

        if (gamepad1.left_bumper) {
            robot.intake.infeed();

        } else if (gamepad1.right_bumper) {
            robot.intake.outfeed();

        } else if (gamepad1.b){
            robot.intake.stopIntake();
        }

        if(gamepad2.left_bumper){
            intake_servo.setPower(-1.0);
        } else if (gamepad2.right_bumper){
            intake_servo.setPower(1.0);
        } else if (gamepad2.y){
            intake_servo.setPower(0.0);
        }

        //For Drivetrain motors
        robot.driveTrain.leftDrive.setPower(gamepad1.left_stick_y);
        robot.driveTrain.rightDrive.setPower(gamepad1.right_stick_y);

        //For Lift motors

        if(gamepad2.dpad_up) {
            robot.latchLift.liftUp();
        } else if (gamepad2.dpad_down){
            robot.latchLift.liftDown();
        } else if (gamepad2.x){
            robot.latchLift.liftStop();
        }

        if(gamepad2.a){
            robot.latchLift.liftServo.setPosition(1.0);
        } else if (gamepad2.b){
            robot.latchLift.liftServo.setPosition(0.0);
        }


        //For team marker
        if(gamepad1.dpad_up){
            robot.output.holdMarker();
        } else if (gamepad1.dpad_down){
            robot.output.dropMarker();
        }

        //Telemetry
        robot.telemetry.addData("RightY", "LeftStick:&7f", gamepad1.right_stick_y);

    }
}
