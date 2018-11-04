package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import static android.os.SystemClock.sleep;


public class Output {

    private Robot robot = null;

    public Servo markerServo = null;

    public Output(Robot inRobot) {
        robot = inRobot;

        markerServo = robot.hardwareMap.get(Servo.class, "marker_servo");

    }
    public void dropMarker (){
        markerServo.setPosition(1.0);
        sleep(1000);
    }

    public void holdMarker(){
        markerServo.setPosition(0.0);
        sleep(1000);
    }

}

