package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Robot {
    public LinearOpMode currentOpMode = null;
    public HardwareMap hardwareMap = null;
    public Telemetry telemetry = null;
    public DriveTrain driveTrain = null;
    public Intake intake = null;
    public LatchLift latchLift = null;
    public Output output = null;
    public Sensors sensors = null;
    public Vision vision = null;


    public Robot(HardwareMap inHardwareMap, Telemetry telemetry){
        hardwareMap = inHardwareMap;
        this.telemetry = telemetry;
        driveTrain = new DriveTrain(this);
        intake = new Intake(this);
        latchLift = new LatchLift(this);
        output = new Output(this);
        sensors = new Sensors(this);
        vision = new Vision(this);
    }

    public Robot(LinearOpMode currentOpMode, HardwareMap inHardwareMap, Telemetry telemetry){
        this(inHardwareMap, telemetry);
        this.currentOpMode = currentOpMode;
    }

    public boolean isRunningAutonomous() {
        return currentOpMode != null;
    }
}
