package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class Robot {
    public HardwareMap hardwareMap = null;
    public DriveTrain driveTrain = null;
    public Intake intake = null;
    public LatchLift latchLift = null;
    public Output output = null;
    public Sensors sensors = null;
    public Vision vision = null;


    public Robot(HardwareMap inHardwareMap){
        hardwareMap = inHardwareMap;
        driveTrain = new DriveTrain(this);
        intake = new Intake(this);
        latchLift = new LatchLift(this);
        output = new Output(this);
        sensors = new Sensors(this);
        vision = new Vision(this);
    }
}
