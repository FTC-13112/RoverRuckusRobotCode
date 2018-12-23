package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.teamcode.robot.Constants;
import org.firstinspires.ftc.teamcode.robot.MineralLocation;
import org.firstinspires.ftc.teamcode.robot.Robot;

/**
 * Created by Dravid-C on 12/16/2018.

 */
@Autonomous(name = "blue_crater")
public class BlueCrater extends LinearOpMode {
    private Robot robot = null;
    private MineralLocation goldMineralLocation;
    private double startingHeading;
    private double currentHeading;
    private double targetAngle;
    private int telemetryState = 0;
    private double telemetry_pixyValue;
    private boolean telemetry_pixyVisible;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        composeTelemetry();

        waitForStart();

        //landRobot();

        sampleMineral();

        claimDepot();

        parkInCrater();

    }

    private void initialize() {
        robot = new Robot(this, hardwareMap, telemetry);
        robot.output.holdMarker();
        robot.latchLift.lockLift();
        robot.latchLift.latch();
    }

    private void composeTelemetry(){
        telemetry.clearAll();

        telemetry.addAction(new Runnable() {
            @Override public void run() {
                // Acquiring the angles is relatively expensive; we don't want
                // to do that in each of the three items that need that info, as that's
                // three times the necessary expense.
                telemetry_pixyValue = robot.sensors.pixy_x_location.getVoltage();
                telemetry_pixyVisible = robot.vision.isMarkerVisible();

            }
        });

        telemetry.addLine()
                .addData("telemetry state", new Func<String>() {
                    @Override
                    public String value() {
                        return Integer.toString(telemetryState);
                    }
                });

        //if(telemetryState == 0) {
        telemetry.addLine()
                .addData("pixy X voltage", new Func<String>() {
                    @Override
                    public String value() {
                        return Double.toString(telemetry_pixyValue);
                    }
                });

        telemetry.addLine()
                .addData("pixy marker visible", new Func<String>() {
                    @Override
                    public String value() {
                        return Boolean.toString(telemetry_pixyVisible);
                    }
                });
        telemetry.addLine()
                .addData("starting - current", new Func<String>() {
                    @Override
                    public String value() {
                        return Double.toString(startingHeading - currentHeading);
                    }
                });
        telemetry.addLine()
                .addData("target", new Func<String>() {
                    @Override
                    public String value() {
                        return Double.toString(targetAngle);
                    }
                });
    }

    private void landRobot(){
        // Begin lowering sequence, robot must raise before lift can be unlocked


        robot.latchLift.raiseRobotToReleasePin(3.0);
        robot.latchLift.unlockLift();

        // Lower the robot and unlatch from the lander
        robot.latchLift.lowerRobotToGround(7.0);
        robot.latchLift.unlatch();

        // Reset the lift to the down position and close the latch
        robot.latchLift.retractLift(4.0);
        robot.latchLift.unlatch();
        robot.latchLift.raiseRobotToReleasePin(1.5);
        robot.latchLift.unlatch();
    }

    private void sampleMineral(){

        //Get the starting heading so we can use it in claimDepot
        startingHeading = robot.sensors.getHeading();
        //Find location of gold mineral and turn to it
        robot.intake.raiseXRailToVisionPosition();
        goldMineralLocation = robot.driveTrain.turnToGoldMineral(5.0);
        robot.intake.lowerXRailToFloor();

        robot.intake.infeed();


        // Drive to the gold mineral
        if (goldMineralLocation == MineralLocation.MIDDLE) {
            robot.driveTrain.driveForwardEncoder(Constants.CRATER_DISTANCE_TO_MIDDLE_MINERAL, 0.4, 5.0);
        } else {
            robot.driveTrain.driveForwardEncoder(Constants.CRATER_DISTANCE_TO_OUTER_MINERALS, 0.5, 5.0);
        }

        if(goldMineralLocation == MineralLocation.MIDDLE){
            robot.driveTrain.driveBackwardEncoder(Constants.DISTANCE_CRATER_REVERSE_MIDDLE, 0.4, 5.0);
        } else {
            robot.driveTrain.driveBackwardEncoder(Constants.DISTANCE_CRATER_REVERSE_OUTER ,0.4  ,5.0  );
        }

        //Pick up gold mineral and return to pre picked position
        //robot.intake.outfeed();
        //sleep(500);
        /*robot.driveTrain.driveForwardEncoder(
                Constants.DISTANCE_TO_PICKUP_MINERAL,
                Constants.SPEED_GENERAL_MOVE,
                5.0
        );*/
        robot.intake.stopIntake();

    }
    private void claimDepot(){
        //Drive into depot
        robot.driveTrain.gyroTurnToHeading(robot.sensors.addToHeading(startingHeading, 90), 5.0);

        robot.driveTrain.driveTillDistanceLessThanValue(25.0, -0.5, 5.0);

        robot.driveTrain.gyroTurnToHeading(robot.sensors.addToHeading(startingHeading, 55), 5.0);

        robot.driveTrain.driveTillDistanceLessThanValue(30, -0.7, 5.0);

        robot.driveTrain.gyroTurnToHeading(startingHeading,5.0);

        //Drop marker
        robot.output.dropMarker();
        robot.output.holdMarker();
    }

    private void parkInCrater(){
        //Turn to face crater
        double targetHeading = robot.sensors.addToHeading(startingHeading, Constants.ANGLE_BLUE_CRATER_TO_BLUE_CRATER);
        robot.driveTrain.gyroTurnToHeading(targetHeading, 5.0);

        //Drive to crater
        //robot.driveTrain.driveForwardEncoder(Constants.DISTANCE_FROM_DEPOT_TO_CRATER, 0.5, 10.0);
        robot.driveTrain.driveForwardUntilTilted(0.5, 10.0);
    }



}
