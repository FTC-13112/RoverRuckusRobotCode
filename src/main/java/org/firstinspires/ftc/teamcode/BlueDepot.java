package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.teamcode.robot.Constants;
import org.firstinspires.ftc.teamcode.robot.MineralLocation;
import org.firstinspires.ftc.teamcode.robot.Robot;

/**
 * Created by Dravid-C on 11/18/2018.
 */
@Autonomous(name = "blue_depot")
public class BlueDepot extends LinearOpMode {

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
        //} else if (telemetryState == 1){

        //} else if (telemetryState == 2){

        //} else if (telemetryState == 3) {}
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
            robot.driveTrain.driveForwardEncoder(Constants.DISTANCE_TO_MIDDLE_MINERAL, 0.4, 5.0);
        } else {
            robot.driveTrain.driveForwardEncoder(Constants.DISTANCE_TO_OUTER_MINERALS, 0.5, 5.0);
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

        if(goldMineralLocation == MineralLocation.MIDDLE){
            robot.driveTrain.driveForwardEncoder(Constants.DISTANCE_FROM_MIDDLE_TO_DEPOT, 0.25, 5.0);
        } else {
            currentHeading = robot.sensors.getHeading();
            targetAngle = robot.sensors.addToHeading(2 * (currentHeading-startingHeading ));
            robot.driveTrain.gyroTurnToHeading(targetAngle, 5.0);
            robot.driveTrain.driveForwardEncoder(Constants.DISTANCE_FROM_OUTER_TO_DEPOT, 0.25, 999.0);

        }
        robot.driveTrain.gyroTurnToHeading(robot.sensors.addToHeading(startingHeading, 90),5.0);

        //Drop marker
        robot.output.dropMarker();
        robot.output.holdMarker();
    }

        private void parkInCrater(){
            //Turn to face crater
            double targetHeading = robot.sensors.addToHeading(startingHeading, Constants.ANGLE_BLUE_DEPOT_TO_BLUE_CRATER);
            robot.driveTrain.gyroTurnToHeading(targetHeading, 5.0);

            //Drive to crater
            //robot.driveTrain.driveForwardEncoder(Constants.DISTANCE_FROM_DEPOT_TO_CRATER, 0.5, 10.0);
            robot.driveTrain.driveForwardUntilTilted(0.5, 10.0);
        }


}
