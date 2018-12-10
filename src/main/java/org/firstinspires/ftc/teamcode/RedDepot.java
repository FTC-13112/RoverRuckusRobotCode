package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.Constants;
import org.firstinspires.ftc.teamcode.robot.MineralLocation;
import org.firstinspires.ftc.teamcode.robot.Robot;

/**
 * Created by Dravid-C on 11/18/2018.
 */
@Autonomous(name = "red_depot")
public class RedDepot extends LinearOpMode {

    private Robot robot = null;
    private MineralLocation goldMineralLocation;
    private double startingHeading;


    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        waitForStart();

        //landRobot();

        //sampleMineral();

        claimDepot();

        parkInCrater();

    }

    private void initialize() {
        robot = new Robot(this, hardwareMap, telemetry);
        robot.output.holdMarker();
        robot.latchLift.lockLift();
        robot.latchLift.latch();
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
/*
        //Get the starting heading so we can use it in claimDepot
        startingHeading = robot.sensors.getHeading();
        //Find location of gold mineral and turn to it
        robot.intake.raiseXRailToVisionPosition();
        goldMineralLocation = robot.driveTrain.turnToGoldMineral(5.0);
        robot.intake.lowerXRailToFloor();



        // Drive to the gold mineral
        if (goldMineralLocation == MineralLocation.MIDDLE) {
            robot.driveTrain.driveForwardEncoder(Constants.DISTANCE_TO_MIDDLE_MINERAL, 0.4, 5.0);
        } else {
            robot.driveTrain.driveForwardEncoder(Constants.DISTANCE_TO_OUTER_MINERALS, 0.5, 5.0);
        }
        /**/
        //Pick up gold mineral and return to pre picked position
        /*robot.intake.infeed();
        sleep(500);
        robot.driveTrain.driveForwardEncoder(
                Constants.DISTANCE_TO_PICKUP_MINERAL,
                Constants.SPEED_GENERAL_MOVE,
                5.0
        );
        robot.intake.stopIntake();*/

    }


    private void claimDepot(){
        //Drive into depot
        /*
        if(goldMineralLocation == MineralLocation.MIDDLE){
            robot.driveTrain.driveForwardEncoder(Constants.DISTANCE_FROM_MIDDLE_TO_DEPOT, 0.25, 5.0);
        } else {
            double currentHeading = robot.sensors.getHeading();
            double targetAngle = robot.sensors.addToHeading(2 * (startingHeading-currentHeading));
            robot.driveTrain.gyroTurnByAngle(targetAngle, 5.0);
            robot.driveTrain.driveForwardEncoder(Constants.DISTANCE_FROM_OUTER_TO_DEPOT, 0.25, 5.0);

        }
        robot.driveTrain.gyroTurnToHeading(robot.sensors.addToHeading(startingHeading, 90),5.0);
        */
        //Drop marker
        robot.driveTrain.driveForwardEncoder(60, 0.5, 10.0);
        robot.driveTrain.gyroTurnByAngle(90, 5.0);
        robot.output.dropMarker();
        robot.output.holdMarker();
    }

    private void parkInCrater(){
        //Turn to face crater
        double targetHeading = robot.sensors.addToHeading(startingHeading, Constants.ANGLE_RED_DEPOT_TO_RED_CRATER);
        robot.driveTrain.gyroTurnToHeading(targetHeading, 5.0);

        //Drive to crater
        robot.driveTrain.driveForwardEncoder(Constants.DISTANCE_FROM_DEPOT_TO_CRATER, 0.5, 10.0);
    }

}

