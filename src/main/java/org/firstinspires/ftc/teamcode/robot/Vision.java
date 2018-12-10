package org.firstinspires.ftc.teamcode.robot;

public class Vision {
    private Robot robot = null;


    public Vision(Robot inRobot) {
        robot = inRobot;
    }

    /**
     * Finds gold mineral and gives angle from 0 (left) to 3.3 (right)
     * @return value of the angle
     */
    public double getPixyXLocation(){
        return robot.sensors.pixy_x_location.getVoltage();
    }

    public MineralLocation determineMineralLocation() {
        double currentLocation = getPixyXLocation();
        if (robot.valueInRange(Constants.PIXY_MIDDLE, Constants.PIXY_FOV_WINDOW, currentLocation)) {
            return MineralLocation.MIDDLE;
        } else if (currentLocation < Constants.PIXY_MIDDLE) {
            return MineralLocation.LEFT;
        } else {
            return MineralLocation.RIGHT;
        }
    }

    public boolean isMarkerVisible() {
        return robot.sensors.pixy_visible.getState();

    }
}
