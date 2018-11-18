package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Dravid-C on 11/4/2018.
 * lol lets get it
 */
public class ActiveTimeout {
    private ElapsedTime runtime = new ElapsedTime();
    private double secondsActive;
    private boolean lastActive = false;

    public ActiveTimeout(double secondsActive) {
        this.secondsActive = secondsActive;
    }

    public boolean checkValid(boolean currentValue) {
        if (currentValue) {
            if (lastActive && runtime.seconds() > secondsActive)
                return true;
            else if (!lastActive) {
                lastActive = true;
                runtime.reset();
            }
        } else {
            lastActive = false;
        }
        return false;
    }
}
