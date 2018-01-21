package org.firstinspires.ftc.teamcode;

/**
 * Created by jedch on 1/20/2018.
 */

public class DriveTrain {

    private HardwareOmniRobot robot;

    public DriveTrain(HardwareOmniRobot robot){
        this.robot = robot;
    }

    public void makeAdjustments(boolean dup, boolean ddown, boolean dleft, boolean dright){
        //convert the booleans to integers.
        int forward  = dup ? 1 : 0;
        int backward = ddown ? 1 : 0;
        int left     = dleft ? 1 : 0;
        int right    = dright ? 1 : 0;


    }

}
