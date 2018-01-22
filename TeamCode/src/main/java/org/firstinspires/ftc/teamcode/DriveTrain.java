package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.RobotLog;

/**
 * Created by jedch on 1/20/2018.
 */

public class DriveTrain {

    private HardwareOmniRobot robot;

    private double forward, side, rotation;

    private enum Side {FRONT, LEFT, RIGHT, BACK}
    private Side currentSide = Side.FRONT;
    public static final String MESSAGETAG = "5040MSG";

    public DriveTrain(HardwareOmniRobot robot){
        this.robot = robot;
    }

    public void makeAdjustments (boolean dup, boolean ddown, boolean dleft, boolean dright){
        //convert the booleans to integers.
        int up    = dup ? 1 : 0;
        int back  = ddown ? 1 : 0;
        int left  = dleft ? 1 : 0;
        int right = dright ? 1 : 0;

        side    = (-0.5 * left) + (0.5 * right);
        forward = (-0.5 * up) + (0.5 * back);
    }

    public void changeSide (boolean front, boolean right, boolean left, boolean back) {
        if (front) {
            currentSide = Side.FRONT;
        } else if (right) {
            currentSide = Side.RIGHT;
        } else if (left) {
            currentSide = Side.LEFT;
        } else if (back) {
            currentSide = Side.BACK:
        }
    }

    public void drive (double ljoystick_x, double ljoystick_y, double rjoystick_y, double rjoystick_x) {
        omniDrive(ljoystick_y, ljoystick_y, rjoystick_x);
    }

    public void omniDrive(double sideways, double forward, double rotation){
        try {
            robot.leftMotor1.setPower(robot.limit(((forward - sideways)/2) * 1 + (-.2 * rotation)));
            robot.leftMotor2.setPower(robot.limit(((forward + sideways)/2) * 1 + (-.2 * rotation)));
            robot.rightMotor1.setPower(robot.limit(((-forward - sideways)/2) * 1 + (-.2 * rotation)));
            robot.rightMotor2.setPower(robot.limit(((-forward + sideways)/2) * 1 + (-.2 * rotation)));
        } catch (Exception e) {
            RobotLog.ee(MESSAGETAG, e.getStackTrace().toString());
        }
    }



}
