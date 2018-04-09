package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.text.DecimalFormat;

/**
 * Created by Matthew on 11/12/2017.
 */
@Disabled
public class AutoPull extends LinearOpMode {
    ElapsedTime runtime = new ElapsedTime();
    private final double MIN_MOTOR_OUTPUT_VALUE = -1.0;
    private final double MAX_MOTOR_OUTPUT_VALUE = 1.0;

    private int columnNum = 0;
    private double flexCurrent;
    private double flexPrevious = 0;
    private final double TOLERANCE = 0.30;

    @Override public void runOpMode() throws InterruptedException {}

    //needed for driving
    public double limit(double a) {
        return Math.min(Math.max(a, MIN_MOTOR_OUTPUT_VALUE), MAX_MOTOR_OUTPUT_VALUE);
    }

    //normal drive for robot
    public void omniDrive (HardwareOmniRobot robot,double sideways, double forward, double rotation, boolean half)
    {

        double rotat;
        if(half == true) {
            rotat = 2;
        }
        else if(rotation == 0) {
            rotat = 1;
        }
        else {
            rotat = 1.4;
        }

        try {
            robot.leftMotor1.setPower(limit(((forward - sideways)/rotat) + (-.3 * rotation)));
            robot.leftMotor2.setPower(limit(((forward + sideways)/rotat) + (-.3 * rotation)));
            robot.rightMotor1.setPower(limit(((-forward - sideways)/rotat) + (-.3 * rotation)));
            robot.rightMotor2.setPower(limit(((-forward + sideways)/rotat) + (-.3 * rotation)));
        } catch (Exception e) {
        }
    }

    //drives robot for certain time amount. Can also be used for waiting time
    public void DriveFor(HardwareOmniRobot robot, double time, double forward, double side, double rotate, boolean half) {
        omniDrive(robot,-side, forward, -rotate, half); //starts moving in wanted direction
        runtime.reset(); //resets time

        while (opModeIsActive() && runtime.seconds() < time) {    //runs for amount of time wanted
        }
        omniDrive(robot,0.0, 0.0, 0.0, half); //stops  moving after
    }

    //Jewel knocking off code - gets called from the jewel code
    public void TurnLeft(HardwareOmniRobot robot){
        telemetry.addLine("Left");
        telemetry.update();
        DriveFor(robot,0.2, 0.0, 0.0, -1, true);
        robot.jknock.setPosition(0.7);
        DriveFor(robot,0.3, 0.0, 0.0, 1, true);
    }
    public void TurnRight(HardwareOmniRobot robot){
        telemetry.addLine("Right");
        telemetry.update();
        DriveFor(robot,0.2, 0.0, 0.0, 1, true);
        robot.jknock.setPosition(0.7);
        DriveFor(robot,0.3, 0.0, 0.0, -1, true);
    }

    //jewel code
    public void JewelKnock(HardwareOmniRobot robot,String side){

        robot.jkcolor.enableLed(true);
        robot.jkcolor2.enableLed(true);
        robot.jknock.setPosition(0.11);
        //DriveFor(robot,0.5,0.0,0.0,0.0);
        boolean decided = false;
        runtime.reset();
        int color1b = robot.jkcolor.blue();
        int color1r = robot.jkcolor.red();
        int color2b = robot.jkcolor2.blue();
        int color2r = robot.jkcolor2.red();
        telemetry.addData("Color 1b", color1b);
        telemetry.addData("Color 1r", color1r);
        telemetry.addData("Color 2b", color2b);
        telemetry.addData("Color 2r", color2r);
        telemetry.update();

        while (opModeIsActive() && decided == false && runtime.seconds() < 1) {
            if (color1r < 2 && color1b< 2 && color2r < 2 && color2b < 2) {
                decided = true;
                robot.jknock.setPosition(robot.JKUP);
            }
            else if(side == "blue") {
                if((color1b>color1r+1 && color1r<color1b-1) || (color2b<color2r-1 && color2r>color2b+1)) {
                    TurnLeft(robot);
                    decided = true;
                }
                else if((color1b<color1r-1 && color1r>color1b+1) || (color2b>color2r+1 && color2r<color2b-1)) {
                    TurnRight(robot);
                    decided = true;
                }
            }
            else if(side == "red") {
                if((color1b>color1r+1 && color1r<color1b-1) || (color2b<color2r-1 && color2r>color2b+1)) {
                    TurnRight(robot);
                    decided = true;
                }
                else if((color1b<color1r-1 && color1r>color1b+1) || (color2b>color2r+1 && color2r<color2b-1)) {
                    TurnLeft(robot);
                    decided = true;
                }
            }
        }
        robot.jkcolor.enableLed(false);
        robot.jkcolor2.enableLed(false);
        if(robot.jknock.getPosition() != robot.JKUP) {robot.jknock.setPosition(robot.JKUP);}
    }

    public void rotateTo(HardwareOmniRobot robot,float degrees,float potent) {
        float heading = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;//getGyro(robot) - gyro;
        double speed = 0.4;
        boolean go = false;
        degrees += potent;

        runtime.reset();
        while (heading != degrees && opModeIsActive() && runtime.seconds() < 1.5) {
            telemetry.addData("HEADING", heading);
            telemetry.addData("Degrees", degrees);
            telemetry.addData("speed", speed);
            telemetry.update();
            heading = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;//robot.gyro.getHeading() - gyro;
            if (degrees-0.5< heading) {
                omniDrive(robot, 0.0, 0.0, -speed,true);
                go = true;
            } else if (degrees+0.5 > heading) {
                omniDrive(robot, 0.0, 0.0, speed,true);
                if (speed > 0.35 && go == true) {
                    speed -= 0.01;
                }
            }
        }
        omniDrive(robot, 0.0, 0.0, 0.0,true);
    }

    //vuforia
    public int Vuforia(int cameraMonitorViewId, String side, VuforiaLocalizer vuforia) {

        int choosen = 0;

        try {

            VuforiaTrackables relicTrackables = vuforia.loadTrackablesFromAsset("RelicVuMark");
            VuforiaTrackable relicTemplate = relicTrackables.get(0);
            relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

            relicTrackables.activate();
            runtime.reset();
            while (opModeIsActive() && choosen == 0 && runtime.seconds() < 3) {
                RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
                if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
                    if(side == "red") {
                        switch (vuMark) {
                            case LEFT:
                                choosen = 3;
                                break;
                            case CENTER:
                                choosen = 2;
                                break;
                            case RIGHT:
                                choosen = 1;
                                break;
                        }
                    }
                    else {
                        switch (vuMark) {
                            case LEFT:
                                choosen = 1;
                                break;
                            case CENTER:
                                choosen = 2;
                                break;
                            case RIGHT:
                                choosen = 3;
                                break;
                        }
                    }
                }
            }
        }catch (Exception e){
            choosen = 0;
        }

        return choosen;
    }
}