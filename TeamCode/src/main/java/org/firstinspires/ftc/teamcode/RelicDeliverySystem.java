package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by jedch on 1/17/2018.
 */

public class RelicDeliverySystem {

    private HardwareOmniRobot robot;

    private final int RELIC_OUT = 3000; // Minimum Value to Prevent Over Extension
    private final int RELIC_IN = 0;
    private double increment = 0.04;

    public RelicDeliverySystem(HardwareOmniRobot robot) {
        //Initialize the robot based on the given one
        this.robot = robot;
    }

    public void moveSlide(double joystick) {
        double power = 0.5;
        int relicMotorPosition = robot.relicMotor.getCurrentPosition();
        int newRelicMotorPosition = relicMotorPosition;

        if(joystick< -0.1 && robot.relicMotor.getCurrentPosition() <RELIC_OUT) {
            robot.relicStopper.setPosition(0.3);
            newRelicMotorPosition = RELIC_OUT;
            power = 1.0;
        }else if(joystick >0.1) {
            newRelicMotorPosition = RELIC_IN;
            power = 1.0;
        }else {
            newRelicMotorPosition = relicMotorPosition;
        }

        newRelicMotorPosition = Range.clip(newRelicMotorPosition, RELIC_IN, RELIC_OUT);
        robot.relicMotor.setTargetPosition(newRelicMotorPosition);
        relicMotorPosition = robot.relicMotor.getCurrentPosition();

        if(power < 0.4){
            power = 0.4;
        }

        robot.relicMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.relicMotor.setPower(power);
    }

    public void moveWrist(double joystick){
        double rwCurrent = robot.relicWrist.getPosition(), rwGoal = rwCurrent;

        if(robot.relicWrist.getPosition() < 0.30){
            increment = 0.01;
        }else{
            increment = 0.04;
        }

        if(joystick < -0.1){
            rwGoal = rwCurrent + increment;
        }else if(joystick > 0.1){
            rwGoal = rwCurrent - increment;
        }

        if(rwGoal > 1.0){
            rwGoal = 1.0;
        }else if(rwGoal < 0.0){
            rwGoal = 0.0;
        }

        robot.relicWrist.setPosition(rwGoal);
    }

    public void openClaw(boolean button) {
        if (button) {
            robot.relicClaw.setPosition(0.0);
        } else {
            robot.relicClaw.setPosition(0.5);
        }
    }


}
