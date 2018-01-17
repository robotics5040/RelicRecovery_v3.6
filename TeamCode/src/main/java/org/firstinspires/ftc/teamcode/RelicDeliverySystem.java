package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by jedch on 1/17/2018.
 */

public class RelicDeliverySystem {
    int relicMotorPosition = robot.relicMotor.getCurrentPosition();
    int newRelicMotorPosition = relicMotorPosition;

    double rwCurrent = robot.relicWrist.getPosition(), rwGoal = rwCurrent;

    double power = 0.5;
    final int RELIC_OUT = 3000; // Minimum Value to Prevent Over Extension
    final int RELIC_IN  = 0;
    double SERVO_INCREMENT = 0.04, decay = 0.008;



        if(right_stick_y_2 < -0.1 && robot.relicMotor.getCurrentPosition() < RELIC_OUT){
        robot.relicStopper.setPosition(0.3);
        newRelicMotorPosition = RELIC_OUT;
        power = 1.0;
    }else if(right_stick_y_2 > 0.1){
        newRelicMotorPosition = RELIC_IN;
        power = 1.0;
    }else{
        newRelicMotorPosition = relicMotorPosition;
    }

        if(robot.relicWrist.getPosition() < 0.30){
        SERVO_INCREMENT = 0.01;
    }else{
        SERVO_INCREMENT = 0.04;
    }

        if(left_stick_y_2 < -0.1){
        rwGoal = rwCurrent + SERVO_INCREMENT;
        //SERVO_INCREMENT -= decay;
    }else if(left_stick_y_2 > 0.1){
        rwGoal = rwCurrent - SERVO_INCREMENT;
        //SERVO_INCREMENT += decay;
    }

        if(rwGoal > 1.0){
        rwGoal = 1.0;
    }else if(rwGoal < 0.0){
        rwGoal = 0.0;
    }

        if(a_button_2){
        robot.relicClaw.setPosition(0.0);
    }else{
        robot.relicClaw.setPosition(0.5);
    }

        robot.relicWrist.setPosition(rwGoal);

        telemetry.addData("Servo Increment: ", SERVO_INCREMENT);
        telemetry.addData("Motor Slide New Position: ", newRelicMotorPosition);
        telemetry.addData("Motor Slide Curent Positon: ", relicMotorPosition);
        telemetry.addData("Relic Claw Position: ", robot.relicClaw.getPosition());
        telemetry.addData("Relic Wrist Position: ", robot.relicWrist.getPosition());

    newRelicMotorPosition = Range.clip(newRelicMotorPosition, RELIC_IN, RELIC_OUT);
        robot.relicMotor.setTargetPosition(newRelicMotorPosition);
    relicMotorPosition = robot.relicMotor.getCurrentPosition();
    //power = Math.abs(right_stick_y_2);

        if(power < 0.4){
        power = 0.4;
    }

        robot.relicMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.relicMotor.setPower(power);
}
