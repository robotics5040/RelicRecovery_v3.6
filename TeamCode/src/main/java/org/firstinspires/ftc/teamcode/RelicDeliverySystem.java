package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by jedch on 1/17/2018.
 */

public class RelicDeliverySystem {

    private HardwareOmniRobot robot;

    private double increment = 0.04;

    /**
     * Instantiates the <code>RelicDeliverySystem</code>
     * @param robot the robot object that contains the needed hardware for the relic slide
     */
    public RelicDeliverySystem(HardwareOmniRobot robot) {
        //Initialize the robot based on the given one
        this.robot = robot;
    }

    /**
     * <code>moveSlide</code> is used to move the relic slide in and out
     * @param joystick a double value of a joystick that moves the slide forward
     * @param isArmUp a boolean value indicating whether or not the grabber system of the robot
     *                is up
     *                <code>true</code>  - the arm is up
     *                <code>false</code> - the arm is not up
     */
    public void moveSlide(double joystick, boolean isArmUp) {
        //Sets the poser to a default of zero
        double power = 0.0;

        //Deactivates the relic stopper and moves the slide forward based on the joystick
        if(joystick < -0.1 && isArmUp) {
            robot.relicStopper.setPosition(0.3);
            power = 1.0;
        }else if(joystick > 0.1 && isArmUp) {
            robot.relicStopper.setPosition(0.3);
            power = 1.0;
        }

        //Set the power to the slide
        robot.relicMotor.setPower(power);
    }

    /**
     *  <code>moveWrist</code> is used to move the wrist on the relic slide
     *  @param joystick a double value from the controller that is used to move the wrist
     *  @param halfwayButton a boolean from the controller that moves the relic wrist to the
     *                       halfway position
     */
    public void moveWrist(double joystick, boolean halfwayButton){
        // Fetch the actual value of the wrist
        double rwCurrent = robot.relicWrist.getPosition(), rwGoal = rwCurrent;

        //Change the speed of the wrist if it excedes the position 0.3;
        if(rwCurrent < 0.30){
            increment = 0.01;
        }else{
            increment = 0.04;
        }

        //Move the servo based on the speed
        if(joystick < -0.1){
            rwGoal = rwCurrent + increment;
        }else if(joystick > 0.1){
            rwGoal = rwCurrent - increment;
        }

        if(halfwayButton) {
            rwGoal = 0.50;
        }

        //Make sure that the new position isn't out of bounds
        Range.clip(rwGoal, 0.0, 1.0);

        //Apply the new position
        robot.relicWrist.setPosition(rwGoal);
    }

    /**
     * Opens the claw to various grips based on the inputted buttons
     * @param buttonOpen
     * @param buttonPartway
     */
    public void openClaw(boolean buttonOpen, boolean buttonPartway) {
        if (buttonOpen) {
            robot.relicClaw.setPosition(0.0);
        } else if (buttonPartway) {
            robot.relicClaw.setPosition(0.46);
        } else {
            robot.relicClaw.setPosition(0.5);
        }
    }

    /* Just ignore this... old code that this class is based on
            int relicMotorPosition = robot.relicMotor.getCurrentPosition();
        int newRelicMotorPosition = relicMotorPosition;

        double rwCurrent = robot.relicWrist.getPosition(), rwGoal = rwCurrent;

        double power = 0.5;
        final int RELIC_OUT = 3500; // Minimum Value to Prevent Over Extension
        final int RELIC_IN  = 0;
        double SERVO_INCREMENT = 0.04, decay = 0.008;

        if(right_stick_y_2 < -0.1 && robot.relicMotor.getCurrentPosition() < RELIC_OUT && run2 == true){
            robot.relicStopper.setPosition(0.0);
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

        if(a_button2 == true){
            robot.relicClaw.setPosition(0.0);
        }
        else if(b_button2 == true) {
            robot.relicClaw.setPosition(0.46);
        }
        else{
            robot.relicClaw.setPosition(0.5);
        }

        robot.relicWrist.setPosition(rwGoal);

                telemetry.addData("Servo Increment: ", SERVO_INCREMENT);
        telemetry.addData("Motor Slide New Position: ", newRelicMotorPosition);
        telemetry.addData("Motor Slide Current Position: ", relicMotorPosition);
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
     */

}
