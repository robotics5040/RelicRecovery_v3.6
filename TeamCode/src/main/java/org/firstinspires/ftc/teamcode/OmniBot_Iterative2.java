/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * This file provides basic Teleop driving for an Omni-wheeled robot (omnibot for short).
 * The code is structured as an Iterative OpMode
 *
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 *
 * This particular OpMode executes a basic Tank Drive Teleop for a PushBot
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Pushbot: Omnibot Pushbot Grabber up", group="Pushbot")
//@Disabled
public class OmniBot_Iterative2 extends OpMode{
    private double position = 0.0;
    public int  pressed = 0,up=10;
    private int gamepadMode1 = 0; //Controls wich control scheme is being used by Gamepad1, is toggled by Start-x
    private int gamepadMode2 = 0; //Controls wich control scheme is being used by Gamepad2, is toggled by Start-x
    double wrist_num = 0;
    boolean run2=false, goup = false,dumperReset = false, grabberDown=true, run =false, grabberReset=false;
    
    
    /**
     * Directional Variables; <code>front-facing, back-facing, left-facing,</code> and <code>right-facing</code> refer towich direction the robot
     * move towards. 
     * *Note: the front of the robot refers to the side with the claw/grabber mechanism 
     */
    boolean frontFacing = true, rightFacing = false, leftFacing = false, backFacing = false;
    
    
    boolean closed = true;

    private double front, side, rotate;

    ElapsedTime runtime = new ElapsedTime();
    /* Declare OpMode members. */
    private HardwareOmniRobot robot; // use the class created to define a Pushbot's hardware

    public OmniBot_Iterative2() {
        robot = new HardwareOmniRobot();
    }

    // could also use HardwarePushbotMatrix class.
   /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap, false);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        robot.grabber.setPower(0.6);
        robot.dumper.setPower(0.5);
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        //note: The joystick goes negative when pushed forwards, so negate it)
        
        /*
         *   Gamepad 1
         */
        //Joystick Inputs
        double left_stick_y1  = gamepad1.left_stick_y;
        double right_stick_y1 = gamepad1.right_stick_y;
        //right_trigger1  = gamepad1.right_trigger;//unused
        double left_stick_x1   = gamepad1.left_stick_x;
        double right_stick_x1  = gamepad1.right_stick_x;

        //Bumpers and Triggers
        boolean left_bumper1  = gamepad1.left_bumper  && gamepadMode1 == 0;
        boolean right_bumper1 = gamepad1.right_bumper && gamepadMode1 == 0;
        //left_trigger1  = gamepad1.left_trigger;//unused
        //right_trigger1 = gamepad1.right_trigger;//unused

        //Button inputs
        boolean b_button1 = gamepad1.b && gamepadMode1 == 0;
        boolean a_button1 = gamepad1.a && gamepadMode1 == 0;
        boolean y_button1 = gamepad1.y && gamepadMode1 == 0;
        boolean x_button1 = gamepad1.x && gamepadMode1 == 0;

        //Directional Pad Inputs
        boolean dup1    = gamepad1.dpad_up    && gamepadMode1 == 0;
        boolean ddown1  = gamepad1.dpad_down  && gamepadMode1 == 0;
        boolean dleft1  = gamepad1.dpad_left  && gamepadMode1 == 0;
        boolean dright1 = gamepad1.dpad_right && gamepadMode1 == 0;

        //Auxillary Inputs
        boolean left_stick_press1  = gamepad1.right_stick_button && gamepadMode1 == 0;
        boolean right_stick_press1 = gamepad1.left_stick_button  && gamepadMode1 == 0;
        boolean home = gamepad1.guide && gamepadMode1 == 0;
        
        /*
         *   Gamepad 2
         */
        //Joystick Inputs
        double left_stick_y_2  = gamepad2.left_stick_y;
        double right_stick_y_2 = gamepad2.right_stick_y;
        double left_stick_x2   = gamepad2.left_stick_x;
        double right_stick_x2  = gamepad2.right_stick_x;
        
        //Bumpers and Triggers
        boolean left_bumper2  = gamepad2.left_bumper  && gamepadMode2 == 0;
        boolean right_bumper2 = gamepad2.right_bumper && gamepadMode2 == 0;
        double left_trigger2  = gamepad2.left_trigger;
        double right_trigger2 = gamepad2.right_trigger;
        
        //Button inputs
        boolean b_button2 = gamepad2.b && gamepadMode2 == 0;
        boolean a_button2 = gamepad2.a && gamepadMode2 == 0;
        boolean y_button2 = gamepad2.y && gamepadMode2 == 0;
        boolean x_button2 = gamepad2.x && gamepadMode2 == 0;
        
        //Directional Pad Inputs
        boolean dup2    = gamepad2.dpad_up    && gamepadMode2 == 0;
        boolean ddown2  = gamepad2.dpad_down  && gamepadMode2 == 0;
        boolean dleft2  = gamepad2.dpad_left  && gamepadMode2 == 0;
        boolean dright2 = gamepad2.dpad_right && gamepadMode2 == 0;
        
        //Auxillary Inputs
        boolean left_stick_press2  = gamepad2.right_stick_button && gamepadMode2 == 0;
        boolean right_stick_press2 = gamepad2.left_stick_button  && gamepadMode2 == 0;
        boolean home2 = gamepad2.guide && gamepadMode2 == 0;
        
        /*
         *   Gamepad 3
         */
        //Joystick Inputs
        double left_stick_y3  = gamepad2.left_stick_y;
        double right_stick_y3 = gamepad2.right_stick_y;
        double right_trigger3  = gamepad2.right_trigger;

        //Button inputs
        boolean b_button3 = (gamepad2.b && gamepadMode2 == 1) || (gamepad1.b && gamepadMode1 == 1);
        boolean a_button3 = (gamepad2.a && gamepadMode2 == 1) || (gamepad1.a && gamepadMode1 == 1);
        boolean y_button3 = (gamepad2.y && gamepadMode2 == 1) || (gamepad1.y && gamepadMode1 == 1);
        boolean x_button3 = (gamepad2.x && gamepadMode2 == 1) || (gamepad1.x && gamepadMode1 == 1);

        //Directional Pad Inputs
        boolean dup2    = (gamepad2.dpad_up    && gamepadMode2 == 1) || (gamepad1.dpad_up    && gamepadMode1 == 1);
        boolean ddown2  = (gamepad2.dpad_down  && gamepadMode2 == 1) || (gamepad1.dpad_down  && gamepadMode1 == 1);
        boolean dleft2  = (gamepad2.dpad_left  && gamepadMode2 == 1) || (gamepad1.dpad_left  && gamepadMode1 == 1);
        boolean dright2 = (gamepad2.dpad_right && gamepadMode2 == 1) || (gamepad1.dpad_right && gamepadMode1 == 1);

        //Auxillary Inputs
        boolean stick_press3  = (gamepad2.right_stick_button && gamepadMode2 == 1) || (gamepad1.right_stick_button && gamepadMode1 == 1);
        boolean stick_press3 = (gamepad2.left_stick_button  && gamepadMode2 == 1)  || (gamepad1.left_stick_button  && gamepadMode1 == 1);
        boolean home3 = (gamepad2.guide && gamepadMode2 == 1);

        //slight adjustments for driver
        if(ddown1 == true) {
            left_stick_y1 = 0.4;
        }
        if(dup1 == true) {
            left_stick_y1 = -0.4;
        }
        if(dleft1 == true) {
            left_stick_x1 = -0.4;
        }
        if(dright1 == true) {
            left_stick_x1 = 0.4;
        }

        //changes front of robot for driver using a,b,x,y
        if(y_button1 == true || frontFacing == true) {
            front = left_stick_y1 * -1;
            side = left_stick_x1 * -1;
            rotate = right_stick_x1*-1;

            frontFacing = true;
            rightFacing = false;
            leftFacing = false;
            yPressed = false;
        }
        if(x_button1 == true || rightFacing == true) {
            front = left_stick_x1;
            side = left_stick_y1*-1;
            rotate = right_stick_y1*-1;

            frontFacing = false;
            rightFacing = true;
            leftFacing = false;
            yPressed = false;
        }
        if(b_button1 == true || leftFacing == true) {
            front = left_stick_x1*-1;
            side = left_stick_y1;
            rotate = right_stick_y1;

            frontFacing = false;
            rightFacing = false;
            leftFacing = true;
            yPressed = false;
        }
        if(a_button1 == true || yPressed == true) {
            front = left_stick_y1;
            side = left_stick_x1;
            rotate = right_stick_x1;

            frontFacing = false;
            rightFacing = false;
            leftFacing = false;
            yPressed = true;
        }

        if(left_bumper1 == true) {
            front /= 2;
            side /= 2;
        }

        robot.onmiDrive(side, front, rotate);

        /*
            The home button moves the grabber down to the bottom position, while this process is happening         
         */
        if(home2 == true || grabberDown == false) {
            if (run == false) {
                robot.grabber.setPower(0.6);
                robot.grabber.setTargetPosition(-1 * robot.GRABBER_AUTOPOS);
                run = true;
                grabberDown = false;
                run2 = false;
                runtime.reset();

            } else if (robot.grabber.getCurrentPosition() > ((-1 * robot.GRABBER_AUTOPOS) + 10) && grabberDown == false && runtime.seconds() < 2) {
                telemetry.addLine("Waiting to get to bottom");
                telemetry.update();
            } else if (robot.grabber.getCurrentPosition() <= ((-1 * robot.GRABBER_AUTOPOS) + 10) && grabberDown == false || runtime.seconds() > 2) {
                robot.grabber.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.grabber.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                grabberDown = true;
                run = false;
            }
        }
        else if (left_bumper2 == true) {
            //robot.grabber.setPower(0.6);
            if(robot.grabber.getCurrentPosition() >= 400) {
                robot.grabber.setPower(0.3);
            }
            else {
                robot.grabber.setPower(0.6);
            }
            robot.grabber.setTargetPosition(550);

        }
        else if(left_trigger2 > 0.3) {
            robot.grabber.setPower(0.35);
            //robot.glyphStop.setPosition(0.6);
            robot.grabber.setTargetPosition(400);
        }
        else if(left_bumper1 == true) {
            robot.grabber.setPower(0.6);
            //robot.glyphStop.setPosition(0.6);
            robot.grabber.setTargetPosition(450);
            robot.claw1.setPosition(0.7);
            robot.claw2.setPosition(0.25);
            run2 = true;
            runtime.reset();
        }
        else if(run2 == true && runtime.seconds() > 2.0) {
            robot.grabber.setPower(0.2);
        }
        else if(run2 == false){
            if(robot.grabber.getCurrentPosition() <= 10) {
                robot.grabber.setPower(0);
            }
            else {
                robot.grabber.setPower(0.6);
            }
            robot.grabber.setTargetPosition(5);
        }
        if(dup2 == true) {
            //robot.glyphStop.setPosition(0.6);
            robot.grabber.setPower(0.2);
            robot.grabber.setTargetPosition(1500);
            grabberReset = true;
        }
        else if(ddown2 == true) {
            //robot.glyphStop.setPosition(0.6);
            robot.grabber.setPower(0.2);
            robot.grabber.setTargetPosition(-1500);
            grabberReset = true;

        }
        else if(grabberReset == true) {
            //Reset the position and encoders of the grabber
            robot.grabber.setPower(0);
            robot.grabber.setTargetPosition(robot.grabber.getCurrentPosition());
            robot.grabber.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.grabber.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            grabberReset = false;
            robot.grabber.setPower(1);
        }

        /*if(left_bumper2 == true || left_trigger2 > 0.3 || dup2 == true || ddown2 == true || robot.grabber.getCurrentPosition() > 10 || grabberDown == false) {
            //robot.glyphStop.setPosition(0.6);
        }
        else{
            //robot.glyphStop.setPosition(0.4);
        }*/
        if(right_trigger2 > 0.4 && left_bumper2 == false && left_trigger2 < 0.3 && robot.grabber.getCurrentPosition() < 20 && dup2 == false && ddown2 == false && run2 == false) {
            robot.glyphStop.setPosition(0.8);
        }
        else {
            robot.glyphStop.setPosition(0.1);
        }
        //wheelie controlls
        if(left_bumper1 == true) {
            robot.wheelie.setPower(-1.0);
            goup = true;
        }
        else if(right_bumper1){
            robot.wheelie.setPower(1.0);
        }
        else {
            robot.wheelie.setPower(0.0);
        }
        //Jewel Remover Controls
        /*if(right_trigger2 > 0.3) {
            robot.jewelGrab.setPosition(0.8);
        }
        else {
            robot.jewelGrab.setPosition(0.19);
        }*/

        //dumper controls
        if (right_bumper2 == true) {
            robot.dumper.setPower(0.5);
            robot.dumper.setTargetPosition(480);
        }
        else if(right_trigger2 > 0.5) {
            if(robot.dumper.getCurrentPosition() >= -200) {
                robot.dumper.setPower(0.6);
            }
            else {
                robot.dumper.setPower(0);
            }
            robot.dumper.setTargetPosition(-300);
            dumperReset = true;
        }
        else if(dumperReset == true) {
            robot.dumper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.dumper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            dumperReset = false;
        }
        else {
            if(robot.dumper.getCurrentPosition() <= 10) {
                robot.dumper.setPower(0);
            }
            else {
                robot.dumper.setPower(0.5);
            }
            robot.dumper.setTargetPosition(0);
        }

        //claw controls
        // OLD NUMBERS -- closed - .76,.24 -- partway - .6,.4
        //closes claws
        if (x_button2 == true || run2 == true) {
            robot.claw1.setPosition(0.51);
            robot.claw2.setPosition(0.49);
        }
        //all the way open
        else if(y_button2 == true) {
            robot.claw1.setPosition(0.7);
            robot.claw2.setPosition(0.3);
        }
        //part way open when not pressing a button
        else {
            robot.claw1.setPosition(0.62);
            robot.claw2.setPosition(0.38);
        }

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
        // Send telemetry message to signify robot running;
        telemetry.addLine();
        telemetry.addData("grabber position", robot.grabber.getCurrentPosition());
        telemetry.addLine();
        telemetry.addLine("Controller Telemetry:");
        telemetry.addData("Gamepade 2 Mode: ", gamepadMode2);
        telemetry.addData("Left Bumper: ", left_bumper2);
        telemetry.addData("Right Bumper: ", right_bumper2 );
        telemetry.addData("Left Trigger: ", left_trigger2);
        telemetry.addData("Right Trigger: ", right_trigger2);
        telemetry.addData("A Button: ", a_button2);
        telemetry.addData("B Button: ", b_button2);
        telemetry.addData("X Button: ", x_button2);
        telemetry.addData("Y Button: ", y_button2);
        telemetry.addData("A Button: ", frontFacing);
        telemetry.addData("B Button: ", rightFacing);
        telemetry.addData("X Button: ", leftFacing);
        telemetry.addData("Y Button: ", yPressed);
        telemetry.addData("2nd Left Trigger",left_stick_x2);
        telemetry.addData("2nd Right Trigger",right_stick_x2);
        telemetry.addData("home",gamepad2.guide);
        telemetry.addData("color 1", robot.jkcolor.blue());
        telemetry.addData("color 1", robot.jkcolor2.blue());
        telemetry.addData("dumper", robot.dumper.getCurrentPosition());
        /*
        telemetry.addData("Ultra back", robot.ultra_back.getDistance(DistanceUnit.CM));
        telemetry.addData("Ultra left", robot.ultra_left.getDistance(DistanceUnit.CM));
        telemetry.addData("Ultra right", robot.ultra_right.getDistance(DistanceUnit.CM));
        */
        telemetry.addData("Ultra Back ", ((robot.ultra_back.getVoltage() / 5) * 512) + 2.5);
        telemetry.addData("Ultra Left ", ((robot.ultra_left.getVoltage() / 5) * 512) + 2.5);
        telemetry.addData("Ultra Right ", ((robot.ultra_right.getVoltage() / 5) * 512) + 2.5);
        telemetry.addLine("What is my name?: Spitz");
        telemetry.addData("rotate",rotate);

    }


    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

    }

}