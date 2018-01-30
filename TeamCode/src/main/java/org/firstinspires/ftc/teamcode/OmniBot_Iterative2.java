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
 * This file provides basic Telop driving for a Pushbot robot.
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
    double wrist_num = 0;
    boolean run2=false,goup = false,done2 = false,there=true,run =false,done=false,aPressed=true,bPressed=false,xPressed=false,yPressed=false,closed = true;
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
        robot.grabber.setPower(1);
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        double left_stick_x, left_stick_y,right_stick_x,right_stick_y,left_trigger,right_trigger1,LX,RX,rotate=0,front=0,side=0, left_stick_y_2, right_stick_y_2;
        boolean home, b_button1,a_button1,y_button1,x_button1,left_bumper, right_bumper, a_button, b_button, x_button, y_button,dup,ddown,dleft,dright,left_bump1,right_bump1, d_up1,d_down1,d_left1,d_right1,stick_press, stick_press1, a_button_2;


        //note: The joystick goes negative when pushed forwards, so negate it)
        left_stick_x = gamepad1.left_stick_x;
        left_stick_y = gamepad1.left_stick_y;
        right_stick_x = gamepad1.right_stick_x;
        right_stick_y = gamepad1.right_stick_y;

        left_bumper = gamepad2.left_bumper;
        right_bumper = gamepad2.right_bumper;
        left_trigger = gamepad2.left_trigger;
        right_trigger1 = gamepad1.right_trigger;

        left_stick_y_2  = gamepad2.left_stick_y;
        right_stick_y_2 = gamepad2.right_stick_y;

        a_button = gamepad2.a;
        b_button = gamepad2.b;
        x_button = gamepad2.x;
        y_button = gamepad2.y;
        b_button1 = gamepad1.b;
        a_button1 = gamepad1.a;
        y_button1 = gamepad1.y;
        x_button1 = gamepad1.x;
        LX = gamepad2.left_stick_y;
        RX = gamepad2.right_stick_y;
        dup = gamepad2.dpad_up;
        ddown = gamepad2.dpad_down;
        dleft = gamepad2.dpad_left;
        dright = gamepad2.dpad_right;
        left_bump1 = gamepad1.left_bumper;
        right_bump1 = gamepad1.right_bumper;
        d_down1 = gamepad1.dpad_down;
        d_up1 = gamepad1.dpad_up;
        d_left1 = gamepad1.dpad_left;
        d_right1 = gamepad1.dpad_right;
        stick_press = gamepad2.right_stick_button;
        stick_press1 = gamepad2.left_stick_button;
        home = gamepad2.guide;

        robot.dumper.setPower(0.5);

        //slight adjustments for driver
        if(d_down1 == true) {
            left_stick_y = 0.5;
        }
        if(d_up1 == true) {
            left_stick_y = -0.5;
        }
        if(d_left1 == true) {
            left_stick_x = -0.5;
        }
        if(d_right1 == true) {
            left_stick_x = 0.5;
        }

        //changes front of robot for driver using a,b,x,y
        if(y_button1 == true || aPressed == true) {
            front = left_stick_y * -1;
            side = left_stick_x * -1;
            rotate = right_stick_x*-1;

            aPressed = true;
            bPressed = false;
            xPressed = false;
            yPressed = false;
        }
        if(x_button1 == true || bPressed == true) {
            front = left_stick_x;
            side = left_stick_y*-1;
            rotate = right_stick_y*-1;

            aPressed = false;
            bPressed = true;
            xPressed = false;
            yPressed = false;
        }
        if(b_button1 == true || xPressed == true) {
            front = left_stick_x*-1;
            side = left_stick_y;
            rotate = right_stick_y;

            aPressed = false;
            bPressed = false;
            xPressed = true;
            yPressed = false;
        }
        if(a_button1 == true || yPressed == true) {
            front = left_stick_y;
            side = left_stick_x;
            rotate = right_stick_x;

            aPressed = false;
            bPressed = false;
            xPressed = false;
            yPressed = true;
        }

        robot.onmiDrive(side, front, rotate);

        //grabber position
        if(home == true || there == false) {
            if (run == false) {
                //robot.glyphStop.setPosition(0.6);
                robot.grabber.setPower(1.0);
                robot.grabber.setTargetPosition(-1 * robot.GRABBER_AUTOPOS);
                run = true;
                there = false;
                run2 = false;
                runtime.reset();

            } else if (robot.grabber.getCurrentPosition() > ((-1 * robot.GRABBER_AUTOPOS) + 10) && there == false && runtime.seconds() < 2) {
                telemetry.addLine("Waiting to get to bottom");
                telemetry.update();
            } else if (robot.grabber.getCurrentPosition() <= ((-1 * robot.GRABBER_AUTOPOS) + 10) && there == false || runtime.seconds() > 2) {
                robot.grabber.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.grabber.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                there = true;
                run = false;
            }
        }
        else if (left_bumper == true) {
            //robot.glyphStop.setPosition(0.6);
            robot.grabber.setTargetPosition(1100);

        }
        else if(left_trigger > 0.3) {
            //robot.glyphStop.setPosition(0.6);
            robot.grabber.setTargetPosition(750);
        }
        else if(left_bump1 == true) {
            //robot.glyphStop.setPosition(0.6);
            robot.grabber.setTargetPosition(950);
            robot.claw1.setPosition(0.7);
            robot.claw2.setPosition(0.25);
            run2 = true;
            runtime.reset();
        }
        else if(run2 == true && runtime.seconds() > 2.0) {
            robot.grabber.setPower(0);
        }
        else if(run2 == false){
            robot.grabber.setTargetPosition(0);
        }
        if(dup == true) {
            //robot.glyphStop.setPosition(0.6);
            robot.grabber.setPower(0.2);
            robot.grabber.setTargetPosition(1500);
            done = true;
        }
        else if(ddown == true) {
            //robot.glyphStop.setPosition(0.6);
            robot.grabber.setPower(0.2);
            robot.grabber.setTargetPosition(-1500);
            done = true;

        }
        else if(done == true) {
            robot.grabber.setPower(0);
            robot.grabber.setTargetPosition(robot.grabber.getCurrentPosition());
            robot.grabber.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.grabber.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            done = false;
            robot.grabber.setPower(1);
        }

        /*if(left_bumper == true || left_trigger > 0.3 || dup == true || ddown == true || robot.grabber.getCurrentPosition() > 10 || there == false) {
            //robot.glyphStop.setPosition(0.6);
        }
        else{
            //robot.glyphStop.setPosition(0.4);
        }*/
        if(right_trigger1 > 0.4) {
            robot.glyphStop.setPosition(1);
        }
        else {
            robot.glyphStop.setPosition(0.1);
        }
        //wheelie controlls
        if(left_bump1 == true) {
            robot.wheelie.setPower(-1.0);
            goup = true;
        }
        else if(right_bump1){
            robot.wheelie.setPower(1.0);
        }
        else {
            robot.wheelie.setPower(0.0);
        }
        //Jewel Remover Controls
        /*if(right_trigger1 > 0.3) {
            robot.jewelGrab.setPosition(0.8);
        }
        else {
            robot.jewelGrab.setPosition(0.19);
        }*/

        //dumper controls
        if (right_bumper == true) {
            robot.dumper.setTargetPosition(480);
        }
        else {
            robot.dumper.setTargetPosition(0);
        }

        //claw controls
        // OLD NUMBERS -- closed - .76,.24 -- partway - .6,.4
        //closes claws
        if (x_button == true || run2 == true) {
            robot.claw1.setPosition(0.51);
            robot.claw2.setPosition(0.49);
        }
        //all the way open
        else if(y_button == true) {
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

        if(right_stick_y_2 < -0.1 && robot.relicMotor.getCurrentPosition() < RELIC_OUT){
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

        if(a_button == true){
            robot.relicClaw.setPosition(0.0);
        }
        else if(b_button == true) {
            robot.relicClaw.setPosition(0.46);
        }
        else{
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
        // Send telemetry message to signify robot running;
        telemetry.addLine("Controller Telemetry:");
        telemetry.addData("Left Bumper: ", left_bumper);
        telemetry.addData("Right Bumper: ", right_bumper );
        telemetry.addData("Left Trigger: ", left_trigger);
        telemetry.addData("Right Trigger: ", right_trigger1);
        telemetry.addData("A Button: ", a_button);
        telemetry.addData("B Button: ", b_button);
        telemetry.addData("X Button: ", x_button);
        telemetry.addData("Y Button: ", y_button);
        telemetry.addData("A Button: ", aPressed);
        telemetry.addData("B Button: ", bPressed);
        telemetry.addData("X Button: ", xPressed);
        telemetry.addData("Y Button: ", yPressed);
        telemetry.addData("2nd Left Trigger",LX);
        telemetry.addData("2nd Right Trigger",RX);
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