package org.firstinspires.ftc.teamcode.Tools;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by jedch on 1/17/2018.
 */

@TeleOp(name="Tool: Controller Test", group ="Tools")
public class ControllerTest extends OpMode {

    @Override
    public void init(){}

    @Override
    public void loop(){
        double left_joystick_y, left_joystick_x, right_joystick_y, right_joystick_x;
        double left_trigger, right_trigger;
        boolean left_bumper, right_bumper;
        boolean left_stick_button, right_stick_button;
        boolean a_button, b_button, x_button, y_button;
        boolean d_up, d_down, d_left, d_right;
        boolean home, back, start;
        
        //Left joystick
        left_joystick_y   = gamepad1.left_stick_y;
        left_joystick_x   = gamepad1.left_stick_x;
        left_stick_button = gamepad1.left_stick_button;
        
        //Right Joystick
        right_joystick_y   = gamepad1.right_stick_y;
        right_joystick_x   = gamepad1.right_stick_x;
        right_stick_button = gamepad1.right_stick_button;

        //Bumpers & Triggers
        left_trigger  = gamepad1.left_trigger;
        left_bumper   = gamepad1.left_bumper;
        right_trigger = gamepad1.right_trigger;
        right_bumper  = gamepad1.right_bumper;

        //Face Buttons
        a_button = gamepad1.a;
        b_button = gamepad1.b;
        x_button = gamepad1.x;
        y_button = gamepad1.y;

        //Directional Pad
        d_up    = gamepad1.dpad_up;
        d_down  = gamepad1.dpad_down;
        d_left  = gamepad1.dpad_left;
        d_right = gamepad1.dpad_right;

        //Utility Buttons
        back  = gamepad1.back;
        start = gamepad1.start;
        home  = gamepad1.guide;
        
        String left_joystick_string = left_joystick_x + ", " + left_joystick_y;
        String right_joystick_string = right_joystick_x + ", " + right_joystick_y;

        telemetry.addLine("Joysticks: ");
        telemetry.addData("Left Joystick: ", left_joystick_string);
        telemetry.addData("Left Stick Button: ", left_stick_button);
        telemetry.addData("Right Joystick: ", right_joystick_string);
        telemetry.addData("right Stick Button: ", right_stick_button);
        telemetry.addLine("-------------------------------------------");
        telemetry.addLine("Bumpers & Triggers: ");
        telemetry.addData("Left Bumper: ", left_bumper);
        telemetry.addData("Left Trigger: ", left_trigger);
        telemetry.addData("right Bumper: ", right_bumper);
        telemetry.addData("Right Trigger: ", right_trigger);
        telemetry.addLine("-------------------------------------------");
        telemetry.addLine("Buttons: ");
        telemetry.addData("A: ", a_button);
        telemetry.addData("B: ", b_button);
        telemetry.addData("X: ", x_button);
        telemetry.addData("Y: ", y_button);
        telemetry.addLine("-------------------------------------------");
        telemetry.addLine("D-Pad: ");
        telemetry.addData("Up: ", d_up);
        telemetry.addData("Down: ", d_down);
        telemetry.addData("Left: ", d_left);
        telemetry.addData("Right: ", d_right);
        telemetry.addLine("-------------------------------------------");
        telemetry.addLine("Utility Buttons: ");
        telemetry.addData("Back: ", back);
        telemetry.addData("Start: ", start);
        telemetry.addData("Home: ", home);
        telemetry.update();



         
    }
    
}