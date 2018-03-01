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
        
        //Left joystick
        double left_joystick_y    = gamepad1.left_stick_y;
        double left_joystick_x    = gamepad1.left_stick_x;
        boolean left_stick_button = gamepad1.left_stick_button;
        
        //Right Joystick
        double right_joystick_y    = gamepad1.right_stick_y;
        double right_joystick_x    = gamepad1.right_stick_x;
        boolean right_stick_button = gamepad1.right_stick_button;

        //Bumpers & Triggers
        double left_trigger  = gamepad1.left_trigger;
        boolean left_bumper  = gamepad1.left_bumper;
        double right_trigger = gamepad1.right_trigger;
        boolean right_bumper = gamepad1.right_bumper;

        //Face Buttons
        boolean a_button = gamepad1.a;
        boolean b_button = gamepad1.b;
        boolean x_button = gamepad1.x;
        boolean y_button = gamepad1.y;

        //Directional Pad
        boolean d_up    = gamepad1.dpad_up;
        boolean d_down  = gamepad1.dpad_down;
        boolean d_left  = gamepad1.dpad_left;
        boolean d_right = gamepad1.dpad_right;

        //Utility Buttons
        boolean back  = gamepad1.back;
        boolean start = gamepad1.start;
        boolean home  = gamepad1.guide;
        
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