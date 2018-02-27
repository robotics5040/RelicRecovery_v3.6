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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@Autonomous(name="Omnibot: Blue2Simple", group="Omnibot")
//@Disabled
public class Blue2Simple extends AutoPull {

    HardwareOmniRobot robot   = new HardwareOmniRobot();
    ElapsedTime runtime = new ElapsedTime();

    @Override public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, false);

        robot.grabber.setPower(0.75);
        robot.grabber.setTargetPosition(robot.GRABBER_AUTOPOS);

        telemetry.addData("Status", "Ready to run");
        telemetry.update();

        waitForStart();
        runtime.reset();
        robot.claw1.setPosition(0.7);
        robot.claw2.setPosition(0.3);
        telemetry.addLine("Pre JKnocks");
        telemetry.update();
        JewelKnock(robot,"blue");
        DriveFor(robot,0.3,0.0,0.0,0.0);
        robot.wheelie.setPower(1.0);
        DriveFor(robot,1.2,1.0,0.0,0.0);
        robot.wheelie.setPower(0.0);
        DriveFor(robot,0.3,0.0,0.0,0.0);

        DriveFor(robot,1.8,0.0,0.0,0.5);

        boolean dis = false;
        while(dis == false && runtime.seconds() < 26 && opModeIsActive()) {
            double distanceBack = 0;//robot.ultra_back.getDistance(DistanceUnit.CM);

            telemetry.addData("Back", distanceBack);
            telemetry.update();

            if(distanceBack >= 58 && distanceBack <= 60) {
                robot.onmiDrive(0.0,0.0,0.0);
                dis = true;
            }
            else if(distanceBack < 58) {
                robot.onmiDrive(0.0,-0.4,0.0);
            }
            else {
                robot.onmiDrive(0.0,0.4,0.0);
            }
        }
    }


}