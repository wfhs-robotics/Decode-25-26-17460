
/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.drive;

import android.graphics.Color;

import androidx.xr.runtime.math.Pose;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.acmerobotics.dashboard.config.Config;



/*
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="Drive", group="Iterative OpMode")
@Config

public class Drive extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private SampleMecanumDrive drive;

    private  DcMotor launchLeft = null;
    private  DcMotor launchRight = null;
    private  Servo revolver = null;
    private  DcMotor intake = null;
    private Servo wrist = null;
    private Servo intakeArm=null;
    private RevColorSensorV3 color;
    boolean prevA = false;
    boolean prevY = false;
    boolean prevX = false;
    boolean prevB = false;
    boolean noprevB= false;
    boolean prevRightStick =false;
    //revolver variables
    public static double Pos1=0.55;
    public static double Pos2=0.13;
    public static double Pos3=0.199;
    public static double Alt1 = 0.315;
    public static double Alt2 = 0.47;
    public static double Alt3 = 0.387;
    public  static double wP1 = .022;
    public  static double wP2 = .163;
    public  static double Wee = .8;

    int posIndex = 0;
    int modeIndex = 0;

    boolean prevLB = false;
    boolean prevRB = false;
    double posMath =0;
    double TPR = 384.5; //Ticks per rot


    /* The colorSensor field will contain a reference to our color sensor hardware object */
    NormalizedColorSensor colorSensor;

    private HuskyLens huskyLens;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        drive = new SampleMecanumDrive(hardwareMap);

        launchLeft = hardwareMap.get(DcMotor.class, "launchleft");
        launchRight = hardwareMap.get(DcMotor.class, "launchright");
        intake = hardwareMap.get(DcMotor.class, "intake");
        revolver =hardwareMap.get(Servo.class, "revolver");
        wrist =hardwareMap.get(Servo.class, "wrist");
        intakeArm =hardwareMap.get(Servo.class, "intakeArm");
        color = hardwareMap.get(RevColorSensorV3.class, "color");

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    //int servo


    @Override
    public void loop() {
        // Setup variables
        double RT;
        double shoot;
        double Intake;


        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.
        //
        drive.setWeightedDrivePower(
                new Pose2d(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x,
                        -gamepad1.right_stick_x
                )
        );

        //shoot the artifacts
        if(gamepad2.right_trigger==0)
            RT=0;
        else
            RT=1;

        if (RT==1)
            shoot = -Wee;
        else
            shoot = 0;

        //intake
        if (gamepad2.y)
            Intake = -1;
        else
            Intake = 0;

        //revolver
        // ========= Right Stick Button BUTTON TOGGLES MODES =========
        if (gamepad2.right_stick_button && !prevRightStick) {
            modeIndex = 1 - modeIndex;   // toggles 0 ↔ 1
            posIndex = 0;                // resets to first position
        }
        prevRightStick = gamepad2.right_stick_button;


// ========= LB/RB CHANGE POSITIONS (0, 1, 2) =========
        if (gamepad2.left_bumper && !prevLB) {
            posIndex++;
        }
        if (gamepad2.right_bumper && !prevRB) {
            posIndex--;
        }

        prevLB = gamepad2.left_bumper;
        prevRB = gamepad2.right_bumper;

// wrap around
        if (posIndex > 2) posIndex = 0;
        if (posIndex < 0) posIndex = 2;


// ========= SELECT TARGET POSITION BASED ON MODE =========
        double target = 0;

        if (modeIndex == 0) {
            // original 3 positions
            if (posIndex == 0) target = Pos1;
            if (posIndex == 1) target = Pos2;
            if (posIndex == 2) target = Pos3;
        } else {
            // alternate 3 positions
            if (posIndex == 0) target = Alt1;
            if (posIndex == 1) target = Alt2;
            if (posIndex == 2) target = Alt3;
        }

// ========= APPLY MOVEMENT =========
        revolver.setPosition(target);




        if (gamepad2.x)
            intakeArm.setPosition(.4);
        else
            intakeArm.setPosition(.05);


        //toggle logic
        if (gamepad2.b && !prevB) {
            noprevB = !noprevB;

        }
        prevB = gamepad2.b;

        //launch angle
        if (noprevB) {
            wrist.setPosition(wP1);
        } else
            wrist.setPosition(wP2);

        //toggle buttons
        prevY = gamepad2.y;
        prevA = gamepad2.a;
        prevX = gamepad2.x;

        // Send calculated power to wheels

        //send power to other motors
        launchLeft.setPower(shoot);
        launchRight.setPower(-shoot);
        intake.setPower(Intake);


        //detect color
        // Get the normalized colors from the sensor
        NormalizedRGBA colors = color.getNormalizedColors();

        telemetry.addData("target", target);
        telemetry.addData("revolverPos", revolver.getPosition());
        //show wrist pos
        telemetry.addData("wristPos", wrist.getPosition());
        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("posMath", posMath);
        telemetry.update();


    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop () {
    }

}
