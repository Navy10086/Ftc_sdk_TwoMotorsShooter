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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="双人模式")
//@Disabled
public class TeleOp_TwoDrivers extends OpMode
{
    private DcMotorEx leftFront = null;
    private DcMotorEx rightFront = null;
    private DcMotorEx leftRear = null;
    private DcMotorEx rightRear = null;
    private DcMotorEx intake = null;
    private DcMotorEx shooter1 = null;
    private DcMotorEx shooter2= null;
    private Servo shootTrigger = null;
    private Servo slopeChanger = null;
    private boolean isIntake = false;
    private boolean isShoot = false;
    private boolean isSlowly = false;
    private ElapsedTime keyDelay = new ElapsedTime();

    private final double INTAKE_POWER = 1.0;
    private final double TRIGGER_ON = 0.45;
    private final double TRIGGER_OFF = 0.65;
    private double slopePosition = 0.88;
    private final double SLOPE_UP = 0.86;
    private final double SLOPE_DOWN = 0.92;

    @Override
    public void init() {
        leftFront = hardwareMap.get(DcMotorEx.class,"leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class,"leftRear");
        rightFront = hardwareMap.get(DcMotorEx.class,"rightFront");
        rightRear = hardwareMap.get(DcMotorEx.class,"rightRear");
        intake = hardwareMap.get(DcMotorEx.class,"intake");
        shooter1 = hardwareMap.get(DcMotorEx.class,"shooter1");
        shooter2 = hardwareMap.get(DcMotorEx.class,"shooter2");
        shootTrigger = hardwareMap.get(Servo.class,"shootTrigger");
        slopeChanger = hardwareMap.get(Servo.class,"slopeChanger");

        //
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter1.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        keyDelay.reset();
    }

    @Override
    public void loop() {
        //drive base

        double drive = gamepad1.left_stick_y;
        double turn  =  gamepad1.right_stick_x;
        double translation = -gamepad1.left_stick_x;

        if(gamepad1.left_bumper && keyDelay.seconds() > 0.5)
        {
            isSlowly = !isSlowly;
            keyDelay.reset();
        }
        if(isSlowly)
        {
            drive *= 0.7;
            turn *= 0.62;
            translation *= 0.7;
        }
        drive = Math.signum(drive) * drive * drive;
        turn = Math.signum(turn) * turn * turn;
        translation = Math.signum(translation) * translation * translation;

        double[] speed = {
                drive - turn + translation,
                drive - turn - translation,
                drive + turn - translation,
                drive + turn + translation
        };
        double max  = 1.0;
        for(double x : speed)
        {
            if(Math.abs(x) > max)
            {
                max = Math.abs(x);
            }
        }
        if(max > 1.0)
        {
            for(int i = 0; i < 4; i++)
            {
                speed[i] /= max;
            }
        }

        leftFront.setPower(speed[0]);
        leftRear.setPower(speed[1]);
        rightFront.setPower(speed[2]);
        rightRear.setPower(speed[3]);

        //intake
        if(gamepad2.left_bumper && keyDelay.seconds() > 0.3)
        {
            isIntake = !isIntake;
            keyDelay.reset();
        }
        if(gamepad2.right_bumper)
        {
            intake.setPower(-INTAKE_POWER);
        }
        else
        {
            if(isIntake)
            {
                intake.setPower(INTAKE_POWER);
            }
            else
            {
                intake.setMotorDisable();
            }
        }


        //shooter
        if(gamepad2.left_trigger > 0.5 && keyDelay.seconds() > 0.5)
        {
            isShoot = !isShoot;
            keyDelay.reset();
        }
        if(isShoot)
        {
            shooter1.setPower(1.0);
            shooter2.setPower(1.0);
        }
        else {
            shooter1.setMotorDisable();
            shooter2.setMotorDisable();
        }
        //launch trigger
        if(gamepad2.right_trigger > 0.5)
        {
            shootTrigger.setPosition(TRIGGER_ON);
        }
        else
        {
            shootTrigger.setPosition(TRIGGER_OFF);
        }
        //lift with limit

        //slopeChanger
        if(gamepad2.dpad_up && keyDelay.seconds() > 0.3)
        {
            keyDelay.reset();
            if(slopePosition >= SLOPE_UP)
            {
                slopePosition -= 0.02;
            }
        }
        else if(gamepad2.dpad_down && keyDelay.seconds() > 0.3)
        {
            keyDelay.reset();
            if(slopePosition <= SLOPE_DOWN)
            {
                slopePosition += 0.02;
            }
        }
        slopeChanger.setPosition(slopePosition);
        telemetry.addData("slopPosition",slopePosition);
    }

    @Override
    public void stop() {
    }

}
