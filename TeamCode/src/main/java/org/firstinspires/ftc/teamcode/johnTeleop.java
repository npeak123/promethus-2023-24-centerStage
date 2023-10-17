package org.firstinspires.ftc.teamcode;

import androidx.annotation.Nullable;

//import statements, no idea what any of this is
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.checkerframework.checker.units.qual.Angle;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp

public class johnTeleop extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        //setting the motor names
        DcMotor frontL = hardwareMap.dcMotor.get("frontL");
        DcMotor backL = hardwareMap.dcMotor.get("backL");
        DcMotor frontR = hardwareMap.dcMotor.get("frontR");
        DcMotor backR = hardwareMap.dcMotor.get("backR");
        DcMotor armBottomL = hardwareMap.dcMotor.get("armBottomL");
        DcMotor armBottomR = hardwareMap.dcMotor.get("armBottomR");
        DcMotor upperArm = hardwareMap.dcMotor.get("upperArm");

        //setting right side motors to reverse so we an do 1 and 1 instead of -1 and 1 (hooray)
        frontR.setDirection(DcMotorSimple.Direction.REVERSE);
        backR.setDirection(DcMotorSimple.Direction.REVERSE);
        armBottomR.setDirection(DcMotorSimple.Direction.REVERSE);

        //imu is a class that creates a hardware map, this calls it and makes an object called iamu
        IMU iamu = hardwareMap.get(IMU.class, "iamu");

        //adjusting the orientation parameters to match our bot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.UP,
            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));

        //previous 3 lines don't do anything unless this exists
        iamu.initialize(parameters);

        //important i think
        waitForStart();

        if (isStopRequested()) return;

        //this is the funny loop from the sims
        while (opModeIsActive()) {

            //assigning the dimensions for each control stick on the rich people submarine controllers
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            //rx is sideways movement? not sure yet
            double rx = gamepad1.right_stick_x;
            double yArm = -gamepad2.left_stick_y;

            //resets the yaw on the robot if the options button is pressed
            if (gamepad1.options) {
                iamu.resetYaw();
            }

            //converts the yaw value to radians? ask noah
            double joystickOrient = iamu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            //uses nerd math to calculate rotation
            double rotX = x * Math.cos(-joystickOrient) - y * Math.sin(-joystickOrient);
            double rotY = x * Math.sin(-joystickOrient) + y * Math.cos(-joystickOrient);

            //adds the bigger of the 2 absolute value numbers of rotx, the absolute value of rotx, and the absolute value of rx, also 1
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            //uses nerd math to calculate the power of each motor, also involves the denominator
            double fLPower = (rotY + rotX + rx) / denominator;
            double bLPower = (rotY - rotX + rx) / denominator;
            double fRPower = (rotY - rotX + rx) / denominator;
            double bRPower = (rotY + rotX - rx) / denominator;

            //says what the power is set to initially
            frontL.setPower(fLPower);
            backL.setPower(bLPower);
            frontR.setPower(fRPower);
            backR.setPower(bRPower);
            armBottomL.setPower(yArm);
            armBottomR.setPower(yArm);
        }
    }
}
