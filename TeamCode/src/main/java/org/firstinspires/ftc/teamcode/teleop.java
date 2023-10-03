package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class teleop extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        //Setting the motors
        DcMotor frontL = hardwareMap.dcMotor.get("frontL");
        DcMotor backL = hardwareMap.dcMotor.get("backL");
        DcMotor frontR = hardwareMap.dcMotor.get("frontR");
        DcMotor backR = hardwareMap.dcMotor.get("backR");

        //Setting the right side motors to revearse
        frontR.setDirection(DcMotorSimple.Direction.REVERSE);
        backR.setDirection(DcMotorSimple.Direction.REVERSE);

        IMU iamu =  hardwareMap.get(IMU.class, "iamu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        iamu.initialize(parameters);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            if (gamepad1.options) {
                iamu.resetYaw();
            }

            double joyStickOrient = iamu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);


            double rotX = x * Math.cos(-joyStickOrient) - y * Math.sin(-joyStickOrient);
            double rotY = x * Math.sin(-joyStickOrient) + y * Math.cos(-joyStickOrient);

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double fLPower = (rotY + rotX + rx) / denominator;
            double bLPower = (rotY - rotX + rx) / denominator;
            double fRPower = (rotY - rotX - rx) / denominator;
            double bRPower = (rotY + rotX - rx) / denominator;
            
            frontL.setPower(fLPower);
            backL.setPower(bLPower);
            frontR.setPower(fRPower);
            backR.setPower(bRPower);
            
        }
    }

}
