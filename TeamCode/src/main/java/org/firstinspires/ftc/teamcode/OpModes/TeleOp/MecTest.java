package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class MecTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotorEx frontLeftMotor = hardwareMap.get(DcMotorEx.class,"fl");
        DcMotorEx backLeftMotor = hardwareMap.get(DcMotorEx.class,"bl");
        // DcMotor frontRightMotor = hardwareMap.dcMotor.get("fr");
        // DcMotor backRightMotor = hardwareMap.dcMotor.get("br");

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        // frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        // backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {


            frontLeftMotor.setPower(.6);
            backLeftMotor.setPower(-.6);
            telemetry.addData("leftMotor velo:",frontLeftMotor.getVelocity()/28);
            telemetry.addData("rightMotor velo:",backLeftMotor.getVelocity()/28);
            telemetry.addData("leftMotor pow:",frontLeftMotor.getPower());
            telemetry.addData("rightMotor pow:",backLeftMotor.getPower());
            telemetry.update();

        }
    }
}