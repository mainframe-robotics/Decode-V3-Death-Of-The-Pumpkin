package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class testEncoder extends LinearOpMode {

    @Override
    public void runOpMode(){
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        AnalogInput encoder = hardwareMap.get(AnalogInput.class, "sp1");
        DcMotorEx e = hardwareMap.get(DcMotorEx.class,"spin");
        e.setDirection(DcMotorSimple.Direction.FORWARD);
        waitForStart();
        while (opModeIsActive()){
            double position = encoder.getVoltage() / 3.2 * 360;

            double offset = 0;
            double offsetPosition = (encoder.getVoltage() / 3.2 * 360 + offset) % 360;

            e.setPower(gamepad1.left_stick_y);

            telemetry.addData("pos: ",position);
            telemetry.addData("offsetPosition: ",offsetPosition);
            telemetry.addData("encoderPos: ",e.getCurrentPosition());
            telemetry.addData("motorPow: ",e.getPower());
            telemetry.update();
        }
    }
}
