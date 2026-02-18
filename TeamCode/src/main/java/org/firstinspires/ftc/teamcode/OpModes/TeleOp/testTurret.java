package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.Transfer;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;

@Config
@TeleOp
public class testTurret extends LinearOpMode {
    public static double tru;

    Transfer transfer;

    ElapsedTime timer;


    @Override
    public void runOpMode(){
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//        Servo hinge = hardwareMap.servo.get("hinge");
//        CRServo tur =hardwareMap.crservo.get("tur");
        Turret turret = new Turret(hardwareMap);
        DcMotorEx e = hardwareMap.get(DcMotorEx.class,"bl");
        transfer = new Transfer(hardwareMap);
        timer = new ElapsedTime();
        timer.reset();
//        DcMotorEx enc = hardwareMap.get(Dc MotorEx.class,"fr");
//        enc.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


//        tur.setDirection(Servo.Direction.REVERSE);
        waitForStart();
        while(opModeIsActive()){
            turret.setYaw(tru);
            turret.update();
            transfer.update(timer.seconds());

            if(gamepad1.aWasPressed()){
                if (!gamepad1.isRumbling())  // Check for possible overlap of rumbles.
                    gamepad1.rumbleBlips(5);
            }
            telemetry.addData("pos: ", turret.getPosition());
            telemetry.addData("target: ", turret.getTarget());
            telemetry.addData("pow: ", turret.getPower());
            telemetry.addData("touchpad 1 x: ", gamepad1.touchpad_finger_1_x);
            telemetry.addData("touchpad 1 y: ", gamepad1.touchpad_finger_1_y);
            telemetry.addData("touchpad 2 x: ", gamepad1.touchpad_finger_2_x);
            telemetry.addData("touchpad 2 y: ", gamepad1.touchpad_finger_2_y);
            telemetry.update();
        }
    }
}
