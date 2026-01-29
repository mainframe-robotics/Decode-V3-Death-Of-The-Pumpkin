package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Subsystems.Turret;

@Config
@TeleOp
public class testTurret extends LinearOpMode {
    public static double tru;


    @Override
    public void runOpMode(){
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//        Servo hinge = hardwareMap.servo.get("hinge");
//        CRServo tur =hardwareMap.crservo.get("tur");
//        Turret turret = new Turret(hardwareMap);
        DcMotorEx enc = hardwareMap.get(DcMotorEx.class,"fr");


//        tur.setDirection(Servo.Direction.REVERSE);
        waitForStart();
        while(opModeIsActive()){
//            turret.update();
            telemetry.addData("power: ", enc.getCurrentPosition());
            telemetry.update();
        }
    }
}
