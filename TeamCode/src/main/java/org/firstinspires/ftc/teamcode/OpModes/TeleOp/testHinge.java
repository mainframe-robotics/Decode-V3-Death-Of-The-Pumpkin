package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Subsystems.Turret;

@Config
@TeleOp
public class testHinge extends LinearOpMode {
    public static double tru=.38;


    @Override
    public void runOpMode(){


        Servo hinge = hardwareMap.get(Servo.class, "hinge");




        waitForStart();
        while(opModeIsActive()){
            hinge.setPosition(tru);
        }
    }
}