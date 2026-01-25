package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp
public class testTurret extends LinearOpMode {
    public static double tru;

    @Override
    public void runOpMode(){
        Servo hinge = hardwareMap.servo.get("hinge");
//        tur.setDirection(Servo.Direction.REVERSE);
        waitForStart();
        while(opModeIsActive()){
            hinge.setPosition(tru);
        }
    }
}
