package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Subsystems.Turret;

@TeleOp
public class testMotor extends LinearOpMode {

    @Override
    public void runOpMode(){
        DcMotor fr = hardwareMap.dcMotor.get("fr");
        DcMotor fl = hardwareMap.dcMotor.get("fl");
        DcMotor bl = hardwareMap.dcMotor.get("bl");
        DcMotor br = hardwareMap.dcMotor.get("br");

        Turret turret =new Turret(hardwareMap);

        DcMotor spin = hardwareMap.dcMotor.get("spin");

        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        spin.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        /*
        fr->bl
        fl->fl
        br->br
        bl->fr
         */



        waitForStart();
        while (opModeIsActive()){
            turret.setYaw(0);
            double frPower = (gamepad1.a)?1:0;
            double flPower = (gamepad1.b)?1:0;
            double brPower = (gamepad1.x)?1:0;
            double blPower = (gamepad1.y)?1:0;

            fr.setPower(frPower);
            fl.setPower(flPower);
            br.setPower(brPower);
            bl.setPower(blPower);

            turret.update();



        }
    }
}
