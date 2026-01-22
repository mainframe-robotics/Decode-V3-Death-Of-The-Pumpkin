package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    private DcMotor motor;

    private double motorPower=0;

    public Intake(HardwareMap hardwareMap){
        motor = hardwareMap.get(DcMotor.class,"int");
    }

    public void update(){
        motor.setPower(motorPower);
    }

    public void intake(){
        motor.setPower(1);
    }
    public void outtake(){
        motor.setPower(-1);
    }
    public void setPower(double x){
        motorPower =x;
    }
    public double getPower(){
        return motor.getPower();
    }









}
