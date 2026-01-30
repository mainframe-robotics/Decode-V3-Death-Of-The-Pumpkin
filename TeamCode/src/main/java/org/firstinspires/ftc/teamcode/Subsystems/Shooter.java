package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
public class Shooter {
    private DcMotorEx motor,motor2;
    private Servo hood;
    public static double ratio,veloMult=1.356;
    private static final double TICKS_PER_REV = 28.0; // Adjust for your motor encoder CPR

//    private InterpLUT gains;

    private PIDFController b, s;

    public static double t = 0,hoodT;
    private final double hoodRatio= 37.0/540.0, servoRatio=1.0/(270.0);


    public static final double baseAngle=66.2;

    /*
    target = 70
    base = 66.2
    (target-base)/hoodRatio = hoodT

    hoodT*servoRatio= servo pos

     */
    public static double bp = 0.003, bd = 0.0, bf =0.00026, sp = 0.008, sd = 0.000, sf = 0.0;

    public static double pSwitch = 300;

    public static double close = .1,far =.8;
    private boolean activated = true;


    public Shooter(HardwareMap hardwareMap) {
//        gains = new InterpLUT();
////        gains.add(-5 * 60.0 / TICKS_PER_REV,.00059);
////        gains.add(1000 * 60.0 / TICKS_PER_REV,.00057);
////        gains.add(2000 * 60.0 / TICKS_PER_REV,.00049);
////        gains.add(5000 * 60.0 / TICKS_PER_REV,.00043);
////        gains.add(10000 * 60.0 / TICKS_PER_REV,.0004);
//        gains.createLUT();


        b = new PIDFController(new PIDFCoefficients(bp, 0, bd, 0));
        s = new PIDFController(new PIDFCoefficients(sp, 0, sd, 0));
        motor = hardwareMap.get(DcMotorEx.class, "shootL");
        motor2 = hardwareMap.get(DcMotorEx.class, "shootR");

        hood = hardwareMap.get(Servo.class,"hood1");
        //r = hardwareMap.get(DcMotorEx.class, "sr");
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor2.setDirection(DcMotorSimple.Direction.FORWARD);
        hood.setDirection(Servo.Direction.REVERSE);
    }

    public double getTarget() {
        return t;
    }

    public double getVelocity() {

        double ticksPerSecond = motor.getVelocity();
        double currentRPM = ticksPerSecond * 60.0 / TICKS_PER_REV;
        return currentRPM*(37.0/35.0);    }

    public void setPower(double p) {
        motor.setPower(p);
        motor2.setPower(p);

        //r.setPower(p);
    }

    public void off() {
        activated = false;
        setPower(0);
    }

    public void on() {
        activated = true;
    }

    public boolean isActivated() {
        return activated;
    }


    public void setTarget(double velocity) {
        t = velocity;
    }

    public void setHood(double x){
//        77.5-68
        x= Range.clip(x,69,80.4);
        hoodT=(x-baseAngle)/hoodRatio;
    }
    public double getHood(){
        return (hoodT*hoodRatio)+baseAngle;
    }






    public void update() {
        b.setCoefficients(new PIDFCoefficients(Math.abs(getTarget() - getVelocity())<166?bp*.4:bp, 0, bd, 0));
        s.setCoefficients(new PIDFCoefficients(sp, 0, sd, 0));
        hood.setPosition(hoodT*servoRatio+.1);
        if (activated) {
            if (Math.abs(getTarget() - getVelocity()) < pSwitch) {
                s.updateError(getTarget() - getVelocity());
                setPower(getTarget()*bf+s.run());
            } else {
            b.updateError(getTarget() - getVelocity());
            setPower(getTarget()*bf+b.run());
            }
//            motor.setVelocity(getTarget());
        }
        else {
            setPower(0);
        }
    }



    public boolean atTarget() {
        return Math.abs((getTarget()- getVelocity())) < 200;
    }
    //-0.00190711x^{2}+0.438204x+56.27323
    public static double hoodRegA=-0.00190711,hoodRegB=0.438204,hoodRegC=56.27323;

    //y=0.0510743x^{2}+1.90001x+1975.57062
    public static double shooterRegA=0.0510743,shooterRegB=1.90001,shooterRegC=1975.57062;


    public void forDistanceHood(double distance){
        setHood(hoodRegA*Math.pow(distance,2)+hoodRegB*Math.pow(distance,1)+hoodRegC);
    }
    public void forDistance(double distance) {

        setHood(hoodRegA*Math.pow(distance,2)+hoodRegB*Math.pow(distance,1)+hoodRegC);
        setTarget(shooterRegA*Math.pow(distance,2)+shooterRegB*Math.pow(distance,1)+shooterRegC);
    }



}

