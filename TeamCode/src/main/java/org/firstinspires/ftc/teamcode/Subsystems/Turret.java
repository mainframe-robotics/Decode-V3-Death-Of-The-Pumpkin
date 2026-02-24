package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@Config
public class Turret {
    private CRServo motor;
    private DcMotorEx encoder;
    public static double ticksPerDeg = 720.0/53669.0; // 384.5 * (170/80d) (??????????)

    private PIDFController bigC,smallC;

    public static double bigKp=0.024,bigKi=0,bigKd=0.0014,bigKf=0,smallKp=0.05,smallKd=0.0007;


    public static boolean on = true;
    private boolean manual = false;

    public static double target =0;
    private double manualPower;
    public static double lowerLimit =-180,upperLimit=135;

    private double prevMotor;



    public Turret(HardwareMap hardwareMap){
        motor=hardwareMap.get(CRServo.class,"tur");
//        motor.setDirection(CRServo.Direction.REVERSE);

        encoder=hardwareMap.get(DcMotorEx.class,"int");



        PIDFCoefficients bigCoff = new PIDFCoefficients(bigKp, bigKi, bigKd, bigKf);
        PIDFCoefficients smallCoff = new PIDFCoefficients(0, 0, 0, 0);

        bigC= new PIDFController(bigCoff);
        smallC = new PIDFController(smallCoff);


    }

    private void setTarget(double x){
        target= Range.clip(x,lowerLimit,upperLimit);
    }
    public double getTarget(){
        return target;
    }

    public double getPosition(){
        return (-encoder.getCurrentPosition()/8192.0)*(58.0/190.0)*360.0;
    }

    public double getNormalixedPos(){ return normalizeAngle(getPosition());}

    public double getPower(){return motor.getPower();}

    public boolean atTarget(){
        return Math.abs(getTarget()-getPosition())<2;
    }

    public void update(){
        if (on){
            if (manual){
                motor.setPower(manualPower);
                return;
            }
            PIDFCoefficients bigCoff = new PIDFCoefficients(bigKp, bigKi, bigKd, bigKf);
            PIDFCoefficients smallCoff = new PIDFCoefficients(smallKp, 0, smallKd, 0);
            bigC.setCoefficients(bigCoff);
            smallC.setCoefficients(smallCoff);
            target= Range.clip(target,lowerLimit,upperLimit);
            bigC.updateError(target-getPosition());
            smallC.updateError(target-getPosition());

            double powr = bigC.run();
            if(Math.abs(bigC.getError())<4){
                powr=smallC.run();
            }

//            if(Math.abs(powr-prevMotor)>.05){
            motor.setPower(powr);
//                prevMotor=(powr);
//            }

        }
        else{
            motor.setPower(0);
        }
    }

    public void manual(double power) {
        manual = true;
        manualPower = power;
    }

    public void automatic() {
        manual = false;
    }

    public void on() {
        on = true;
    }

    public void off() {
        on = false;
    }

    public void setYaw(double deg) {
        deg = normalizeAngle(deg);
        setTarget(deg);
    }



    public static double normalizeAngle(double deg) {

        double angle = deg % 360.0;
        if (angle <= -180) angle += 360;
        if (angle > 180) angle -= 360;
        return angle;
    }



    public void facePoint(Pose targetPose, Pose robotPose,double distance) {
        Pose ballPose = new Pose(robotPose.getX()+3*Math.cos(robotPose.getHeading()), robotPose.getY()+3*Math.sin(robotPose.getHeading()));
        if(distance>=117.5){
            targetPose = new Pose(targetPose.getX()+0,targetPose.getY()+0);
        }

        double angleToTargetFromCenter = Math.toDegrees(Math.atan2(targetPose.getY() - ballPose.getY(), targetPose.getX() - ballPose.getX()));
        double robotAngleDiff = normalizeAngle(Math.toDegrees(robotPose.getHeading())-angleToTargetFromCenter );
        setYaw(robotAngleDiff);
    }
    public void facePoint(Pose targetPose, Pose robotPose, double distance, double Offsetx,double Offsety) {
        Pose ballPose = new Pose(robotPose.getX()+3*Math.cos(robotPose.getHeading()), robotPose.getY()+3*Math.sin(robotPose.getHeading()));
        if(distance>=117.5){
            targetPose = new Pose(targetPose.getX()+Offsetx,targetPose.getY()+Offsety);
        }
        double angleToTargetFromCenter = Math.toDegrees(Math.atan2(targetPose.getY() - ballPose.getY(), targetPose.getX() - ballPose.getX()));
        double robotAngleDiff = normalizeAngle(Math.toDegrees(robotPose.getHeading())-angleToTargetFromCenter );
//        if (distance >= 117.5)
//        {
//            setYaw(robotAngleDiff + Offset);
//        }else{
//            setYaw(robotAngleDiff);
//        }
        setYaw(robotAngleDiff);
    }

    public void facePoint2(Pose targetPose, Pose robotPose) {

        Pose turretCenter = new Pose(
                robotPose.getX() + Math.cos(robotPose.getHeading()),
                robotPose.getY() + Math.sin(robotPose.getHeading())
        );

        double angleToTarget = Math.toDegrees(Math.atan2(
                targetPose.getY() - turretCenter.getY(),
                targetPose.getX() - turretCenter.getX()
        ));

        double turretAngle = normalizeAngle(
                angleToTarget - Math.toDegrees(robotPose.getHeading())
        );

        setYaw(turretAngle);
    }

}
