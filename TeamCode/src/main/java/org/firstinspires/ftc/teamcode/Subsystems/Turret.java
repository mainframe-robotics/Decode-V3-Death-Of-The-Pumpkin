package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Turret {
    private Servo tur;

    private static double gearRatio = 1;///((100.0/30.0)*(70.0/190));
    private static double maxRange = 360-37.8;

    private static final double SERVO_RANGE_DEG = 360.0 - 37.8; // 322.2
    private static final double SERVO_CENTER = 0.525;

    private static final double SERVO_NEG_90 = 0.26;
    private static final double SERVO_POS_90 = 0.81;

    private static final double MAX_NEG_ANGLE = -90.0;
    private static final double MAX_POS_ANGLE = 90.0;



    public static double offset = 0;

    public static double target;
    private double positionDeg;

    public Turret(HardwareMap hardwareMap) {
        tur = hardwareMap.servo.get("tur");
        tur.setDirection(Servo.Direction.REVERSE);

    }
    private static double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }

    public static double angleToServo(double angle) {
        double DEAD = 18.9;
        double TOTAL = 322.2;
        double HALF = 180 - DEAD;   // 161.1

        // Clamp dead zone
        if (angle > -DEAD && angle < DEAD) {
            angle = (angle >= 0) ? DEAD : -DEAD;
        }

        double shifted;
        if (angle >= DEAD) {
            // [18.9 .. 180] → [0 .. 161.1]
            shifted = angle - DEAD;
        } else {
            // [-180 .. -18.9] → [161.1 .. 322.2]
            shifted = HALF + (angle + 180);
        }

        return shifted / TOTAL;
    }

    public double getPositionDeg() {
        return (tur.getPosition()-.525)*maxRange;


    }

    private double wrap360(double angleDeg) {
        double a = angleDeg % 360.0;
        if (a < 0) a += 360.0;
        return a;
    }
    public void setTargetDeg(double angleDeg) {
        target=angleDeg/maxRange+.5;
    }



    public double getTargetDeg() {
        return (target-.525)*maxRange;
    }

    public void update(){
        tur.setPosition(target);
    }
    public static double normalizeAngle(double deg) {

        double angle = deg % 360.0;
        if (angle <= -180) angle += 360;
        if (angle > 180) angle -= 360;
        return angle;
    }
    //3.02 in
//    public void facePoint(Pose targetPose, Pose robotPose,double ballOffset,double turOffset) {
//        Pose ballPose = new Pose(robotPose.getX()+ballOffset*Math.cos(robotPose.getHeading()), robotPose.getY()+ballOffset*Math.sin(robotPose.getHeading()));
//
//
//        double angleToTargetFromCenter = Math.toDegrees(Math.atan2(targetPose.getY() - ballPose.getY(), targetPose.getX() - ballPose.getX()));
//        double robotAngleDiff = normalizeAngle(Math.toDegrees(robotPose.getHeading())-angleToTargetFromCenter );
//        setTargetDeg(robotAngleDiff+turOffset);
//    }
    public void facePoint(Pose targetPose, Pose robotPose) {
        Pose ballPose = new Pose(robotPose.getX()+3*Math.cos(robotPose.getHeading()), robotPose.getY()+3*Math.sin(robotPose.getHeading()));


        double angleToTargetFromCenter = Math.toDegrees(Math.atan2(targetPose.getY() - ballPose.getY(), targetPose.getX() - ballPose.getX()));
        double robotAngleDiff = normalizeAngle(Math.toDegrees(robotPose.getHeading())-angleToTargetFromCenter );
        setTargetDeg(robotAngleDiff);
    }
    public void facePoint(Pose targetPose, Pose robotPose, double distance, double Offset) {
        Pose ballPose = new Pose(robotPose.getX()+3*Math.cos(robotPose.getHeading()), robotPose.getY()+3*Math.sin(robotPose.getHeading()));


        double angleToTargetFromCenter = Math.toDegrees(Math.atan2(targetPose.getY() - ballPose.getY(), targetPose.getX() - ballPose.getX()));
        double robotAngleDiff = normalizeAngle(Math.toDegrees(robotPose.getHeading())-angleToTargetFromCenter );
        if (distance >= 117.5)
        {
            setTargetDeg(robotAngleDiff + Offset);
        }else{
            setTargetDeg(robotAngleDiff);
        }
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

        setTargetDeg(turretAngle);
    }


}
