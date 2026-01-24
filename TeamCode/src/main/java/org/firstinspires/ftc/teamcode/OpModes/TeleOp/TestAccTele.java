package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Transfer;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.Arrays;

@TeleOp
@Config
public class TestAccTele extends LinearOpMode {

    Transfer transfer;
    Turret turret;

    public static double goalX=5,goalY=141;


    public static Pose goalPose = new Pose(goalX, goalY);


    Shooter shooter;
    Follower follower;
    private ElapsedTime timer;

    public static double ange=0.5;
    public static double ball=3;
    public static double turA=0;

    public static double spinMult=1;

//    public static Pose goalPose = new Pose(1,142);

    public static double velo;
    public static boolean auto =true;




    @Override
    public void runOpMode(){


        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        transfer = new Transfer(hardwareMap);
        shooter = new Shooter(hardwareMap);
        DcMotor intake = hardwareMap.dcMotor.get("int");
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72,72,Math.toRadians(90)));
        follower.startTeleopDrive();
        follower.update();

//        turret = new Turret(hardwareMap);

        Servo         tur = hardwareMap.servo.get("tur");
        tur.setDirection(Servo.Direction.REVERSE);




        timer = new ElapsedTime();
        timer.reset();
        String x = "PGP";
        waitForStart();
        while (opModeIsActive()){
            double sec = timer.seconds();
            goalPose = new Pose(goalX, goalY);



//            motor.setPower(gamepad1.left_stick_y);
//            motor2.setPower(gamepad1.left_stick_y);
//            turret.setTargetDeg(ange);
//            if(gamepad1.aWasPressed()){
//                transfer.scan(sec);
//            }
//
//            follower.setTeleOpDrive(
//                    -gamepad1.left_stick_y ,
//                    -gamepad1.left_stick_x ,
//                    -gamepad1.right_stick_x , true);
//
//            if(gamepad1.xWasPressed()){
//                x="GPP";
//                transfer.setTargetDeg(transfer.spin(x),sec);
//            }
//            if(gamepad1.yWasPressed()){
//                x="PGP";
//                transfer.setTargetDeg(transfer.spin(x),sec);
//            }
//            if(gamepad1.bWasPressed()){
//                x="PPG";
//                transfer.setTargetDeg(transfer.spin(x),sec);
//            }
//
//            if(gamepad1.dpadLeftWasPressed()){
//                transfer.setTargetDeg(transfer.wrap360(transfer.getPositionDeg() + 45), sec);
//            }
//
            if(gamepad1.dpadUpWasPressed()){
                transfer.retract();
            } else if (gamepad1.dpadDownWasPressed()) {
                transfer.score();

            }

//
//            if(gamepad1.right_trigger>0){
//                transfer.setAuto();
//                transfer.setTargetDeg(30,sec);
//                transfer.retract();
//            }
//            if(gamepad1.xWasPressed()){
//                transfer.setManual();
//                transfer.manualPower=-.15;
//            }
//            else if(gamepad1.xWasReleased()){
//                transfer.manualPower=0;
//                transfer.setAuto();
//            }

//            if(gamepad1.left_trigger>0){
//                transfer.setManual();
//                transfer.manualPower=-gamepad1.left_trigger;
//            }
//            else{
//                transfer.setAuto();
//                transfer.manualPower=0;
//            }
//            if(gamepad1.left_stick_y!=0){
//                transfer.setManual();
//                transfer.manualPower = gamepad1.left_stick_y*spinMult;
//            }
//            if(auto) {
//                shooter.setTarget(velo);
//                shooter.setHood(ange);
//            }else {
//            shooter.forDistance(Math.hypot(goalPose.getX()-follower.getPose().getX(),goalPose.getY()-follower.getPose().getY()));
//            }

            intake.setPower(gamepad1.left_trigger-gamepad1.right_trigger);
//
//            if(gamepad1.dpad_up){
//                transfer.retract();
//            } else if (gamepad1.dpad_down) {
//                transfer.score();
//            }

//            transfer.setManual();

//            transfer.setTargetDeg(ange,sec);
//            else {
//                transfer.setAuto();
//                transfer.setTargetDeg(ange,sec);
//            }

//            shooter.setHood(ange);


//            turret.setTargetDeg(turA);

            shooter.setTarget(velo);
            shooter.setHood(ange);

            transfer.setManual();
            transfer.manualPower=gamepad1.left_stick_y*spinMult;

            //.5=180/(360-37.8)
//            double xx =ange/(360-37.8)-.0586592;
            double maxRange = 360-37.8;

            tur.setPosition(turA);
            telemetry.addData("pos: ",tur.getPosition());

//            turret.facePoint(goalPose,follower.getPose(),ball,turA);
            transfer.update(sec);
//            turret.update();
            shooter.update();
            follower.update();
            telemetry.addLine(transfer.getMapString());
            telemetry.addLine(transfer.getArrString());
            telemetry.addLine(Arrays.toString(transfer.getOrderArr()));

            telemetry.addData("idx: ",transfer.findBestSlot(transfer.getOrderArr(),x));
            telemetry.addData("goal: ",x);
            telemetry.addData("offset: ", transfer.offset);
            telemetry.addData("targetTranfer: ",transfer.getTargetDeg());
            telemetry.addData("posTranfer: ",transfer.getPositionDeg());
            telemetry.addData("posTranferRaw: ",transfer.getPosition());
            telemetry.addData("posTranferAbs: ",transfer.getPositionDegAbs());

            telemetry.addData("follower pose x:",follower.getPose().getX());
            telemetry.addData("follower pose y:",follower.getPose().getY());
            telemetry.addData("follower pose h:",Math.toDegrees(follower.getPose().getHeading()));
            telemetry.addData("follower dist to goal:", Math.hypot(goalPose.getX()-follower.getPose().getX(),goalPose.getY()-follower.getPose().getY()));

            Pose robotPose = follower.getPose();
            Pose ballPose = new Pose(robotPose.getX()+3*Math.cos(robotPose.getHeading()), robotPose.getY()+3*Math.sin(robotPose.getHeading()));
            telemetry.addData("ball pose x:",ballPose.getX());
            telemetry.addData("ball pose y:",ballPose.getY());
            telemetry.addData("ball dist to goal:", Math.hypot(goalPose.getX()-ballPose.getX(),goalPose.getY()-ballPose.getY()));

            telemetry.addData("shooter tar: ",shooter.getTarget());
            telemetry.addData("shooter velo: ",shooter.getVelocity());
            telemetry.update();

        }
    }
}
