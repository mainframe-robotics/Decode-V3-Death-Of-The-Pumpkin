package org.firstinspires.ftc.teamcode.OpModes.Auton; // make sure this aligns with class location

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.InvertedFTCCoordinates;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Transfer;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Config
@Autonomous(name = "Red18Ball", group = "Examples")
public class Red18BallClose extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private String motif = "PGP";

    public static double goalX=5,goalY=141.4;


    public static Pose goalPose = new Pose(goalX,goalY).mirror();

    private final Pose startPose = new Pose(23.38, 125.18377760519193, Math.toRadians(139.8)).mirror();
    private final Pose shoot1Pose = new Pose(63, 75, Math.toRadians(180)).mirror();
    private final Pose intake1Pose = new Pose(20, 59, Math.toRadians(180)).mirror();
    private final Pose intake1PoseControl = new Pose(50, 60).mirror();

//    private final Pose gatePose = new Pose(18, 71, Math.toRadians(180));

    private final Pose shoot2Pose = new Pose(63, 75, Math.toRadians(180)).mirror();//old x: 50, y: 83.4

    private final Pose intake2PrimePose = new Pose(13, 66, Math.toRadians(175)).mirror();
    private final Pose intake2Pose = new Pose(10, 50, Math.toRadians(100)).mirror();

    private final Pose intake2ControlPose = new Pose(19,56).mirror();


//    private final Pose intake2PoseControl = new Pose(41, 31.5);


    private final Pose shoot3Pose = new Pose(63, 75, Math.toRadians(180)).mirror();// old x: 70, y: 74

    private final Pose intake3PrimePose = new Pose(18, 68, Math.toRadians(175)).mirror();
    private final Pose intake3Pose = new Pose(10, 50, Math.toRadians(100)).mirror();

    private final Pose intake3ControlPose = new Pose(19,56).mirror();

    private final Pose shoot4Pose = new Pose(63, 75, Math.toRadians(180)).mirror();// old x: 70, y: 74

    private final Pose intake4Pose = new Pose(20, 36, Math.toRadians(180)).mirror();

    private final Pose intake4PoseControl = new Pose(52, 35).mirror();

    private final Pose shoot5Pose = new Pose(61, 84, Math.toRadians(180)).mirror();// old x: 70, y: 74


    private final Pose intake5Pose = new Pose(21, 84, Math.toRadians(180)).mirror();
    //    private final Pose intake2PoseControl = new Pose(41, 31.5);
    private final Pose shoot6Pose = new Pose(63, 84, Math.toRadians(180)).mirror();// old x: 70, y: 74
    private final Pose leavePose = new Pose(56,84,Math.toRadians(180)).mirror();

    private int pathState;
    private PathChain scorePreload,intakeSet1, scoreSet1,intakeSet2,intakePrimeSet2, scoreSet2, intakeSet3,intakePrimeSet3, scoreSet3,intakeSet4,scoreSet4,intakeSet5,scoreSet5,leave;
    private double intPow;

    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */


        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shoot1Pose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shoot1Pose.getHeading())
                .addParametricCallback(0,()->follower.setMaxPower(1))
                .addParametricCallback(.8,()->follower.setMaxPower(.3))
                .build();

    /* Here is an example for Constant Interpolation
    scorePreload.setConstantInterpolation(startPose.getHeading()); */

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        intakeSet1 = follower.pathBuilder()
                .addPath(new BezierCurve(shoot1Pose, intake1PoseControl, intake1Pose))
                .setTangentHeadingInterpolation()
                .addParametricCallback(0,()->follower.setMaxPower(1))
                .build();


        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scoreSet1 = follower.pathBuilder()
                .addPath(new BezierLine(intake1Pose, shoot2Pose))
                .setTangentHeadingInterpolation()
                .addParametricCallback(0.0,()->follower.setMaxPower(1))
                .addParametricCallback(.7,()->follower.setMaxPower(.25))
                .setReversed()
                .build();

        intakePrimeSet2=follower.pathBuilder()
                .addPath(new BezierLine(shoot2Pose,intake2PrimePose))
                .setLinearHeadingInterpolation(shoot2Pose.getHeading(),intake2PrimePose.getHeading())
                .addParametricCallback(0.0,()->follower.setMaxPower(1))
                .addParametricCallback(.6,()->follower.setMaxPower(.25))
                .build();

        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        intakeSet2 = follower.pathBuilder()
                .addPath(new BezierCurve(intake2PrimePose,intake2ControlPose, intake2Pose))
                .setLinearHeadingInterpolation(intake2PrimePose.getHeading(),intake2Pose.getHeading())
                .addParametricCallback(0.0,()->follower.setMaxPower(1))

                .build();

        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scoreSet2 = follower.pathBuilder()
                .addPath(new BezierLine(intake2Pose, shoot4Pose))
                .setLinearHeadingInterpolation(intake2Pose.getHeading(),shoot4Pose.getHeading())
                .addParametricCallback(0.0,()->follower.setMaxPower(1))
                .addParametricCallback(.8,()->follower.setMaxPower(.35))
                .build();

//        intakePrimeSet3=follower.pathBuilder()
//                .addPath(new BezierLine(shoot3Pose,intake3PrimePose))
//                .setLinearHeadingInterpolation(shoot3Pose.getHeading(),intake3PrimePose.getHeading())
//                .addParametricCallback(0.0,()->follower.setMaxPower(1))
//                .addParametricCallback(.8,()->follower.setMaxPower(.5))
//                .build();
//
//        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
//        intakeSet3 = follower.pathBuilder()
//                .addPath(new BezierCurve(intake3PrimePose,intake3ControlPose, intake3Pose))
//                .setLinearHeadingInterpolation(intake3PrimePose.getHeading(),intake3Pose.getHeading())
//                .addParametricCallback(0.0,()->follower.setMaxPower(1))
//
//                .build();
//
//        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
//        scoreSet3 = follower.pathBuilder()
//                .addPath(new BezierLine(intake3Pose, shoot4Pose))
//                .setLinearHeadingInterpolation(intake3Pose.getHeading(),shoot4Pose.getHeading())
//                .addParametricCallback(0.0,()->follower.setMaxPower(1))
//                .addParametricCallback(.8,()->follower.setMaxPower(.3))
//
//                .build();



        intakeSet4 = follower.pathBuilder()
                .addPath(new BezierCurve(shoot4Pose,intake4PoseControl,intake4Pose))
                .setTangentHeadingInterpolation()
                .addParametricCallback(0.0,()->follower.setMaxPower(1))
                .build();

        scoreSet4 = follower.pathBuilder()
                .addPath(new BezierLine(intake4Pose,shoot5Pose))
                .setTangentHeadingInterpolation()
                .setReversed()
                .addParametricCallback(0.0,()->follower.setMaxPower(1))
                .addParametricCallback(.6,()->follower.setMaxPower(.3))
                .build();


        intakeSet5 = follower.pathBuilder()
                .addPath(new BezierLine(shoot5Pose,intake5Pose))
                .setTangentHeadingInterpolation()
                .addParametricCallback(0.0,()->follower.setMaxPower(1))
                .addParametricCallback(.8,()->follower.setMaxPower(.4))
                .build();

        scoreSet5 = follower.pathBuilder()
                .addPath(new BezierLine(intake5Pose,shoot6Pose))
                .setTangentHeadingInterpolation()
                .setReversed()
                .addParametricCallback(0.0,()->follower.setMaxPower(1))
                .addParametricCallback(.7,()->follower.setMaxPower(.25))
                .build();

        /* This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        leave = follower.pathBuilder()
                .addPath(new BezierLine(shoot6Pose, leavePose))
                .setLinearHeadingInterpolation(shoot6Pose.getHeading(), leavePose.getHeading())
                .build();


    }

    private Position cameraPosition = new Position(DistanceUnit.INCH,
            0, -3.25, 8.25, 0);
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, 180, -65, 180, 0);

    private AprilTagProcessor aprilTag;

    private VisionPortal visionPortal;


    private void initAprilTag(HardwareMap hardwareMap) {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()
                .setCameraPose(cameraPosition, cameraOrientation)
                .setLensIntrinsics(622.001, 622.001, 319.803, 241.251)
                .build();


        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCameraResolution(new Size(640, 480));
        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        builder.addProcessor(aprilTag);
        visionPortal = builder.build();
    }

    //Fill this out to get the robot Pose from the camera's output (apply any filters if you need to using follower.getPose() for fusion)
    //Pedro Pathing has built-in KalmanFilter and LowPassFilter classes you can use for this
    //Use this to convert standard FTC coordinates to standard Pedro Pathing coordinates

    double myXg = 0;
    double myYg = 0;
    double myYawg = 0;
    double myX = 0;
    double myY = 0;
    double myYaw = 0;

    private void setRobotPoseFromCamera() {


        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            // Look to see if we have size info on this tag.
            if (detection.metadata != null) {
                //  Check to see if we want to track towards this tag.
                if (detection.id == 20 || detection.id == 24) {
                    myX = detection.robotPose.getPosition().x;
                    myY = detection.robotPose.getPosition().y;
                    myYaw = detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES);
//                    sleep(500);
                }
                //PPG
                if (detection.id == 23) {
                    motif = "PPG";
                }
                //PGP
                if (detection.id == 22) {
                    motif = "PGP";
                }
                //GPP
                if (detection.id == 21) {
                    motif = "GPP";
                }
            }
        }

        Pose ftcStandard = PoseConverter.pose2DToPose(new Pose2D(DistanceUnit.INCH, -myY - 72, myX - 72, AngleUnit.DEGREES, myYaw), InvertedFTCCoordinates.INSTANCE);
//        ftcStandard=ftcStandard.getAsCoordinateSystem(PedroCoordinates.INSTANCE);
        if (!currentDetections.isEmpty()) {
            follower.setPose(new Pose(myY + 72, 72 - myX, Math.toRadians(myYaw)));
        }
        myXg = follower.getPose().getX();
        myYg = follower.getPose().getY();
        myYawg = Math.toDegrees(follower.getPose().getHeading());
    }


    Shooter shooter;
    Transfer transfer;

    DcMotor intake;

    ElapsedTime readyStateTimer,shootStateTimer;



    public boolean shooterOn=false;

    private double turTarg;


    public void autonomousPathUpdate(double sec,double dist) {
        switch (pathState) {
            case 0:
                intPow=0;
                turTarg=54;
                transfer.setManual();
                transfer.score();
                follower.followPath(scorePreload);
                shooterOn=true;
                setPathState(104);
                break;
            case 104:
                if(pathTimer.getElapsedTimeSeconds()>2&&!follower.isBusy()){
                    transfer.startTransfer(dist);
                    setPathState(105);
                }
                break;
            case 105:
                if(pathTimer.getElapsedTimeSeconds()>.8){
                    transfer.endTransfer();
                    transfer.setAuto();
                    transfer.retract();
                    transfer.setTargetDeg(transfer.wrap360(-20),sec);
//                    intake.setPower(-1);
                    shooterOn=false;
                    setPathState(1);
                }
                break;
            case 1:
//                intake.setPower(-1);


                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if (pathTimer.getElapsedTimeSeconds() > 0) {
                    /* Grab Sample */
//                    transfer.setTargetDeg(30,opmodeTimer.getElapsedTimeSeconds());
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    transfer.retract();
                    transfer.retract();
                    intPow=-1;
                    transfer.setTargetDeg(transfer.wrap360(-20),sec);
                    follower.followPath(intakeSet1, true);
                    setPathState(2);
                }
                break;
//            case 12:
//                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1) {
//                    /* Grab Sample */
////                    transfer.setTargetDeg(30,opmodeTimer.getElapsedTimeSeconds());
//                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
////                    intake.setPower(0);
//                    follower.followPath(hitGate, true);
//                    setPathState(2);
//                }
//                break;
            case 2:
//                intake.setPower(-1);

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1.3) {
                    /* Grab Sample */
//                    transfer.setTargetDeg(30,opmodeTimer.getElapsedTimeSeconds());
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
//                    intake.setPower(0);
                    //turTarg=55;
                    turTarg=74;
                    transfer.retract();
                    shooterOn=true;
                    follower.followPath(scoreSet1, true);
                    setPathState(300);
                }
                break;

            case 300:
                if(pathTimer.getElapsedTimeSeconds()>1.2){
//                    intake.setPower(0);
//                    intPow=1;
                    transfer.spinToScore(sec);
                    setPathState(301);

                }
                break;
            case 301:
                if (transfer.atTarget()){
                    transfer.score();
                    setPathState(302);
                } else if (pathTimer.getElapsedTimeSeconds()>2) {
                    intPow=1;
                }
                break;
            case 302:
                if(pathTimer.getElapsedTimeSeconds()>.5){
//                    intake.setPower(0);
                    transfer.setTargetDeg(transfer.wrap360(transfer.getPositionDeg() + 45), sec);
                    setPathState(303);
                }
                break;
            case 303:
                if(!follower.isBusy()&&shooter.atTarget()&& transfer.atTarget()&&turret.atTarget()){
                    setPathState(304);
                }
                else if (pathTimer.getElapsedTimeSeconds()>1.5) {
                    intPow=1;
                }
                break;
            case 304:
                if(pathTimer.getElapsedTimeSeconds()>.25){
                    transfer.startTransfer(dist);
                    setPathState(305);
                }
                break;
            case 305:
                if(pathTimer.getElapsedTimeSeconds()>1){
                    transfer.endTransfer();
                    transfer.setAuto();
                    transfer.retract();
                    transfer.setTargetDeg(transfer.wrap360(-20),sec);
                    intPow=-1;
                    shooterOn=false;
                    setPathState(3);
                }
                break;


            case 3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0) {
                    /* Score Sample */
//                    transfer.setTargetDeg(240,opmodeTimer.getElapsedTimeSeconds());

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */

                    intPow=0;
                    transfer.retract();
                    transfer.retract();
                    transfer.setTargetDeg(transfer.wrap360(-20),sec);
                    follower.followPath(intakePrimeSet2, true);

                    setPathState(4);
                }
                break;
            case 4:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > .5) {
                    /* Grab Sample */
//                    transfer.setTargetDeg(30,opmodeTimer.getElapsedTimeSeconds());
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    intPow=-1;
                    follower.followPath(intakeSet2, true);
                    setPathState(45);
                }
                break;
            case 45:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2) {
                    /* Grab Sample */
//                    transfer.setTargetDeg(30,opmodeTimer.getElapsedTimeSeconds());
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    turTarg=54;
                    shooterOn=true;
                    follower.followPath(scoreSet2, true);
                    setPathState(500);
                }
                break;
            case 500:
                if(pathTimer.getElapsedTimeSeconds()>0&&follower.getCurrentTValue()>.08){
//                    intake.setPower(0);
                    intPow=1;
                    transfer.spinToScore(sec);
                    setPathState(501);

                }
                break;
            case 501:
                if (transfer.atTarget()){
//                    intake.setPower(0);
                    transfer.score();
                    setPathState(502);
                }
                else if (pathTimer.getElapsedTimeSeconds()>1) {
                    intPow=1;
                }
                break;
            case 502:
                if(pathTimer.getElapsedTimeSeconds()>1){
                    transfer.setTargetDeg(transfer.wrap360(transfer.getPositionDeg() + 45), sec);
                    setPathState(503);
                }
                break;
            case 503:
                if(!follower.isBusy()&&shooter.atTarget()&& transfer.atTarget()&&turret.atTarget()){
                    setPathState(504);
                }
                else if (pathTimer.getElapsedTimeSeconds()>1) {
                    intPow=1;
                }
                break;
            case 504:
                if(pathTimer.getElapsedTimeSeconds()>.25){
                    transfer.startTransfer(dist);
                    setPathState(505);
                }
                break;
            case 505:
                if(pathTimer.getElapsedTimeSeconds()>1){
                    transfer.endTransfer();
                    transfer.setAuto();
                    transfer.retract();
                    transfer.setTargetDeg(transfer.wrap360(-20),sec);
                    intPow=-1;
                    shooterOn=false;
                    setPathState(8);
                }
                break;

//            case 5:
//                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
//                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0) {
//                    /* Score Sample */
////                    transfer.setTargetDeg(240,opmodeTimer.getElapsedTimeSeconds());
//
//                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
//
////                    intake.setPower(-1);
//                    intPow=0;
//                    transfer.retract();
//                    transfer.retract();
//                    transfer.setTargetDeg(transfer.wrap360(-20),sec);
//                    follower.followPath(intakePrimeSet3, true);
//
//                    setPathState(6);
//                }
//                break;
//            case 6:
//                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
//                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > .5) {
//                    /* Grab Sample */
////                    transfer.setTargetDeg(30,opmodeTimer.getElapsedTimeSeconds());
//                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
//                    intPow=-1;
//                    follower.followPath(intakeSet3, true);
//                    setPathState(65);
//                }
//                break;
//            case 65:
//                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
//                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2) {
//                    /* Grab Sample */
////                    transfer.setTargetDeg(30,opmodeTimer.getElapsedTimeSeconds());
//                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
//                    turTarg=54;
//                    shooterOn=true;
//                    follower.followPath(scoreSet3, true);
//                    setPathState(700);
//                }
//                break;
//            case 700:
//                if(pathTimer.getElapsedTimeSeconds()>0&&follower.getCurrentTValue()>.08){
////                    intake.setPower(0);
//                    intPow=1;
//                    transfer.spinToScore(sec);
//                    setPathState(701);
//
//                }
//                break;
//            case 701:
//                if (transfer.atTarget()){
////                    intake.setPower(0);
//                    transfer.score();
//                    setPathState(702);
//                }
//                else if (pathTimer.getElapsedTimeSeconds()>1) {
//                    intPow=1;
//                }
//                break;
//            case 702:
//                if(pathTimer.getElapsedTimeSeconds()>.5){
//                    transfer.setTargetDeg(transfer.wrap360(transfer.getPositionDeg() + 45), sec);
//                    setPathState(703);
//                }
//                break;
//            case 703:
//                if(!follower.isBusy()&&shooter.atTarget()&& transfer.atTarget()){
//                    setPathState(704);
//                }
//                else if (pathTimer.getElapsedTimeSeconds()>2) {
//                    intPow=1;
//                }
//                break;
//            case 704:
//                if(pathTimer.getElapsedTimeSeconds()>.75){
//                    transfer.startTransfer(.8,true);
//                    setPathState(705);
//                }
//                break;
//            case 705:
//                if(pathTimer.getElapsedTimeSeconds()>1){
//                    transfer.endTransfer();
//                    transfer.setAuto();
//                    transfer.retract();
//                    transfer.setTargetDeg(transfer.wrap360(-20),sec);
//                    intPow=-1;
//                    shooterOn=false;
//                    setPathState(8);
//                }
//                break;



            case 8:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    /* Score Sample */
//                    transfer.setTargetDeg(240, opmodeTimer.getElapsedTimeSeconds());
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
//                    intake.setPower(-1);
                    transfer.retract();
                    transfer.retract();
                    transfer.setTargetDeg(transfer.wrap360(-20),sec);
                    follower.followPath(intakeSet4, true);
                    setPathState(9);
                }
                break;
            case 9:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > .75) {
                    /* Grab Sample */
//                    transfer.setTargetDeg(30,opmodeTimer.getElapsedTimeSeconds());
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    turTarg=97;
                    //turTarg=55;
                    transfer.retract();
                    shooterOn=true;
                    follower.followPath(scoreSet4, true);
                    setPathState(1000);
                }
                break;

            case 1000:
                if(pathTimer.getElapsedTimeSeconds()>2){
//                    intake.setPower(0);
//                    intPow=1;
                    transfer.spinToScore(sec);
                    setPathState(1001);

                }
                break;
            case 1001:
                if (transfer.atTarget()){
//                    intake.setPower(0);
                    transfer.score();
                    setPathState(1002);
                }
                else if (pathTimer.getElapsedTimeSeconds()>1) {
                    intPow=1;
                }
                break;
            case 1002:
                if(pathTimer.getElapsedTimeSeconds()>1){
                    transfer.setTargetDeg(transfer.wrap360(transfer.getPositionDeg() + 45), sec);
                    setPathState(1003);
                }
                break;
            case 1003:
                if(!follower.isBusy()&&shooter.atTarget()&& transfer.atTarget()&&turret.atTarget()){
                    setPathState(1004);
                }
                else if (pathTimer.getElapsedTimeSeconds()>1) {
                    intPow=1;
                }
                break;
            case 1004:
                if(pathTimer.getElapsedTimeSeconds()>.25){
                    transfer.startTransfer(dist);
                    setPathState(1005);
                }
                break;
            case 1005:
                if(pathTimer.getElapsedTimeSeconds()>1){
                    transfer.endTransfer();
                    transfer.setAuto();
                    transfer.retract();
                    transfer.setTargetDeg(transfer.wrap360(-20),sec);
                    intPow=-1;
                    shooterOn=false;
                    setPathState(10);
                }
                break;

            case 10:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0) {
                    /* Score Sample */
//                    transfer.setTargetDeg(240, opmodeTimer.getElapsedTimeSeconds());
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
//                    intake.setPower(-1);
                    transfer.retract();
                    transfer.retract();
                    transfer.setTargetDeg(transfer.wrap360(-20),sec);
                    follower.followPath(intakeSet5, true);
                    setPathState(11);
                }
                break;
            case 11:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1.5) {
                    /* Score Sample */
//                    transfer.setTargetDeg(240, opmodeTimer.getElapsedTimeSeconds());
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */

                    turTarg=49;
                    transfer.retract();
                    shooterOn=true;
                    follower.followPath(scoreSet5, true);
                    setPathState(1200);
                }
                break;
            case 1200:
                if(pathTimer.getElapsedTimeSeconds()>1.2){
//                    intake.setPower(0);
                    transfer.spinToScore(sec);
                    setPathState(1201);

                }
                break;
            case 1201:
                if (transfer.atTarget()){
//                    intake.setPower(0);
                    transfer.score();
                    setPathState(1202);
                }
                else if (pathTimer.getElapsedTimeSeconds()>1) {
                    intPow=1;
                }
                break;
            case 1202:
                if(pathTimer.getElapsedTimeSeconds()>1){
                    transfer.setTargetDeg(transfer.wrap360(transfer.getPositionDeg() + 45), sec);
                    setPathState(1203);
                }
                break;
            case 1203:
                if(!follower.isBusy()&&shooter.atTarget()&& transfer.atTarget()&&turret.atTarget()){
                    setPathState(1204);
                }
                else if (pathTimer.getElapsedTimeSeconds()>1) {
                    intPow=1;
                }
                break;
            case 1204:
                if(pathTimer.getElapsedTimeSeconds()>.75){
                    transfer.startTransfer(dist);
                    setPathState(1205);
                }
                break;
            case 1205:
                if(pathTimer.getElapsedTimeSeconds()>1){
                    transfer.endTransfer();
                    transfer.setAuto();
                    transfer.retract();
                    transfer.setTargetDeg(transfer.wrap360(-20),sec);
                    shooterOn=false;
                    setPathState(12);
                }
                break;
            case 12:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2) {
                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
                    intPow=0;
                    setPathState(-1);
                }
                break;
        }
    }


    /**
     * These change the states of the paths and actions. It will also reset the timers of the individual switches
     **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
    Turret turret;
    @Override
    public void loop() {
        double sec = opmodeTimer.getElapsedTimeSeconds();

        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        double goalDistance = Math.hypot(goalPose.getX() - follower.getPose().getX(), goalPose.getY() - follower.getPose().getY());
        goalPose=new Pose(goalX,goalY).mirror();
        autonomousPathUpdate(sec,goalDistance);
        if(shooterOn){
            shooter.forDistance(goalDistance);
        }
        else{
            shooter.setTarget(0);
        }
//        shootReady(sec,goalDistance);
//        shoot(goalDistance);

//        if(shootState!=-1||readyState!=-1){
//            shooter.forDistance(goalDistance);
//        }
//        else{
////            shooter.setTarget(0);
            shooter.forDistanceHood(goalDistance);
//        }
        turret.facePoint(goalPose,follower.getPose(),goalDistance);
//        turret.setYaw(turTarg);


        intake.setPower(intPow);
//
        turret.update();
        transfer.update(sec);
        shooter.update();


        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("shooter atTarget: ",shooter.atTarget());
        telemetry.addData("transfer atTarget: ",transfer.atTarget());
        telemetry.addData("turret atTarget: ",turret.atTarget());

        telemetry.addData("targetTranfer: ",transfer.getTargetDeg());
        telemetry.addData("posTranfer: ",transfer.getPositionDeg());


        telemetry.addData("shooter tar: ",shooter.getTarget());
        telemetry.addData("shooter velo: ",shooter.getVelocity());

        telemetry.addData("runtime:",opmodeTimer.getElapsedTimeSeconds());
        telemetry.update();
    }


    /**
     * This method is called once at the init of the OpMode.
     **/
    @Override
    public void init() {
        turret=new Turret(hardwareMap);
        initAprilTag(hardwareMap);
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        readyStateTimer=new ElapsedTime();
        shootStateTimer=new ElapsedTime();
        readyStateTimer.reset();
        shootStateTimer.reset();
        intake= hardwareMap.dcMotor.get("int");
        shooter= new Shooter(hardwareMap);
        transfer= new Transfer(hardwareMap);
        transfer.setAuto();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

    }

    /**
     * This method is called continuously after Init while waiting for "play".
     **/
    @Override
    public void init_loop() {
        transfer.score();
    }

    /**
     * This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system
     **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /**
     * We do not use this because everything should automatically disable
     **/
    @Override
    public void stop() {
    }
}