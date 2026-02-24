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
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.lynx.LynxModule;
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
@Autonomous(name = "Blue6BallFarRP", group = "Examples")
public class Blue6BallFarRP extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private String motif = "PGP";

    public static double goalX=5,goalY=141.4;


    public static Pose goalPose = new Pose(goalX, goalY);

    private final Pose startPose = new Pose(57, 6.56, Math.toRadians(90));
    private final Pose shoot1Pose = new Pose(60, 18, Math.toRadians(90));
    private final Pose intake1PrimePose = new Pose(6, 5, Math.toRadians(190));

    private final Pose intake1StrafePose = new Pose(6, 8, Math.toRadians(180));

//    private final Pose intake1Prime2Pose = new Pose(9, 24, Math.toRadians(265));
//    private final Pose intake1Pose = new Pose(9, 6, Math.toRadians(265));
//    private final Pose intake1ControlPose = new Pose(8.888242142025614, 21.616996507566906);
//    private final Pose gatePose = new Pose(18, 71, Math.toRadians(180));

    private final Pose shoot2Pose = new Pose(60, 18, Math.toRadians(200));
    private final Pose intake2PrimePose = new Pose(9, 5, Math.toRadians(190));

    private final Pose intake2StrafePose = new Pose(9, 8, Math.toRadians(180));

    private final Pose shoot3Pose = new Pose(60, 18, Math.toRadians(200));


    private final Pose leavePose = new Pose(40, 12, Math.toRadians(270));



    private int pathState;
    private Path scorePreload;
    private PathChain intakePrimeSet1,intakeStrafe,scoreSet1,intakePrimeSet2,intake2Strafe,scoreSet2,leave;
    private double intPow;
    private double shootTar;

    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(startPose, shoot1Pose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), shoot1Pose.getHeading());



    /* Here is an example for Constant Interpolation
    scorePreload.setConstantInterpolation(startPose.getHeading()); */

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        intakePrimeSet1 = follower.pathBuilder()
                .addPath(new BezierLine(shoot1Pose,intake1PrimePose))
                .setLinearHeadingInterpolation(shoot1Pose.getHeading(),intake1PrimePose.getHeading())
                .addParametricCallback(0,()->follower.setMaxPower(1))
                .addParametricCallback(0.7,()->follower.setMaxPower(.3))

                .build();
        intakeStrafe = follower.pathBuilder()
                .addPath(new BezierLine(intake1PrimePose,intake1StrafePose))
                .setLinearHeadingInterpolation(intake1PrimePose.getHeading(),intake1StrafePose.getHeading())
                .addParametricCallback(0,()->follower.setMaxPower(1.25))
//                .addParametricCallback(0.7,()->follower.setMaxPower(.3))
                .build();

//        intakePrime2Set1 = follower.pathBuilder()
//                .addPath(new BezierLine(intake1PrimePose,intake1Prime2Pose))
//                .setLinearHeadingInterpolation(intake1PrimePose.getHeading(),intake1Prime2Pose.getHeading())
//                .addParametricCallback(0,()->follower.setMaxPower(1))
//                .build();
//
//
//        intakeSet1 = follower.pathBuilder()
//                .addPath(new BezierCurve(intake1Prime2Pose, intake1ControlPose, intake1Pose))
//                .setTangentHeadingInterpolation()
//                .addParametricCallback(0,()->follower.setMaxPower(.3))
//                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scoreSet1 = follower.pathBuilder()
                .addPath(new BezierLine(intake1StrafePose, shoot2Pose))
                .setLinearHeadingInterpolation(intake1StrafePose.getHeading(),shoot2Pose.getHeading())
                .setVelocityConstraint(.05)
                .setTimeoutConstraint(200)
                .addParametricCallback(0.0,()->follower.setMaxPower(1))
                .addParametricCallback(.7,()->follower.setMaxPower(.1))
//                .setReversed()
                .build();

        intakePrimeSet2 = follower.pathBuilder()
                .addPath(new BezierLine(shoot2Pose,intake2PrimePose))
                .setLinearHeadingInterpolation(shoot2Pose.getHeading(),intake2PrimePose.getHeading())
                .addParametricCallback(0,()->follower.setMaxPower(1))
                .addParametricCallback(0.7,()->follower.setMaxPower(.3))

                .build();
        intake2Strafe = follower.pathBuilder()
                .addPath(new BezierLine(intake2PrimePose,intake2StrafePose))
                .setLinearHeadingInterpolation(intake2PrimePose.getHeading(),intake2StrafePose.getHeading())
                .addParametricCallback(0,()->follower.setMaxPower(1))
//                .addParametricCallback(0.7,()->follower.setMaxPower(.3))
                .build();

        scoreSet2 = follower.pathBuilder()
                .addPath(new BezierLine(intake2StrafePose, shoot3Pose))
                .setLinearHeadingInterpolation(intake2StrafePose.getHeading(),shoot3Pose.getHeading())
                .setVelocityConstraint(.05)
                .setTimeoutConstraint(200)
                .addParametricCallback(0.0,()->follower.setMaxPower(1))
                .addParametricCallback(.7,()->follower.setMaxPower(.2))
//                .setReversed()
                .build();

        leave = follower.pathBuilder()
                .addPath(new BezierLine(shoot3Pose, leavePose))
                .setLinearHeadingInterpolation(shoot3Pose.getHeading(), leavePose.getHeading())
                .addParametricCallback(0.0,()->follower.setMaxPower(1))
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
                turTarg=-22;
                shootTar=3165;
                transfer.setManual();
                transfer.score();
                follower.followPath(scorePreload);
                shooterOn=true;
                setPathState(104);
                break;
            case 104:
                if(pathTimer.getElapsedTimeSeconds()>3&&!follower.isBusy()){
                    transfer.startTransfer(.6,true);
                    setPathState(105);
                }
                break;
            case 105:
                if(pathTimer.getElapsedTimeSeconds()>2){
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
                if (pathTimer.getElapsedTimeSeconds() > .75) {
                    /* Grab Sample */
//                    transfer.setTargetDeg(30,opmodeTimer.getElapsedTimeSeconds());
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    transfer.retract();
                    transfer.retract();
                    intPow=-1;
                    transfer.setTargetDeg(transfer.wrap360(-20),sec);
                    follower.followPath(intakePrimeSet1, true);
                    setPathState(12);
                }
                break;
            case 12:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2) {
                    /* Grab Sample */
//                    transfer.setTargetDeg(30,opmodeTimer.getElapsedTimeSeconds());
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
//                    intake.setPower(0);
                    follower.followPath(intakeStrafe, true);
                    setPathState(2);
                }
                break;
//            case 13:
//                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1) {
//                    /* Grab Sample */
////                    transfer.setTargetDeg(30,opmodeTimer.getElapsedTimeSeconds());
//                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
////                    intake.setPower(0);
//                    follower.followPath(intakeSet1, true);
//                    setPathState(2);
//                }
//                break;

            case 2:
//                intake.setPower(-1);

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1.5) {
                    /* Grab Sample */
//                    transfer.setTargetDeg(30,opmodeTimer.getElapsedTimeSeconds());
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
//                    intake.setPower(0);
                    turTarg=93;
                    shootTar=3180;
                    transfer.retract();
                    shooterOn=true;
                    follower.followPath(scoreSet1, true);
                    setPathState(300);
                }
                break;

            case 300:
                if(pathTimer.getElapsedTimeSeconds()>1.75){
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
                } else if (pathTimer.getElapsedTimeSeconds()>3) {
                    intPow=1;
                }
                break;
            case 302:
                if(pathTimer.getElapsedTimeSeconds()>1){
//                    intake.setPower(0);
                    transfer.setTargetDeg(transfer.wrap360(transfer.getPositionDeg() + 45), sec);
                    setPathState(303);
                }
                break;
            case 303:
                if(!follower.isBusy()&&shooter.atTarget()&& transfer.atTarget()){
                    setPathState(304);
                }
                else if (pathTimer.getElapsedTimeSeconds()>3) {
                    intPow=1;
                }
                break;
            case 304:
                if(pathTimer.getElapsedTimeSeconds()>.75){
                    transfer.startTransfer(.6,true);
                    setPathState(305);
                }
                break;
            case 305:
                if(pathTimer.getElapsedTimeSeconds()>2){
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
//                intake.setPower(-1);


                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if (pathTimer.getElapsedTimeSeconds() > .75) {
                    /* Grab Sample */
//                    transfer.setTargetDeg(30,opmodeTimer.getElapsedTimeSeconds());
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    transfer.retract();
                    transfer.retract();
                    intPow=-1;
                    transfer.setTargetDeg(transfer.wrap360(-20),sec);
                    follower.followPath(intakePrimeSet2, true);
                    setPathState(22);
                }
                break;
            case 22:
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 2.3) {
                    /* Grab Sample */
//                    transfer.setTargetDeg(30,opmodeTimer.getElapsedTimeSeconds());
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
//                    intake.setPower(0);
                    follower.followPath(intake2Strafe, true);
                    setPathState(21);
                }
                break;
//            case 13:
//                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1) {
//                    /* Grab Sample */
////                    transfer.setTargetDeg(30,opmodeTimer.getElapsedTimeSeconds());
//                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
////                    intake.setPower(0);
//                    follower.followPath(intakeSet1, true);
//                    setPathState(2);
//                }
//                break;

            case 21:
//                intake.setPower(-1);

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if ((!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1.5)||pathTimer.getElapsedTimeSeconds()>2.3) {
                    /* Grab Sample */
//                    transfer.setTargetDeg(30,opmodeTimer.getElapsedTimeSeconds());
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
//                    intake.setPower(0);
                    turTarg=93.5;
                    shootTar=3180;
                    transfer.retract();
                    shooterOn=true;
                    follower.followPath(scoreSet2, true);
                    setPathState(500);
                }
                break;

            case 500:
                if(pathTimer.getElapsedTimeSeconds()>2){
//                    intake.setPower(0);
//                    intPow=1;
                    transfer.spinToScore(sec);
                    setPathState(501);

                }
                if(follower.getCurrentTValue()>.06){
                    intPow=1;
                }
                break;
            case 501:
                if (transfer.atTarget()){
                    transfer.score();
                    setPathState(502);
                } else if (pathTimer.getElapsedTimeSeconds()>3) {
                    intPow=1;
                }
                break;
            case 502:
                if(pathTimer.getElapsedTimeSeconds()>1){
//                    intake.setPower(0);
                    transfer.setTargetDeg(transfer.wrap360(transfer.getPositionDeg() + 45), sec);
                    setPathState(503);
                }
                break;
            case 503:
                if(!follower.isBusy()&&shooter.atTarget()&& transfer.atTarget()){
                    setPathState(504);
                }
                else if (pathTimer.getElapsedTimeSeconds()>3) {
                    intPow=1;
                }
                break;
            case 504:
                if(pathTimer.getElapsedTimeSeconds()>.75){
                    transfer.startTransfer(.6,true);
                    setPathState(505);
                }
                break;
            case 505:
                if(pathTimer.getElapsedTimeSeconds()>2){
                    transfer.endTransfer();
                    transfer.setAuto();
                    transfer.retract();
                    transfer.setTargetDeg(transfer.wrap360(-20),sec);
                    intPow=-1;
                    shooterOn=false;
                    setPathState(9);
                }
                break;
            case 9:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2) {
                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
                    intPow=0;
                    follower.followPath(leave);
                    setPathState(10);
                }
                break;
            case 10:
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
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
        double sec = opmodeTimer.getElapsedTimeSeconds();

        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        double goalDistance = Math.hypot(goalPose.getX() - follower.getPose().getX(), goalPose.getY() - follower.getPose().getY());
        goalPose=new Pose(goalX,goalY);
        autonomousPathUpdate(sec,goalDistance);
        if(shooterOn){
            shooter.setTarget(shootTar);
        }
        else{
            shooter.setTarget(0);
        }
        shooter.setHood(90);
//        shootReady(sec,goalDistance);
//        shoot(goalDistance);

//        if(shootState!=-1||readyState!=-1){
//            shooter.forDistance(goalDistance);
//        }
//        else{
////            shooter.setTarget(0);
//            shooter.forDistanceHood(goalDistance);
//        }
//        turret.facePoint(goalPose,follower.getPose(),goalDistance);
        turret.setYaw(turTarg);


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

        telemetry.addData("targetTranfer: ",transfer.getTargetDeg());
        telemetry.addData("posTranfer: ",transfer.getPositionDeg());


        telemetry.addData("shooter tar: ",shooter.getTarget());
        telemetry.addData("shooter velo: ",shooter.getVelocity());

        telemetry.addData("runtime: ",opmodeTimer.getElapsedTimeSeconds());


        telemetry.update();
    }

    List<LynxModule> allHubs;
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
        allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
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