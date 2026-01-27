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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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
@TeleOp(name = "Blue15Ball", group = "Examples")
public class Blue15BallClose extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private String motif = "PGP";

    public static double goalX=-8,goalY=132;


    public static Pose goalPose = new Pose(goalX, goalY);

    private final Pose startPose = new Pose(23.416, 125.26377760519193, Math.toRadians(140.8));
    private final Pose shoot1Pose = new Pose(63, 75, Math.toRadians(180));
    private final Pose intake1Pose = new Pose(22, 59, Math.toRadians(180));
    private final Pose intake1PoseControl = new Pose(50, 60);

    private final Pose gatePose = new Pose(18, 71, Math.toRadians(180));

    private final Pose shoot2Pose = new Pose(63, 75, Math.toRadians(180));//old x: 50, y: 83.4

//    private final Pose intake2Pose = new Pose(14, 64, Math.toRadians(157));
//    private final Pose intake2PoseControl = new Pose(41, 31.5);


    private final Pose shoot3Pose = new Pose(63, 75, Math.toRadians(180));// old x: 70, y: 74
    private final Pose intake3Pose = new Pose(23, 36, Math.toRadians(180));

    private final Pose intake3PoseControl = new Pose(52, 37);

    private final Pose shoot4Pose = new Pose(61, 75, Math.toRadians(180));// old x: 70, y: 74


    private final Pose intake4Pose = new Pose(24, 84, Math.toRadians(180));
    //    private final Pose intake2PoseControl = new Pose(41, 31.5);
    private final Pose shoot5Pose = new Pose(63, 75, Math.toRadians(180));// old x: 70, y: 74
//    private final Pose leavePose = new Pose(45.4,66.7,Math.toRadians(180));

    private int pathState;
    private Path scorePreload;
    private PathChain intakeSet1, scoreSet1,hitGate, intakeSet2, scoreSet2, intakeSet3, scoreSet3,intakeSet4,scoreSet4;

    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(startPose, shoot1Pose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), shoot1Pose.getHeading());



    /* Here is an example for Constant Interpolation
    scorePreload.setConstantInterpolation(startPose.getHeading()); */

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        intakeSet1 = follower.pathBuilder()
                .addPath(new BezierCurve(shoot1Pose, intake1PoseControl, intake1Pose))
                .setTangentHeadingInterpolation()
                .addParametricCallback(0,()->follower.setMaxPower(1))
                .build();

        hitGate = follower.pathBuilder()
                .addPath(new BezierLine(intake1Pose, gatePose))
                .setLinearHeadingInterpolation(intake1Pose.getHeading(), gatePose.getHeading())
                .build();


        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scoreSet1 = follower.pathBuilder()
                .addPath(new BezierLine(intake1Pose, shoot2Pose))
                .setLinearHeadingInterpolation(intake1Pose.getHeading(),shoot2Pose.getHeading())
                .setVelocityConstraint(.05)
                .setTimeoutConstraint(200)
                .addParametricCallback(0.0,()->follower.setMaxPower(1))
                .addParametricCallback(.8,()->follower.setMaxPower(.3))
//                .setReversed()
                .build();

        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
//        intakeSet2 = follower.pathBuilder()
//                .addPath(new BezierLine(shoot2Pose, intake2Pose))
//                .setLinearHeadingInterpolation(shoot2Pose.getHeading(),intake2Pose.getHeading())
//                .build();

        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
//        scoreSet2 = follower.pathBuilder()
//                .addPath(new BezierLine(intake2Pose, shoot3Pose))
//                .setLinearHeadingInterpolation(intake2Pose.getHeading(),shoot3Pose.getHeading())
//                .setVelocityConstraint(.05)
//                .setTimeoutConstraint(200)
////                .setReversed()
//                .build();

        intakeSet3 = follower.pathBuilder()
                .addPath(new BezierCurve(shoot2Pose,intake3PoseControl, intake3Pose))
                .setTangentHeadingInterpolation()
                .addParametricCallback(0.0,()->follower.setMaxPower(1))
                .build();

        scoreSet3 = follower.pathBuilder()
                .addPath(new BezierLine(intake3Pose, shoot4Pose))
                .setHeadingInterpolation(HeadingInterpolator.piecewise(
                        new HeadingInterpolator.PiecewiseNode(
                                0,
                                .7,
                                HeadingInterpolator.tangent.reverse()

                        ),
                        new HeadingInterpolator.PiecewiseNode(
                                .7,
                                1,
                                HeadingInterpolator.linear(Math.toRadians(234), Math.toRadians(180))

                        )
                ))
//                .setTangentHeadingInterpolation()
//                .setReversed()
                .addParametricCallback(0.0,()->follower.setMaxPower(1))
                .addParametricCallback(.8,()->follower.setMaxPower(.3))

                .build();

        intakeSet4 = follower.pathBuilder()
                .addPath(new BezierLine(shoot4Pose,intake4Pose))
                .setTangentHeadingInterpolation()
                .addParametricCallback(0.0,()->follower.setMaxPower(1))
                .build();

        scoreSet4 = follower.pathBuilder()
                .addPath(new BezierLine(intake4Pose,shoot5Pose))
                .setTangentHeadingInterpolation()
                .setReversed()
                .addParametricCallback(0.0,()->follower.setMaxPower(1))
                .addParametricCallback(.8,()->follower.setMaxPower(.3))
                .build();

        /* This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
//        leave = follower.pathBuilder()
//                .addPath(new BezierLine(shoot3Pose, leavePose))
//                .setLinearHeadingInterpolation(shoot3Pose.getHeading(), leavePose.getHeading())
//                .build();


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
                turTarg=120;
                transfer.retract();
                follower.followPath(scorePreload);
                shooterOn=true;
                setPathState(100);
                break;
            case 100:
                if(pathTimer.getElapsedTimeSeconds()>1){
                    transfer.spinToScore(sec);
                    setPathState(101);

                }
                break;
            case 101:
                if (transfer.atTarget()){
                    transfer.score();
                    setPathState(102);
                }
                break;
            case 102:
                if(pathTimer.getElapsedTimeSeconds()>1){
                    transfer.setTargetDeg(transfer.wrap360(transfer.getPositionDeg() + 45), sec);
                    setPathState(103);
                }
                break;
            case 103:
                if(!follower.isBusy()&&shooter.atTarget()&& transfer.atTarget()){
                    setPathState(104);
                }
                break;
            case 104:
                if(pathTimer.getElapsedTimeSeconds()>.75){
                    transfer.startTransfer(.8,true);
                    setPathState(105);
                }
                break;
            case 105:
                if(pathTimer.getElapsedTimeSeconds()>2){
                    transfer.endTransfer();
                    transfer.setAuto();
                    transfer.retract();
                    transfer.setTargetDeg(transfer.wrap360(-25),sec);
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
                    transfer.setTargetDeg(transfer.wrap360(-25),sec);
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
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1.5) {
                    /* Grab Sample */
//                    transfer.setTargetDeg(30,opmodeTimer.getElapsedTimeSeconds());
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
//                    intake.setPower(0);
                    turTarg=120;
                    transfer.retract();
                    shooterOn=true;
                    follower.followPath(scoreSet1, true);
                    setPathState(300);
                }
                break;

            case 300:
                if(pathTimer.getElapsedTimeSeconds()>2){
//                    intake.setPower(0);
                    transfer.spinToScore(sec);
                    setPathState(301);

                }
                break;
            case 301:
                if (transfer.atTarget()){
                    transfer.score();
                    setPathState(302);
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
                break;
            case 304:
                if(pathTimer.getElapsedTimeSeconds()>.75){
                    transfer.startTransfer(.8,true);
                    setPathState(305);
                }
                break;
            case 305:
                if(pathTimer.getElapsedTimeSeconds()>2){
                    transfer.endTransfer();
                    transfer.setAuto();
                    transfer.retract();
                    transfer.setTargetDeg(transfer.wrap360(-25),sec);
                    shooterOn=false;
                    setPathState(5);
                }
                break;


//            case 3:
//                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
//                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2) {
//                    /* Score Sample */
////                    transfer.setTargetDeg(240,opmodeTimer.getElapsedTimeSeconds());
//
//                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
//
//                    intake.setPower(-1);
//                    transfer.retract();
//                    transfer.retract();
//                    transfer.setTargetDeg(transfer.wrap360(-25),sec);
//                    follower.followPath(intakeSet2, true);
//
//                    setPathState(-4);
//                }
//                break;
//            case 4:
//                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
//                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 6) {
//                    /* Grab Sample */
////                    transfer.setTargetDeg(30,opmodeTimer.getElapsedTimeSeconds());
//                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
//                    follower.followPath(scoreSet2, true);
//                    setPathState(5);
//                }
//                break;


            case 5:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > .75) {
                    /* Score Sample */
//                    transfer.setTargetDeg(240, opmodeTimer.getElapsedTimeSeconds());
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
//                    intake.setPower(-1);
                    transfer.retract();
                    transfer.retract();
                    transfer.setTargetDeg(transfer.wrap360(-25),sec);
                    follower.followPath(intakeSet3, true);
                    setPathState(6);
                }
                break;
            case 6:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > .75) {
                    /* Grab Sample */
//                    transfer.setTargetDeg(30,opmodeTimer.getElapsedTimeSeconds());
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    turTarg=120;
                    transfer.retract();
                    shooterOn=true;
                    follower.followPath(scoreSet3, true);
                    setPathState(700);
                }
                break;

            case 700:
                if(pathTimer.getElapsedTimeSeconds()>2){
//                    intake.setPower(0);
                    transfer.spinToScore(sec);
                    setPathState(701);

                }
                break;
            case 701:
                if (transfer.atTarget()){
//                    intake.setPower(0);
                    transfer.score();
                    setPathState(702);
                }
                break;
            case 702:
                if(pathTimer.getElapsedTimeSeconds()>1){
                    transfer.setTargetDeg(transfer.wrap360(transfer.getPositionDeg() + 45), sec);
                    setPathState(703);
                }
                break;
            case 703:
                if(!follower.isBusy()&&shooter.atTarget()&& transfer.atTarget()){
                    setPathState(704);
                }
                break;
            case 704:
                if(pathTimer.getElapsedTimeSeconds()>.75){
                    transfer.startTransfer(.8,true);
                    setPathState(705);
                }
                break;
            case 705:
                if(pathTimer.getElapsedTimeSeconds()>2){
                    transfer.endTransfer();
                    transfer.setAuto();
                    transfer.retract();
                    transfer.setTargetDeg(transfer.wrap360(-25),sec);
                    shooterOn=false;
                    setPathState(7);
                }
                break;

            case 7:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > .5) {
                    /* Score Sample */
//                    transfer.setTargetDeg(240, opmodeTimer.getElapsedTimeSeconds());
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
//                    intake.setPower(-1);
                    transfer.retract();
                    transfer.retract();
                    transfer.setTargetDeg(transfer.wrap360(-25),sec);
                    follower.followPath(intakeSet4, true);
                    setPathState(8);
                }
                break;
            case 8:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1.5) {
                    /* Score Sample */
//                    transfer.setTargetDeg(240, opmodeTimer.getElapsedTimeSeconds());
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    turTarg=120;
                    transfer.retract();
                    shooterOn=true;
                    follower.followPath(scoreSet4, true);
                    setPathState(900);
                }
                break;
            case 900:
                if(pathTimer.getElapsedTimeSeconds()>.5){
//                    intake.setPower(0);
                    transfer.spinToScore(sec);
                    setPathState(901);

                }
                break;
            case 901:
                if (transfer.atTarget()){
//                    intake.setPower(0);
                    transfer.score();
                    setPathState(902);
                }
                break;
            case 902:
                if(pathTimer.getElapsedTimeSeconds()>1){
                    transfer.setTargetDeg(transfer.wrap360(transfer.getPositionDeg() + 45), sec);
                    setPathState(903);
                }
                break;
            case 903:
                if(!follower.isBusy()&&shooter.atTarget()&& transfer.atTarget()){
                    setPathState(904);
                }
                break;
            case 904:
                if(pathTimer.getElapsedTimeSeconds()>.75){
                    transfer.startTransfer(.8,true);
                    setPathState(905);
                }
                break;
            case 905:
                if(pathTimer.getElapsedTimeSeconds()>2){
                    transfer.endTransfer();
                    transfer.setAuto();
                    transfer.retract();
                    transfer.setTargetDeg(transfer.wrap360(-25),sec);
                    shooterOn=false;
                    setPathState(9);
                }
                break;
            case 9:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2) {
                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
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
        goalPose=new Pose(goalX,goalY);
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
//            shooter.forDistanceHood(goalDistance);
//        }
//        turret.facePoint(goalPose,follower.getPose());
        turret.setTargetDeg(turTarg);

        intake.setPower(-1);
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
        transfer.retract();
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