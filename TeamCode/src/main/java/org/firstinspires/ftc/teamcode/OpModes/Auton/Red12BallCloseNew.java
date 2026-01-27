package org.firstinspires.ftc.teamcode.OpModes.Auton; // make sure this aligns with class location

import android.util.Size;

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

@Autonomous(name = "Red12Ball", group = "Examples")
public class Red12BallCloseNew extends OpMode {
    private double scoreAngle = 155;

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private String motif = "PGP";


    public static Pose goalPose = new Pose(1, 158).mirror();

    private final Pose startPose = new Pose(21.63, 122.23, Math.toRadians(140)).mirror();
    private final Pose shoot1Pose = new Pose(53, 110, Math.toRadians(180)).mirror();
    private final Pose intake1Pose = new Pose(22, 59, Math.toRadians(180)).mirror();
    private final Pose intake1PoseControl = new Pose(50, 60).mirror();

    private final Pose gatePose = new Pose(18, 71, Math.toRadians(180)).mirror();

    private final Pose shoot2Pose = new Pose(66, 74, Math.toRadians(180)).mirror();//old x: 50, y: 83.4

    private final Pose intake2Pose = new Pose(14, 35, Math.toRadians(180)).mirror();
    private final Pose intake2PoseControl = new Pose(41, 31.5).mirror();
    private final Pose shoot3Pose = new Pose(59, 85, Math.toRadians(180)).mirror();// old x: 70, y: 74
    private final Pose intake3Pose = new Pose(18, 81.5, Math.toRadians(180)).mirror();
    private final Pose shoot4Pose = new Pose(50, 120, Math.toRadians(180)).mirror();// old x: 70, y: 74

//    private final Pose leavePose = new Pose(45.4,66.7,Math.toRadians(180));

    private int pathState;
    private Path scorePreload;
    private PathChain intakeSet1, hitGate, scoreSet1, intakeSet2, scoreSet2, intakeSet3, scoreSet3;

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
                .build();

        hitGate = follower.pathBuilder()
                .addPath(new BezierLine(intake1Pose, gatePose))
                .setLinearHeadingInterpolation(intake1Pose.getHeading(), gatePose.getHeading())
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scoreSet1 = follower.pathBuilder()
                .addPath(new BezierLine(gatePose, shoot2Pose))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        intakeSet2 = follower.pathBuilder()
                .addPath(new BezierCurve(shoot2Pose, intake2PoseControl, intake2Pose))
                .setTangentHeadingInterpolation()
                .build();

        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scoreSet2 = follower.pathBuilder()
                .addPath(new BezierLine(intake2Pose, shoot3Pose))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        intakeSet3 = follower.pathBuilder()
                .addPath(new BezierLine(shoot3Pose, intake3Pose))
                .setTangentHeadingInterpolation()
                .build();

        scoreSet3 = follower.pathBuilder()
                .addPath(new BezierLine(intake3Pose, shoot4Pose))
                .setTangentHeadingInterpolation()
                .setReversed()
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

    private int readyState = -1;
    private int shootState=-1;

    Shooter shooter;
    Transfer transfer;

    DcMotor intake;

    ElapsedTime readyStateTimer,shootStateTimer;

    public void shootReady(double sec, double dist) {
        switch (readyState) {
            case -1:
                break;
            case 0:
                shooter.forDistance(dist);
                transfer.retract();
                transfer.scan(sec);
                readyStateTimer.reset();
                readyState = 101;
                break;
            case 101:
                shooter.forDistance(dist);
                if (transfer.atTarget()) {
                    transfer.scan(sec);
                    readyStateTimer.reset();
                    readyState = 1;
                }
                break;
            case 1:
                shooter.forDistance(dist);
                if (readyStateTimer.milliseconds() > 200) {
                    transfer.setTargetDeg(transfer.spin(motif), sec);
                    readyState = 2;
                }
                break;
            case 2:
                shooter.forDistance(dist);
                if (transfer.atTarget()) {
                    transfer.score();
                    readyState = 3;
                    readyStateTimer.reset();
                }
                break;
            case 3:
//                transfer.setManual();
//                transfer.manualPower = -.07;
                shooter.forDistance(dist);
                if (readyStateTimer.milliseconds() > 200) {
                    transfer.setTargetDeg(transfer.wrap360(transfer.getPositionDeg() - 45), sec);
                    readyStateTimer.reset();
                    readyState = 4;
                }
                break;
            case 4:
                shooter.forDistance(dist);
                if (readyStateTimer.milliseconds() > 00&& shooter.atTarget()) {
                    readyState = -1;
                }
                break;

        }
    }

    public void shoot(double dist){
        switch (shootState){
            case -1:
                break;
            case 0:
                shooter.forDistance(dist);
                transfer.startTransfer(dist);
                shootState=1;
                shootStateTimer.reset();
                break;
            case 1:
                if(shootStateTimer.milliseconds()>1500) {
                    transfer.endTransfer();
                    transfer.setAuto();
                    transfer.retract();
                    shooter.setTarget(0);
                    shootState = -1;
                }
                break;

        }
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                readyState=0;
                setPathState(100);
                break;
            case 100:
//                setRobotPoseFromCamera();
                if(!follower.isBusy() && readyState==-1&& shooter.atTarget()&&pathTimer.getElapsedTimeSeconds()>3){
                    shootState=0;
                    setPathState(-1);
                }
                break;
            case 1:

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 3&&shootState==-1) {
                    /* Grab Sample */
//                    transfer.setTargetDeg(30,opmodeTimer.getElapsedTimeSeconds());
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    setRobotPoseFromCamera();
                    readyState=-1;
                    shootState=-1;
                    shooter.setTarget(0);
                    transfer.setAuto();
                    transfer.setTargetDeg(30,opmodeTimer.getElapsedTimeSeconds());
                    intake.setPower(-1);
                    follower.followPath(intakeSet1, true);
                    setPathState(2);
                }
                break;
            case 2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1) {
                    /* Grab Sample */
//                    transfer.setTargetDeg(30,opmodeTimer.getElapsedTimeSeconds());
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    readyState=0;
                    intake.setPower(0);
                    follower.followPath(hitGate, true);
                    setPathState(3);
                }
                break;
            case 3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1) {
                    /* Score Sample */
//                    transfer.setTargetDeg(240,opmodeTimer.getElapsedTimeSeconds());

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */


                    follower.followPath(scoreSet1, true);

                    setPathState(300);
                }
                break;
            case 300:
//                setRobotPoseFromCamera();
                if(!follower.isBusy() && readyState==-1&&shooter.atTarget()&&pathTimer.getElapsedTimeSeconds()>2){
                    shootState=0;
                    setPathState(4);
                }
                break;

            case 4:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1&&shootState==-1) {
                    /* Grab Sample */
//                    transfer.setTargetDeg(30,opmodeTimer.getElapsedTimeSeconds());
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    readyState=-1;
                    shootState=-1;
                    shooter.setTarget(0);
                    transfer.setAuto();
                    transfer.setTargetDeg(30,opmodeTimer.getElapsedTimeSeconds());
                    intake.setPower(-1);
                    follower.followPath(intakeSet2, true);
                    setPathState(5);
                }
                break;
            case 5:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1) {
                    /* Score Sample */
//                    transfer.setTargetDeg(240, opmodeTimer.getElapsedTimeSeconds());
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    readyState=0;
                    intake.setPower(0);
                    follower.followPath(scoreSet2, true);
                    setPathState(500);
                }
                break;
            case 500:
//                setRobotPoseFromCamera();
                if(!follower.isBusy() && readyState==-1&&shooter.atTarget()&&pathTimer.getElapsedTimeSeconds()>2){
                    shootState=0;
                    setPathState(6);
                }
            case 6:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1&&shootState==-1) {
                    /* Grab Sample */
//                    transfer.setTargetDeg(30,opmodeTimer.getElapsedTimeSeconds());
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    readyState=-1;
                    shootState=-1;
                    shooter.setTarget(0);
                    transfer.setAuto();
                    transfer.setTargetDeg(30,opmodeTimer.getElapsedTimeSeconds());
                    intake.setPower(-1);
                    follower.followPath(intakeSet3, true);
                    setPathState(7);
                }
                break;
            case 7:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1) {
                    /* Score Sample */
//                    transfer.setTargetDeg(240, opmodeTimer.getElapsedTimeSeconds());
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    readyState=0;
                    intake.setPower(0);
                    follower.followPath(scoreSet3, true);
                    setPathState(700);
                }
                break;
            case 700:
//                setRobotPoseFromCamera();
                if(!follower.isBusy() && readyState==-1&& shooter.atTarget()&&pathTimer.getElapsedTimeSeconds()>2){
                    shootState=0;
                    setPathState(800);
                }

            case 8:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1) {
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
        autonomousPathUpdate();
        double goalDistance = Math.hypot(goalPose.getX() - follower.getPose().getX(), goalPose.getY() - follower.getPose().getY());
        shootReady(sec,goalDistance);
        shoot(goalDistance);

        if(shootState!=-1||readyState!=-1){
            shooter.forDistance(goalDistance);
        }
        else{
//            shooter.setTarget(0);
            shooter.forDistanceHood(goalDistance);
        }
        turret.facePoint(goalPose,follower.getPose());

        turret.update();
        transfer.update(sec);
        shooter.update();


        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
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