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

@Config
@Autonomous(name = "BlueGateIntake", group = "Examples")
public class BlueGateIntake extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private String motif = "PGP";

    public static double goalX=-8,goalY=132;


    public static Pose goalPose = new Pose(goalX, goalY);

    private final Pose startPose = new Pose(72, 72, Math.toRadians(180));
    private final Pose shoot1Pose = new Pose(63, 75, Math.toRadians(180));
    private final Pose intake1Pose = new Pose(22, 59, Math.toRadians(180));
    private final Pose intake1PoseControl = new Pose(50, 60);

//    private final Pose gatePose = new Pose(18, 71, Math.toRadians(180));

    private final Pose shoot2Pose = new Pose(63, 75, Math.toRadians(180));//old x: 50, y: 83.4

    private final Pose intake2PrimePose = new Pose(18, 66, Math.toRadians(175));
    private final Pose intake2Pose = new Pose(10, 50, Math.toRadians(100));

    private final Pose intake2ControlPose = new Pose(19,56);


//    private final Pose intake2PoseControl = new Pose(41, 31.5);


    private final Pose shoot3Pose = new Pose(63, 75, Math.toRadians(180));// old x: 70, y: 74

    private final Pose intake3PrimePose = new Pose(14, 63, Math.toRadians(175));
    private final Pose intake3Pose = new Pose(10, 52, Math.toRadians(100));

    private final Pose intake3ControlPose = new Pose(15.797147846332944,56.65133876600699);

    private final Pose shoot4Pose = new Pose(63, 75, Math.toRadians(180));// old x: 70, y: 74

    private final Pose intake4Pose = new Pose(23, 36, Math.toRadians(180));

    private final Pose intake4PoseControl = new Pose(52, 37);

    private final Pose shoot5Pose = new Pose(61, 84, Math.toRadians(180));// old x: 70, y: 74


    private final Pose intake5Pose = new Pose(20, 84, Math.toRadians(180));
    //    private final Pose intake2PoseControl = new Pose(41, 31.5);
    private final Pose shoot6Pose = new Pose(63, 75, Math.toRadians(180));// old x: 70, y: 74
    private final Pose leavePose = new Pose(45.4,66.7,Math.toRadians(180));

    private int pathState;
    private Path scorePreload;
    private PathChain intakeSet1, scoreSet1,intakeSet2,intakePrimeSet2, scoreSet2, intakeSet3,intakePrimeSet3, scoreSet3,intakeSet4,scoreSet4,intakeSet5,scoreSet5,leave;
    private double intPow;

    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(startPose, shoot1Pose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), shoot1Pose.getHeading());



    /* Here is an example for Constant Interpolation
    scorePreload.setConstantInterpolation(startPose.getHeading()); */

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */

        intakePrimeSet2 = follower.pathBuilder()
                .addPath(new BezierLine(startPose,intake2PrimePose))
                .setLinearHeadingInterpolation(startPose.getHeading(),intake2PrimePose.getHeading())
                .addParametricCallback(0.0,()->follower.setMaxPower(1))
                .addParametricCallback(.8,()->follower.setMaxPower(.5))
                .build();

        intakeSet2 = follower.pathBuilder()
                .addPath(new BezierCurve(intake2PrimePose,intake2ControlPose,intake2Pose))
                .setLinearHeadingInterpolation(intake2PrimePose.getHeading(),intake2Pose.getHeading())
                .addParametricCallback(0.0,()->follower.setMaxPower(1))
                .build();
        scoreSet2 = follower.pathBuilder()
                .addPath(new BezierLine(intake2Pose,shoot3Pose))
                .setLinearHeadingInterpolation(intake2Pose.getHeading(),shoot3Pose.getHeading())
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
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0) {
                    /* Score Sample */
//                    transfer.setTargetDeg(240,opmodeTimer.getElapsedTimeSeconds());

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */

                    intPow=0;
                    transfer.retract();
                    transfer.retract();
                    transfer.setTargetDeg(transfer.wrap360(-25),sec);
                    follower.followPath(intakePrimeSet2, true);

                    setPathState(4);
                }
                break;
            case 4:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if (!follower.isBusy()&&pathTimer.getElapsedTimeSeconds() > .5) {
                    /* Grab Sample */
//                    transfer.setTargetDeg(30,opmodeTimer.getElapsedTimeSeconds());
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    intPow=-1;
                    follower.followPath(intakeSet2, true);
                    setPathState(5);
                }
                break;
            case 5:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2) {
                    /* Grab Sample */
//                    transfer.setTargetDeg(30,opmodeTimer.getElapsedTimeSeconds());
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
//                    intPow=1;
                    follower.followPath(scoreSet2, true);
                    setPathState(6);
                }
                break;
            case 6:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (follower.getCurrentTValue()>.3) {
                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
                    intPow=1;
                    setPathState(7);
                }
                break;
            case 7:
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