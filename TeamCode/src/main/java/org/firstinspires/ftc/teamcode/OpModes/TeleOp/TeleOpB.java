package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.InvertedFTCCoordinates;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.List;

@TeleOp
@Config
public class TeleOpB extends LinearOpMode {
    Follower follower;

    Turret turret;
    Transfer transfer;

    Shooter shooter;

    public static double rotateMult=1, driveMult=1;
    private int stateUnsorted=-1;
    private int stateSorted=-1;
    private String motif = "PGP";
    private int stateShoot=-1;

    private int intakeState=-1;

    ElapsedTime shootStateTimer,sortTimer,timer;

    public static Pose goalPose = new Pose(1,142);
    private DcMotor intake;

//    private double transferTar=0;

    private boolean readyToShoot;


    @Override
    public void runOpMode() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72,72,Math.toRadians(90)));
        follower.startTeleopDrive();
        follower.update();

        turret=new Turret(hardwareMap);

        transfer = new Transfer(hardwareMap);

        shooter=new Shooter(hardwareMap);

        intake = hardwareMap.dcMotor.get("int");

        readyToShoot=false;

        shootStateTimer=new ElapsedTime();
        sortTimer=new ElapsedTime();
        timer=new ElapsedTime();
        timer.reset();


        waitForStart();
        while (opModeIsActive()) {

            double sec=timer.seconds();



            if(gamepad1.yWasPressed()&&isSlowMode()){
                setSlowMode();
            } else if (gamepad1.yWasPressed()&&!isSlowMode()) {
                setFastMode();

            }
            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y*driveMult ,
                    -gamepad1.left_stick_x*driveMult ,
                    -gamepad1.right_stick_x*rotateMult , true);
            if(gamepad1.dpad_right){
                setRobotPoseFromCamera();
                turret.setTargetDeg(120);
            }
            else{
                turret.facePoint(goalPose,follower.getPose());
            }

            if(gamepad1.dpad_left){
                follower.setPose(new Pose(72,72,Math.toRadians(90)));
            }


            follower.update();

            double dist = Math.hypot(goalPose.getX()-follower.getPose().getX(),goalPose.getY()-follower.getPose().getY());




            if(!gamepad1.dpad_right){
                turret.facePoint(goalPose,follower.getPose());
            }

            if(gamepad1.bWasPressed()){
                intakeState=0;
                stateShoot=-1;
                stateUnsorted=-1;
                stateSorted=-1;
                transfer.setAuto();
                transfer.setTargetDeg(transfer.wrap360(-25),sec);
            }

            if(intakeState!=-1&&stateSorted==-1&&stateUnsorted==-1&& stateShoot ==-1){
                transfer.retract();
            }

            if(gamepad1.leftBumperWasPressed()&&stateSorted==-1&&!readyToShoot){
                readyToShoot=false;
                intakeState=-1;
                stateSorted=0;
                stateUnsorted=-1;
                stateShoot=-1;

            } else if (gamepad1.rightBumperWasPressed()&&stateUnsorted==-1&&readyToShoot) {
                readyToShoot=false;
                intakeState=-1;
                stateSorted=-1;
                stateUnsorted=0;
                stateShoot=-1;
            }

            if(gamepad1.aWasPressed()&&readyToShoot){
                intakeState=-1;
                stateUnsorted=-1;
                stateSorted=-1;
                stateShoot=0;
            }

            primeSortedBalls(sec);
            primeUnsortedBalls(sec);
            shootPrimedBalls(sec);


            if(stateUnsorted!=-1||stateSorted!=-1||stateShoot!=-1){
                shooter.forDistance(dist);
            }
            if(intakeState==0){

                shooter.setTarget(0);
            }

            intake.setPower(gamepad1.left_trigger-gamepad1.right_trigger);



            transfer.update(sec);
            shooter.update();
            turret.update();

            telemetry.addData("stateSorted: ",stateSorted);
            telemetry.addData("stateUnsorted: ",stateUnsorted);
            telemetry.addData("stateShoot: ",stateShoot);
            telemetry.addData("stateIntake: ",intakeState);

            telemetry.addData("follower pose x:",follower.getPose().getX());
            telemetry.addData("follower pose y:",follower.getPose().getY());
            telemetry.addData("follower pose h:",Math.toDegrees(follower.getPose().getHeading()));
            telemetry.addData("follower dist to goal:", Math.hypot(goalPose.getX()-follower.getPose().getX(),goalPose.getY()-follower.getPose().getY()));
            Pose robotPose = follower.getPose();
            Pose ballPose = new Pose(robotPose.getX()+Math.cos(robotPose.getHeading()), robotPose.getY()+Math.sin(robotPose.getHeading()));
            telemetry.addData("ball pose x:",ballPose.getX());
            telemetry.addData("ball pose y:",ballPose.getY());
            telemetry.addData("ball dist to goal:", Math.hypot(goalPose.getX()-ballPose.getX(),goalPose.getY()-ballPose.getY()));


            telemetry.addData("transfer auto",transfer.isAuto());
            telemetry.addData("targetTranfer: ",transfer.getTargetDeg());
            telemetry.addData("motionTarget Transfer: ",transfer.getMotionTargetDeg());
            telemetry.addData("posTranfer: ",transfer.getPositionDeg());
            telemetry.addData("transfer power: ",transfer.getPower());

            telemetry.addData("timer: ",timer.seconds());
            telemetry.update();



        }
    }

    private void primeUnsortedBalls(double sec) {
        switch (stateUnsorted){
            case -1:
                break;
            case 0:
                transfer.retract();
                sortTimer.reset();
                stateUnsorted=1;
                break;
            case 1:
                if(sortTimer.milliseconds()>500) {
                    transfer.spinToScore(sec);
                    stateUnsorted = 2;
                }
                break;
            case 2:
                if(transfer.atTarget()){
                    transfer.score();
                    sortTimer.reset();
                    stateUnsorted=3;
                }
                break;
            case 3:
                if(sortTimer.milliseconds()>500) {
                    transfer.setTargetDeg(transfer.wrap360(transfer.getPositionDeg() + 45), sec);
                    readyToShoot=true;
                    stateUnsorted = -1;
                }
                break;
        }
    }

    private void primeSortedBalls(double sec){
        switch (stateSorted){
            case -1:
                break;
            case 0:
                transfer.retract();
                stateSorted=1;
                break;
            case 1:
                transfer.scan(sec);
                stateSorted=2;
                break;
            case 2:
                if(transfer.atTarget()) {
                    transfer.scan(sec);
                    stateSorted=3;
                }
                break;
            case 3:
                transfer.setTargetDeg(transfer.spin(motif), sec);
                stateSorted = 4;
                break;
            case 4:
                if(transfer.atTarget()) {
                    transfer.score();
                    stateSorted = 5;
                }
                break;
            case 5:
                transfer.setTargetDeg(transfer.wrap360(transfer.getPositionDeg() + 45), sec);
                readyToShoot=true;
                stateUnsorted=-1;
        }
    }

    public void shootPrimedBalls(double dist){
        switch (stateShoot){
            case -1:
                break;
            case 0:
                transfer.startTransfer(dist);
                stateShoot =1;
                shootStateTimer.reset();
                break;
            case 1:
                if(shootStateTimer.milliseconds()>1500) {
                    transfer.endTransfer();
                    transfer.setAuto();
                    transfer.retract();
                    shooter.setTarget(0);
                    readyToShoot=false;
                    stateShoot = -1;
                }
                break;

        }
    }

    private boolean isSlowMode() {
        return driveMult!=1&&rotateMult!=1;
    }

    private void setFastMode() {
        driveMult =1;
        rotateMult=1;
    }

    private void setSlowMode() {
        driveMult =.3;
        rotateMult=.5;
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

    double myXg =0 ;
    double myYg =0;
    double myYawg =0;
    double myX =0 ;
    double myY =0;
    double myYaw =0;
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
            }
        }

        Pose ftcStandard = PoseConverter.pose2DToPose(new Pose2D(DistanceUnit.INCH,-myY-72,myX-72,AngleUnit.DEGREES,myYaw), InvertedFTCCoordinates.INSTANCE);
//        ftcStandard=ftcStandard.getAsCoordinateSystem(PedroCoordinates.INSTANCE);
        if(!currentDetections.isEmpty()) {
            follower.setPose(new Pose(myY+72,72-myX, Math.toRadians(myYaw)));
        }
        myXg=follower.getPose().getX();
        myYg=follower.getPose().getY();
        myYawg=Math.toDegrees(follower.getPose().getHeading());
    }
}