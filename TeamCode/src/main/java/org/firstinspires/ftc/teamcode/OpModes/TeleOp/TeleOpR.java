package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.InvertedFTCCoordinates;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
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

import java.util.Arrays;
import java.util.List;

@TeleOp
@Config
public class TeleOpR extends LinearOpMode {

    Transfer transfer;
    Turret turret;
    Shooter shooter;
    Follower follower;
    private ElapsedTime timer, readyStateTimer,shootStateTimer;

    public static double ange=180;

    public static double spinMult=.8;

    public static Pose goalPose = new Pose(1,142).mirror();

    public static double velo;
    private int readyState=-1;
    private int shootState=-1;

    public  static String x = "PGP";
    private int intakeState=0;
    private DcMotor intake;
    public static double intakePos =350;
    public static double rotateMult=1, driveMult=1;


    @Override
    public void runOpMode(){
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72,72,Math.toRadians(90)));
        follower.startTeleopDrive();
        follower.update();
        transfer = new Transfer(hardwareMap);
        shooter = new Shooter(hardwareMap);
        intake = hardwareMap.dcMotor.get("int");

        Servo hinge = hardwareMap.servo.get("hinge");
//        Servo hood = hardwareMap.get(Servo.class,"hood1");

        turret = new Turret(hardwareMap);
        initAprilTag(hardwareMap);
//        DcMotorEx motor=hardwareMap.get(DcMotorEx.class,"shootL");
//        DcMotorEx motor2=hardwareMap.get(DcMotorEx.class,"shootR");
//        motor2.setDirection(DcMotorSimple.Direction.REVERSE);
        timer = new ElapsedTime();
        timer.reset();
        readyStateTimer = new ElapsedTime();
        readyStateTimer.reset();
        shootStateTimer = new ElapsedTime();
        shootStateTimer.reset();
        transfer.retract();

//        transfer.setTargetDeg(transfer.getPositionDeg());
//        transfer.targetDeg=transfer.getPositionDeg();
        waitForStart();
        while (opModeIsActive()){
            double sec = timer.seconds();

//            motor.setPower(gamepad1.left_stick_y);
//            motor2.setPower(gamepad1.left_stick_y);
//            turret.setTargetDeg(ange);
//            if(gamepad1.aWasPressed()){
//                transfer.scan(sec);
//            }

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
//            if(gamepad1.leftBumperWasPressed()){
//                shooter.forDistance(Math.hypot(goalPose.getX()-follower.getPose().getX(),goalPose.getY()-follower.getPose().getY()));
//            }

//            if(gamepad1.aWasPressed()){
//                transfer.startTransfer(Math.hypot(goalPose.getX()-follower.getPose().getX(),goalPose.getY()-follower.getPose().getY()));
//            }
//            if(gamepad1.aWasReleased()){
//                transfer.endTransfer();
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

//            transfer.setManual();
//            transfer.manualPower = gamepad1.left_stick_y*spinMult;


//            intake.setPower(-gamepad1.right_trigger);
//            if (gamepad1.dpad_down)
//            {
//                hinge.setPosition(.3175);
//            }
//            if (gamepad1.dpad_up)
//            {
//                hinge.setPosition(.41);
//            }
//            if(gamepad1.right_trigger>.01){
//                transfer.setAuto();
//                transfer.setTargetDeg(10,sec);
//                transfer.retract();
//            }
//            if(gamepad1.startWasPressed()){
//                transfer.startTransfer(Math.hypot(goalPose.getX()-follower.getPose().getX(),goalPose.getY()-follower.getPose().getY()));
//            } else if (gamepad1.startWasReleased()) {
//                transfer.endTransfer();
//            }
            //


//


//
//            shooter.setHood(ange);
//            hood.setPosition(ange);
//            shooter.setTarget(velo);

            //            turret.setTargetDeg(ange);

//            if(gamepad1.right_trigger!=0){
//                driveMult=.5;
//                rotateMult=.4;
//            }
//            else{
//                driveMult=1;
//                rotateMult=1;
//            }

            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y*driveMult ,
                    -gamepad1.left_stick_x*driveMult ,
                    -gamepad1.right_stick_x*rotateMult , true);
//


            double dist = Math.hypot(goalPose.getX()-follower.getPose().getX(),goalPose.getY()-follower.getPose().getY());

//            if((gamepad1.bWasPressed()||gamepad1.aWasPressed())&&intakeState!=0){
//                shootState=-1;
//                intakeState=0;
//            }
//            if (gamepad1.bWasPressed()&&intakeState==0) {
//                shootState=-1;
//                intakeState=-1;
//            }
//            if (gamepad1.aWasPressed()&&intakeState==0) {
//                shootState=-1;
//                intakeState=1;
//            }

            if(gamepad1.bWasPressed()){
                intakeState=-1;
                shootState=-1;
                transfer.retract();
            }
            if(gamepad1.aWasPressed()){
                intakeState=1;
                shootState=-1;
                transfer.retract();
            }

            if(gamepad1.xWasPressed()){
                intakeState=0;
            }



            if(gamepad1.yWasPressed()){
                shootState=0;
                intakeState=0;
            }

            if(gamepad1.right_bumper) {
                setRobotPoseFromCamera();
                turret.setYaw(-120);
            }else{
                turret.facePoint(goalPose,follower.getPose());
            }
            if(gamepad1.left_bumper){
                follower.setPose(new Pose(72,72,Math.toRadians(90)));
            }




//            shoot(dist);
//            shootReady(sec,dist);
//            intake(sec);

//            if(gamepad1.left_trigger>0){
//                intake.setPower(gamepad1.left_trigger-gamepad1.right_trigger);
//            }

//            if(intakeState!=-1){
//                transfer.retract();
//                intake.setPower(gamepad1.left_trigger-gamepad1.right_trigger);
//                shooter.setTarget(0);
//            }



            transfer.setManual();
            transfer.manualPower=gamepad1.left_trigger-gamepad1.right_trigger;


            if(shootState==0){
                shooter.forDistance(dist);
            }
            else if(shootState==-1){
//                transfer.retract();
                shooter.setTarget(0);
            }




            intake.setPower(intakeState);


            if(gamepad1.dpad_up){
                transfer.retract();
            } else if (gamepad1.dpad_down) {
                transfer.score();
            }

            if (gamepad1.xWasPressed()){
                shooter.setTarget(0);
            }


//
            shooter.forDistanceHood(dist);
//            shooter.setTarget(velo);
//            shooter.setHood(ange);
            shooter.update();
            turret.update();
            transfer.update(sec);
            follower.update();
            telemetry.addData("ready state: ",readyState);
            telemetry.addData("shoot state: ",shootState);
            telemetry.addData("intake state: ",intakeState);



            telemetry.addLine(transfer.getMapString());
            telemetry.addLine(transfer.getArrString());
            telemetry.addLine(Arrays.toString(transfer.getOrderArr()));
            telemetry.addData("idx: ",transfer.findBestSlot(transfer.getOrderArr(),x));
            telemetry.addData("goal: ",x);
            telemetry.addData("offset: ", transfer.offset);

            telemetry.addData("velo: ",shooter.getVelocity());
            telemetry.addData("targetshooter: ",shooter.getTarget());
            telemetry.addData("targetTranfer: ",transfer.getTargetDeg());
            telemetry.addData("posTranfer: ",transfer.getPositionDeg());
            telemetry.addData("targetTuttet: ",turret.getTarget());
            telemetry.addData("transferManual: ",transfer.on);


            telemetry.addData("follower pose x:",follower.getPose().getX());
            telemetry.addData("follower pose y:",follower.getPose().getY());
            telemetry.addData("follower pose h:",Math.toDegrees(follower.getPose().getHeading()));
            telemetry.addData("follower dist to goal:", Math.hypot(goalPose.getX()-follower.getPose().getX(),goalPose.getY()-follower.getPose().getY()));

            Pose robotPose = follower.getPose();
            Pose ballPose = new Pose(robotPose.getX()+Math.cos(robotPose.getHeading()), robotPose.getY()+Math.sin(robotPose.getHeading()));
            telemetry.addData("ball pose x:",ballPose.getX());
            telemetry.addData("ball pose y:",ballPose.getY());
            telemetry.addData("ball dist to goal:", Math.hypot(goalPose.getX()-ballPose.getX(),goalPose.getY()-ballPose.getY()));

            telemetry.update();

        }
    }


    /*

     */

    public void shootReady(double sec, double dist){
        switch (readyState){
            case -1:
                break;
            case 0:
                shooter.forDistance(dist);
                transfer.retract();
                transfer.scan(sec);
                readyStateTimer.reset();
                readyState=101;
                break;
            case 101:
                shooter.forDistance(dist);
                if(transfer.atTarget()) {
                    transfer.scan(sec);
                    readyStateTimer.reset();
                    readyState = 1;
                }
                break;
            case 1:
                shooter.forDistance(dist);
                if(readyStateTimer.milliseconds()>200) {
                    transfer.setTargetDeg(transfer.spin(x), sec);
                    readyState = 2;
                }
                break;
            case 2:
                shooter.forDistance(dist);
                if(transfer.atTarget()) {
                    transfer.score();
                    readyState = 3;
                    readyStateTimer.reset();
                }
                break;
            case 3:
//                transfer.setManual();
//                transfer.manualPower = -.07;
                shooter.forDistance(dist);
                if(readyStateTimer.milliseconds()>200) {
                    transfer.setTargetDeg(transfer.wrap360(transfer.getPositionDeg() - 45), sec);
                    readyStateTimer.reset();
                    readyState = 4;
                }
                break;
            case 4:
                shooter.forDistance(dist);
                if(readyStateTimer.milliseconds()>00){
                    readyState=-1;
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

    public void intake(double sec){
        switch (intakeState){
            case -1:
                intake.setPower(0);
                break;
            case 0:
                transfer.retract();
//                intake.setPower(-1);
                break;
        }
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
