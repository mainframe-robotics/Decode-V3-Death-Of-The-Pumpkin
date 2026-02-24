package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(11.75)
            .forwardZeroPowerAcceleration(-55.801)
            .lateralZeroPowerAcceleration(-78.35)
            .useSecondaryTranslationalPIDF(true)
            .useSecondaryHeadingPIDF(true)
            .useSecondaryDrivePIDF(true)
            .translationalPIDFCoefficients(PedroDashTuning.translational)
            .secondaryTranslationalPIDFCoefficients(PedroDashTuning.secondaryTranslational)
            .headingPIDFCoefficients(PedroDashTuning.heading)
            .secondaryHeadingPIDFCoefficients(PedroDashTuning.secondaryHeading)
            .drivePIDFCoefficients(PedroDashTuning.drive)
            .secondaryDrivePIDFCoefficients(PedroDashTuning.secondaryDrive);
//            .holdPointHeadingScaling(0.6)
//            .holdPointTranslationalScaling(1);
//            .centripetalScaling(PedroDashTuning.centri);


    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("fr")
            .rightRearMotorName("br")
            .leftRearMotorName("bl")
            .leftFrontMotorName("fl")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(80.83)
            .yVelocity(63.12);
    public static PinpointConstants localizerConstants = new PinpointConstants()
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD )
            .strafePodX(5.37)
            .forwardPodY(-4.496);
    public static PathConstraints pathConstraints = new PathConstraints(0.95, 0.1, 0.1, 0.011, 50, PedroDashTuning.brakeStrength, 10, PedroDashTuning.breakStart);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();

    }
}
