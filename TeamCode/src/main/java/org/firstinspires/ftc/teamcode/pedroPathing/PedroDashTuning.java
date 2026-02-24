package org.firstinspires.ftc.teamcode.pedroPathing;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;

@Config
//@Configurable
public class PedroDashTuning {
    public static PIDFCoefficients translational = new PIDFCoefficients(
            0.15,
            0,
            0.01,
            0.02
    );
    public static PIDFCoefficients secondaryTranslational = new PIDFCoefficients(
            0.13,
            0,
            0.01,
            0.0005
    );

    public static PIDFCoefficients heading = new PIDFCoefficients(
            5,
            0,
            0.5,
            0.03
    );
    public static PIDFCoefficients secondaryHeading = new PIDFCoefficients(
            1.7,
            0,
            0.05,
            0.03
    );

    public static FilteredPIDFCoefficients drive = new FilteredPIDFCoefficients(
            0.05,
            0,
            0.0001,
            0.6,
            0.01
    );
    public static FilteredPIDFCoefficients secondaryDrive = new FilteredPIDFCoefficients(
            0.028,
            0,
            0.0001,
            0.6,
            0.01
    );

    public static double centri=.005;
    public static double brakeStrength=.91;
    public static double breakStart=1;
}
