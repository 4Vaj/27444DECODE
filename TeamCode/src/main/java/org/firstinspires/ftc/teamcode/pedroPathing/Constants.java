package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.PIDFCoefficients;
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
            .mass(11.3398)
            .lateralZeroPowerAcceleration(79.7) //
            .forwardZeroPowerAcceleration(79.7) // If too much overshoot or aggression when braking lower to 70.
            //.useSecondaryTranslationalPIDF(true)
            //.useSecondaryHeadingPIDF(true)
            //.useSecondaryDrivePIDF(true)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.16, 0.00000005,0.011,0.026))
            .headingPIDFCoefficients(new PIDFCoefficients(1.45, 0, 0.08, 0));

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(0.4)
            .rightFrontMotorName("FR")
            .rightRearMotorName("BR")
            .leftRearMotorName("BL")
            .leftFrontMotorName("FL")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .yVelocity(64.1)
            .xVelocity(51.28); // Because no x pod use 80% of yVelocity

    public static PathConstraints pathConstraints = new PathConstraints(0.8, 40, Math.toRadians(180), 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }

    // GO BACK TO PEDRO PATH STEPS AND GO TO ENCODER RESOLUTION.
    public static PinpointConstants localizerConstants = new PinpointConstants()
            //.forwardPodY(4) // Y-Pod distance to center
            .forwardPodY(0)
            .strafePodX(0.625) // X-Pod distance to center
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);
}
