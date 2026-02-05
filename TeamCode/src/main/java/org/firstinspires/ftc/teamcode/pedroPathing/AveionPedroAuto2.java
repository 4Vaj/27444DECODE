package org.firstinspires.ftc.teamcode.pedroPathing; // make sure this aligns with class location

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Curve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.bylazar.configurables.annotations.Configurable;
@Autonomous(name = "AveionPedroAuto", group = "Aveion")
public class AveionPedroAuto2 extends OpMode {
    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private enum PathState{
        DRIVE1,
        DRIVE2,
    };

    private final Pose startPose = new Pose(56.000, 8.000, Math.toRadians(90));
    private final Pose secondPose = new Pose(84.000, 36.000, Math.toRadians(180));
    private PathState PathState;
    private Path scorePreload; // Name of Path
    private PathChain drive1;


    @Override
    public void init(){
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

        PathState = PathState.DRIVE1;
    }
    @Override
    public void init_loop(){
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }
    @Override
    public void start(){
        opmodeTimer.resetTimer();
    }
    @Override
    public void loop(){
        follower.update(); // Update Pedro Pathing
        autonomousPathUpdate();// Update autonomous state machine

        // Log values to Panels and Driver Station
        telemetry.addData("path state", PathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }
    @Override
    public void stop() {

    }

    public static class Paths{
        public PathChain Path1;

        public Paths(Follower follower){
            Path1 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(56.000, 8.000),
                            new Pose(84.000, 36.000)))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(90),
                            Math.toRadians(180))
                    .build();
        }
    }

    public void buildPaths() {
        scorePreload = new Path(new BezierLine(startPose, secondPose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), secondPose.getHeading());
    }

    public void autonomousPathUpdate() {
        // Add your state machine Here
        switch (PathState){
            case DRIVE1:
                follower.followPath(scorePreload);
                setPathState(PathState.DRIVE2);
                break;
            case DRIVE2:
                break;
        }
    }

    public void setPathState(PathState pState) {
        PathState = pState;
        pathTimer.resetTimer();
    }
}
