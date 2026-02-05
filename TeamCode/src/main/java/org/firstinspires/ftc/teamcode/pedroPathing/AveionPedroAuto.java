package org.firstinspires.ftc.teamcode.pedroPathing; // make sure this aligns with class location
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
@Autonomous(name = "AveionPedroAuto", group = "Aveion")
public class AveionPedroAuto extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private ElapsedTime waitTimer = new ElapsedTime();

    private int pathState;

    private final Pose startPose = new Pose(0, 10, Math.toRadians(180)); // Start position. line up with the outer edge of foam tile.
    private final Pose driveToPivotPose = new Pose(0, 100, Math.toRadians(0)); // Drive to pivot point.
    private final Pose driveToGoalPose = new Pose(0, 120, Math.toRadians(135)); // Drive to goal.
    private final Pose parkPose = new Pose(0, 120, Math.toRadians(90)); // Move away from goal.

    private Path scoreBottomTriangle;
    private PathChain driveToGoalChain, parkChain;


    static final double TICKS_PER_REV = 537.7;
    static final double WHEEL_DIAMETER_IN = 4.09;
    static final double GEAR_RATIO = 1.0;
    private double wheelCircumference = Math.PI * WHEEL_DIAMETER_IN;

    // Motors
    private DcMotorEx frontRight = null;
    private DcMotorEx frontLeft = null;
    private DcMotorEx backLeft = null;
    private DcMotorEx backRight = null;


    private boolean timerRunning;
    private double timerDuration = 0;






    @Override
    public void init(){
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);


        // Motor Configuration
        frontRight = hardwareMap.get(DcMotorEx.class, "FR");
        frontLeft = hardwareMap.get(DcMotorEx.class, "FL");
        backLeft = hardwareMap.get(DcMotorEx.class, "BL");
        backRight = hardwareMap.get(DcMotorEx.class, "BR");

        // Motor Directions
        frontRight.setDirection(DcMotorEx.Direction.FORWARD);
        frontLeft.setDirection(DcMotorEx.Direction.REVERSE);
        backRight.setDirection(DcMotorEx.Direction.FORWARD);
        backLeft.setDirection(DcMotorEx.Direction.REVERSE);

        // Checks
        timerRunning = false;
    }
    @Override
    public void init_loop(){

    }
    @Override
    public void start(){
        opmodeTimer.resetTimer();
        setPathState(0);
    }
    @Override
    public void loop(){
        updateTimer();
        follower.update();
        autonomousPathUpdate();

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("Timer Running", timerRunning);
        telemetry.update();
    }
    @Override
    public void stop() {

    }

    public void buildPaths(){
        scoreBottomTriangle = new Path(new BezierLine(startPose, driveToPivotPose));
        scoreBottomTriangle.setLinearHeadingInterpolation(startPose.getHeading(), driveToPivotPose.getHeading());

        driveToGoalChain = follower.pathBuilder()
                .addPath(new BezierLine(driveToPivotPose, driveToGoalPose))
                .setLinearHeadingInterpolation(driveToPivotPose.getHeading(), driveToGoalPose.getHeading())
                .build();
        parkChain = follower.pathBuilder()
                .addPath(new BezierLine(driveToGoalPose, parkPose))
                .setLinearHeadingInterpolation(driveToGoalPose.getHeading(), parkPose.getHeading())
                .build();

    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scoreBottomTriangle);
                setPathState(1);
                break;
            case 1:
                if(!follower.isBusy()) {
                    follower.followPath(driveToGoalChain, true);
                    setPathState(2);
                    Wait(10);
                }
                break;
            case 2:
                if(!follower.isBusy() && !timerRunning){
                    moveForward(36, 0.2);
                    setPathState(3);
                    Wait(10);
                }
                break;
            case 3:
                if(!moveForwardBusy() && !timerRunning){
                    follower.followPath(parkChain, true);
                    setPathState(4);
                    Wait(10);
                }
                break;
            case 4:
                if(!follower.isBusy() && !timerRunning){
                    setPathState(-1);
                    Wait(10);
                }
                break;
        }
    }
    public void setPathState(int pState){
        pathState = pState;
        pathTimer.resetTimer();
    }

    int InchesToTicks(double distanceIN){
        return (int)((distanceIN/wheelCircumference) * TICKS_PER_REV * GEAR_RATIO);
    }

    public void moveForward(double xIN, double power){

        // Convert to encoder ticks
        int ticks = InchesToTicks(xIN);

        //Reset encoders
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Set target positions
        frontLeft.setTargetPosition(ticks);
        frontRight.setTargetPosition(ticks);
        backRight.setTargetPosition(ticks);
        backLeft.setTargetPosition(ticks);
        // Run to position
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // Set motor power
        frontLeft.setPower(power);
        frontRight.setPower(power);
        backRight.setPower(power);
        backLeft.setPower(power);
    }
    boolean moveForwardBusy(){
        return !frontRight.isBusy() && !frontLeft.isBusy() && !backLeft.isBusy() && !backRight.isBusy();
    }
    void updateTimer(){
        if(!timerRunning){
            return;
        }
        if(waitTimer.seconds() > timerDuration){
            timerRunning = false;
        }
    }

    void Wait(double time){
        waitTimer.reset();
        timerDuration = time;
        timerRunning = true;
    }



}
