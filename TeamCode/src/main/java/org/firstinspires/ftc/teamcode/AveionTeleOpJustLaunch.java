package org.firstinspires.ftc.teamcode;

// Imports

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="AveionTeleOpJustLaunch", group = "Aveion")
//@Disabled
public class AveionTeleOpJustLaunch extends OpMode{

    private boolean spinning;
    final double FEED_TIME_SECONDS = 0.20;
    final double STOP_SPEED = 0.0;
    final double FULL_SPEED = 1.0;

    // Launcher Target and Min Velocities
    final double LAUNCHER_TARGET_VELOCITY = 1200;
    final double LAUNCHER_MIN_VELOCITY = 1100;


    //Declare Motors
    private DcMotorEx launcher1 = null;
    private DcMotorEx launcher2 = null;

    ElapsedTime feederTimer = new ElapsedTime();

    private enum LaunchState {
        IDLE,
        SPOOL,
        PUSH,
        LAUNCH,
        LAUNCHING,
    }
    private LaunchState launchState;

    //Variables for each drive motor
    double frontRightPower;
    double frontLeftPower;
    double backRightPower;
    double backLeftPower;

    /*/////////////////////////////////////////////////////////////////////////////////
    INITIALIZE
    /////////////////////////////////////////////////////////////////////////////////*/
    @Override
    public void init(){
        launchState = LaunchState.IDLE;

        //Configuration

        launcher1 = hardwareMap.get(DcMotorEx.class, "launch1");
        launcher2 = hardwareMap.get(DcMotorEx.class, "launch2");
        //Motor Directions

            //Drive Motors

            //Launch Motors
            launcher1.setDirection(DcMotorSimple.Direction.REVERSE);
            launcher1.setDirection(DcMotorSimple.Direction.FORWARD);

        //Servo Directions


        //Motor Behaviors

            //Launch Motors
            launcher1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            launcher2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //Drive Motors

        //Servo Behaviors

            //Feeders

        //PIDF Coefficients (i actually don't know how these work)
        launcher1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));
        launcher2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*//////////////////////////////////////////////////////////////////////////////////////
    BEFORE START
    /////////////////////////////////////////////////////////////////////////////////////*/

    @Override
    public void init_loop() {
        //Set push servos to 0 position
    }
    /*//////////////////////////////////////////////////////////////////////////////////////
    START
    /////////////////////////////////////////////////////////////////////////////////////*/

    @Override
    public void start() {
        spinning = false;
    }

    /*//////////////////////////////////////////////////////////////////////////////////////
    START LOOP BEFORE STOP
    /////////////////////////////////////////////////////////////////////////////////////*/

    @Override
    public void loop() {

        if(gamepad1.circleWasPressed()){
            spinning = false;
        }
        if(gamepad1.crossWasPressed()) {
            spinning = true;
            Spool();
        }
        telemetry.addData("Spinning: ", spinning);
        telemetry.addData("Launcher 1: ", launcher1.getVelocity());
        telemetry.addData("Launcher 2: ", launcher2.getVelocity());
    }

    /*//////////////////////////////////////////////////////////////////////////////////////
    STOP
    /////////////////////////////////////////////////////////////////////////////////////*/
    @Override
    public void stop() {
    }

    /*//////////////////////////////////////////////////////////////////////////////////////
    METHODS
    /////////////////////////////////////////////////////////////////////////////////////*/
    void Spool(){

        while(spinning){
            launcher1.setVelocity(LAUNCHER_TARGET_VELOCITY);
            launcher2.setVelocity(LAUNCHER_TARGET_VELOCITY);
        }
    }
    /*void launch(boolean shotRequested) {
        switch (launchState) {
            case IDLE:
                if (shotRequested) {
                    launchState = LaunchState.SPOOL;
                }
                break;
            case SPOOL:
                launcher1.setVelocity(LAUNCHER_TARGET_VELOCITY);
                if (launcher1.getVelocity() > LAUNCHER_MIN_VELOCITY) {
                     launchState = LaunchState.LAUNCH;
                }
                break;
            case LAUNCH:
                leftConveyor.setPower(FULL_SPEED);
                rightConveyor.setPower(FULL_SPEED);
                feederTimer.reset();
                launchState = LaunchState.LAUNCHING;
                break;
            case LAUNCHING:
                if (feederTimer.seconds() > FEED_TIME_SECONDS) {
                    launchState = LaunchState.IDLE;
                    leftConveyor.setPower(STOP_SPEED);
                    rightConveyor.setPower(STOP_SPEED);
                }
                break;
        }
    }*/
}
