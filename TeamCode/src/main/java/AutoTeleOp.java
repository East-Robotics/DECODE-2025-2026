package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

@TeleOp(name = "AutoTeleOp", group = "TeleOp")
public class AutoTeleOp extends LinearOpMode {
    private DcMotor LFMotor;
    private DcMotor RFMotor;
    private DcMotor LBMotor;
    private DcMotor RBMotor;
    private DcMotorEx LSlide;
    private DcMotorEx RSlide;
    private DcMotorEx LArm;
    private DcMotorEx RArm;
    private Servo Wrist;
    private Servo Claw;
    private DcMotorEx driveEncoder;

    private DcMotorEx strafeEncoder;
    private boolean WristIsOpen = true;
    private boolean ClawIsOpen = true;
    private boolean lastYState = false;
    private boolean currentYState = false;
    private boolean lastXState = false;
    private boolean currentXState = false;

    // FSM state enum for slide
    public enum SlideState {
        SLIDE_START,
        SLIDE_UP
    }

    public enum ClawState {
        CLAW_IDLE,
        CLAW_MOVING
    }

    // Persistent state variable
    private SlideState slideState = SlideState.SLIDE_START;
    private ClawState clawState = ClawState.CLAW_IDLE;

    // Constants for slide
    private final int SLIDE_UP_POSITION = -1500; // Target position for slide up
    private final int SLIDE_START_POSITION = 100;
    private final int SLIDE_TOLERANCE = 50; // Encoder tolerance
    private boolean wasLBumperPressed = false; // For slide trigger
    private boolean wasRBumperPressed = false;
    private boolean wasXPressed = false;
    private final double ClawClosedPos = 0.1;
    private final double ClawOpenPos = 0.27;
    private final double CLAW_MOVE_DURATION = 0.5;
    private double clawStartTime;

    public class Drive{
        public Action turn(double angle){
            return new TodoAction();
        }
    }

    private SimplifiedOdometryRobot robot;





    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize hardware
        LFMotor = hardwareMap.get(DcMotor.class, "LFMotor");
        RFMotor = hardwareMap.get(DcMotor.class, "RFMotor");
        LBMotor = hardwareMap.get(DcMotor.class, "LBMotor");
        RBMotor = hardwareMap.get(DcMotor.class, "RBMotor");
        LSlide = hardwareMap.get(DcMotorEx.class, "LSlide");
        RSlide = hardwareMap.get(DcMotorEx.class, "RSlide");
        LArm = hardwareMap.get(DcMotorEx.class, "LArm");
        RArm = hardwareMap.get(DcMotorEx.class, "RArm");
        driveEncoder = hardwareMap.get(DcMotorEx.class, "LFMotor");
        strafeEncoder = hardwareMap.get(DcMotorEx.class, "LFMotor");
        Wrist = hardwareMap.get(Servo.class, "Wrist");
        Claw = hardwareMap.get(Servo.class, "Claw");

        robot = new SimplifiedOdometryRobot(this);
        robot.initialize(true);
        robot.resetHeading();




        // Configure drivetrain
        LBMotor.setDirection(DcMotor.Direction.REVERSE);
        LFMotor.setDirection(DcMotor.Direction.REVERSE);

        // Configure slide motors
//        LSlide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//        RSlide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        LSlide.setDirection(DcMotorSimple.Direction.FORWARD);
        RSlide.setDirection(DcMotorSimple.Direction.FORWARD);
        LSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LSlide.setTargetPosition(0);
        RSlide.setTargetPosition(0);
        LSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Configure arm motors
//        LArm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//        RArm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        LArm.setDirection(DcMotorSimple.Direction.REVERSE);
        RArm.setDirection(DcMotorSimple.Direction.FORWARD);
        LArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        // Initialize IMU
        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        imu.initialize(parameters);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        imu.resetYaw();

        while (opModeIsActive()) {
            // Driving logic
            double py = -gamepad1.left_stick_y;
            double px = gamepad1.left_stick_x;
            double pa = gamepad1.right_stick_x;
            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            botHeading -= Math.toRadians(90);

            if (gamepad1.start) {
                imu.resetYaw();
            }

            double rotX = px * Math.cos(-botHeading) - py * Math.sin(-botHeading);
            double rotY = px * Math.sin(-botHeading) + py * Math.cos(-botHeading);
            rotX = rotX * 1.5; // Counteract imperfect strafing
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(pa), 1);
            double LFtgtPower = (rotY + rotX + pa) / (denominator / 2);
            double LBtgtPower = (-rotY + rotX - pa) / (denominator / 2);
            double RFtgtPower = (rotY - rotX - pa) / (denominator / 2);
            double RBtgtPower = (rotY + rotX - pa) / (denominator / 2);

            RFMotor.setPower(RFtgtPower);
            LFMotor.setPower(LFtgtPower);
            RBMotor.setPower(RBtgtPower);
            LBMotor.setPower(LBtgtPower);



            // FSM for slide
            switch (slideState) {
                case SLIDE_START:
                    if (gamepad1.left_bumper && !wasLBumperPressed) {
//                        LSlide.setTargetPosition(SLIDE_UP_POSITION);
//                        RSlide.setTargetPosition(SLIDE_UP_POSITION);
//                        LSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                        RSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//                        LSlide.setPower(1);
//                        RSlide.setPower(1);

                        robot.drive(85, 0.5, 0.25);

//                        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
//                                .lineToX(5);
//
//                        Action trajectoryActionClose1 = tab1.endTrajectory().fresh()
//                                //.strafeTo(new Vector2d(48, 12))
//                                .build();
//
//                        Actions.runBlocking(
//                                new SequentialAction(
//                        tab1.build(),
//                        trajectoryActionClose1));


                        slideState = SlideState.SLIDE_UP;


                    } else if (Math.abs(LSlide.getCurrentPosition() - Math.abs(SLIDE_START_POSITION)) < SLIDE_TOLERANCE){
                        LSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        RSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        LSlide.setPower(0);
                        RSlide.setPower(0);

                    }
                    break;

                case SLIDE_UP:

                    if (gamepad1.right_bumper && !wasRBumperPressed) {
                        LSlide.setTargetPosition(3000);
                        RSlide.setTargetPosition(SLIDE_START_POSITION);
                        LSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        RSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);


                        LSlide.setPower(0.7);
                        RSlide.setPower(0.7);
                        slideState = SlideState.SLIDE_START;

                    } else if (Math.abs(LSlide.getCurrentPosition() - Math.abs(SLIDE_UP_POSITION)) < SLIDE_TOLERANCE) {
                        LSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        RSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        LSlide.setPower(0);
                        RSlide.setPower(0);


                    }
                    break;
                default:
                    slideState = SlideState.SLIDE_START;
                    break;
            }



            // Cancel slide action
            if (gamepad1.y && slideState != SlideState.SLIDE_START) {
                LSlide.setPower(0);
                RSlide.setPower(0);
                LSlide.setTargetPosition(LSlide.getCurrentPosition());
                RSlide.setTargetPosition(RSlide.getCurrentPosition());
                slideState = SlideState.SLIDE_START;
            }

            // Update slide edge detection
            wasLBumperPressed = gamepad1.left_bumper;
            wasRBumperPressed = gamepad1.right_bumper;


            // Claw toggle (non-blocking)
            switch (clawState){
                case CLAW_IDLE:
                    if (gamepad1.x && !wasXPressed) {
                        ClawIsOpen = !ClawIsOpen;
                        Claw.setPosition(ClawIsOpen ? ClawOpenPos : ClawClosedPos);
                        clawStartTime = System.nanoTime() / 1e9;
                        clawState = ClawState.CLAW_MOVING;
                    }
                    break;
                case CLAW_MOVING:
                    if (System.nanoTime() / 1e9 - clawStartTime >= CLAW_MOVE_DURATION) {
                        clawState = ClawState.CLAW_IDLE;
                    }
                    break;
            }


            // Wrist toggle (non-blocking)


            // Arm control
            int RArmPos = RArm.getCurrentPosition();
            if (gamepad2.right_bumper) { // Down
                if (RArmPos < 50) {
                    LArm.setPower(0);
                    RArm.setPower(0);
                } else {
                    LArm.setPower(-0.85);
                    RArm.setPower(-0.85);
                }
            } else if (gamepad2.left_bumper) { // Up
                if (RArmPos > 2700) {
                    LArm.setPower(0);
                    RArm.setPower(0);
                } else {
                    LArm.setPower(0.85);
                    RArm.setPower(0.85);
                }
            } else {
                LArm.setPower(0);
                RArm.setPower(0);
            }

            // Manual slide control (only if not in SLIDE_UP state)
//            int LSlidePos = LSlide.getCurrentPosition();
//            if (slideState == SlideState.SLIDE_START) {
//                if (gamepad2.right_bumper) { // Down
//                    LSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                    RSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                    LSlide.setPower(0.5);
//                    RSlide.setPower(0.5);
//                } else if (gamepad2.left_bumper) { // Up
//                    if (RArmPos < 1300 && LSlidePos > 900) {
//                        LSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                        RSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                        LSlide.setPower(0);
//                        RSlide.setPower(0);
//                    } else {
//                        LSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                        RSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                        LSlide.setPower(-0.5);
//                        RSlide.setPower(-0.5);
//                    }
//                } else {
//                    LSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                    RSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                    LSlide.setPower(-0.04);
//                    RSlide.setPower(-0.04);
//                }
//            }

            // Reset encoders
            if (gamepad1.b) {
                LArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                RArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                LSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                RSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                LSlide.setTargetPosition(0);
                RSlide.setTargetPosition(0);
                LSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                RSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                LArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                RArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            // Telemetry
            telemetry.addData("SlideState", slideState);
            telemetry.addData("ClawState", clawState);
            telemetry.addData("wasXPressed", wasXPressed);
            telemetry.addData("LSlidePos", LSlide.getCurrentPosition());
            telemetry.addData("RSlidePos", RSlide.getCurrentPosition());
            telemetry.addData("ClawPos", ClawIsOpen);
            telemetry.addData("WristPos", WristIsOpen);
            telemetry.addData("RArmPos", RArmPos);
            telemetry.addData("Status", "Running");
            telemetry.update();
        }
    }
}