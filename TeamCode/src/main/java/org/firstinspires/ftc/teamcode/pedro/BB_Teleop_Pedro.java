package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.follower.Follower;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import com.pedropathing.ivy.Scheduler;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

@TeleOp(name = "BB TeleOp", group = "BIOBUZZ")

public class BB_Teleop_Pedro extends OpMode {

    // Pedro Pathing
    private Follower follower;

    // Mechanisms
    private DcMotor intake;
    private DcMotor windmillMotor;
    private DcMotorEx launcher;

    private CRServo leftIntakeServo;
    private CRServo rightIntakeServo;

    // Drive settings
    private boolean slowMode = false;
    private boolean previousA = false;

    private double driveSpeed = 1.0;

    // Launcher settings
    private static final double LAUNCHER_TARGET_RPM = 630.0;
    private static final double LAUNCHER_MIN_RPM = 540.0;

    // Assumes a 28-tick motor encoder and direct shaft measurement.
    private static final double TICKS_PER_REV = 28.0;

    private static final double LAUNCHER_TARGET_VELOCITY =
            LAUNCHER_TARGET_RPM * TICKS_PER_REV / 60.0;

    private static final double LAUNCHER_MIN_VELOCITY =
            LAUNCHER_MIN_RPM * TICKS_PER_REV / 60.0;

    private double intakePower = 0;

    @Override
    public void init() {

        // Initialize Ivy
        Scheduler.reset();

        // Initialize Pedro Pathing
        follower = Constants.create(hardwareMap);

        // Initialize mechanisms
        intake = hardwareMap.get(DcMotor.class, "intake");

        launcher = hardwareMap.get(DcMotorEx.class, "launcher");

        windmillMotor = hardwareMap.get(
                DcMotor.class, "windmill_motor");

        leftIntakeServo = hardwareMap.get(
                CRServo.class, "left_intake_servo");

        rightIntakeServo = hardwareMap.get(
                CRServo.class, "right_intake_servo");

        // Motor braking
        intake.setZeroPowerBehavior(BRAKE);
        windmillMotor.setZeroPowerBehavior(BRAKE);

        // Launcher configuration
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        launcher.setZeroPowerBehavior(BRAKE);

        launcher.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(40, 0, 0, 12.5)
        );

        // Servo directions
        rightIntakeServo.setDirection(
                DcMotorSimple.Direction.REVERSE
        );

        leftIntakeServo.setPower(0);
        rightIntakeServo.setPower(0);

        telemetry.addLine("BIOBUZZ Pedro TeleOp Initialized");
        telemetry.addLine("Gamepad 1: Driving");
        telemetry.addLine("Gamepad 2: Mechanisms");
        telemetry.update();
    }

    @Override
    public void start() {
        follower.update();
    }

    @Override
    public void loop() {

        // =====================================
        // GAMEPAD 1 - ROBOT CENTRIC DRIVING
        // =====================================

        // Toggle slow mode using A
        if (gamepad1.a && !previousA) {
            slowMode = !slowMode;
        }

        previousA = gamepad1.a;

        driveSpeed = slowMode ? 0.35 : 1.0;

        double forward =
                -gamepad1.left_stick_y * driveSpeed;

        double strafe =
                gamepad1.left_stick_x * driveSpeed *-1;

        double turn =
                gamepad1.right_stick_x * driveSpeed * -1;

        // Pedro robot-centric manual drive
        follower.manual(forward, strafe, turn);

        // =====================================
        // GAMEPAD 2 - MECHANISMS
        // =====================================

        boolean emergencyStop = gamepad2.y;

        if (emergencyStop) {

            stopMechanisms();

        } else {

            // INTAKE
            // Right trigger = Forward
            // Left trigger = Reverse

            intakePower =
                    gamepad2.right_trigger -
                            gamepad2.left_trigger;

            intake.setPower(intakePower);

            // Left and right intake servos
            leftIntakeServo.setPower(intakePower);
            rightIntakeServo.setPower(intakePower);

            // LAUNCHER
            // Right bumper = Launcher ON

            if (gamepad2.right_bumper) {

                launcher.setVelocity(
                        LAUNCHER_TARGET_VELOCITY
                );

            } else {

                launcher.setVelocity(0);
            }

            // WINDMILLWINDMILL
            // Left bumper = Forward feed
            // X = Reverse

            if (gamepad2.x) {

                windmillMotor.setPower(-0.15);

            } else if (
                    gamepad2.left_bumper ) {

                windmillMotor.setPower(0.2);

            } else {
                windmillMotor.setPower(0);
            }
        }

        // =====================================
        // UPDATE PEDRO AND IVY
        // =====================================

        follower.update();

        Scheduler.execute();

        // =====================================
        // TELEMETRY
        // =====================================

        telemetry.addData(
                "Drive Mode",
                slowMode ? "SLOW" : "NORMAL"
        );

        telemetry.addData(
                "Robot X",
                "%.2f", follower.pose().x()
        );

        telemetry.addData(
                "Robot Y",
                "%.2f", follower.pose().y()
        );

        telemetry.addData(
                "Heading",
                "%.1f",
                Math.toDegrees(follower.pose().heading())
        );

        telemetry.addData(
                "Intake Power",
                "%.2f", intakePower
        );

        telemetry.addData(
                "Launcher RPM",
                "%.1f",
                launcher.getVelocity() * 60.0 / TICKS_PER_REV
        );

        telemetry.addData(
                "Windmill Power",
                emergencyStop ? 0 :
                        gamepad2.x ? -1 :
                        gamepad2.left_bumper &&
                                launcher.getVelocity() > LAUNCHER_MIN_VELOCITY
                        ? 1 : 0
        );

        telemetry.addData(
                "Emergency Stop",
                emergencyStop
        );

        telemetry.update();
    }

    // =====================================
    // STOP ALL MECHANISMS
    // =====================================

    private void stopMechanisms() {

        intakePower = 0;

        intake.setPower(0);

        leftIntakeServo.setPower(0);
        rightIntakeServo.setPower(0);

        windmillMotor.setPower(0);

        launcher.setVelocity(0);
    }

    @Override
    public void stop() {

        stopMechanisms();

        if (follower != null) {
            follower.stop();
        }

        Scheduler.reset();
    }
}
