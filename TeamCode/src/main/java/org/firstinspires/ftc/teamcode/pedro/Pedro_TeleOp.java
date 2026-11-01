package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.pedropathing.math.Pose;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.ManualDrive;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp(name = "BIOBUZZ TeleOp", group = "BIOBUZZ")

public class Pedro_TeleOp extends OpMode {

    // =========================================
    // PEDRO PATHING
    // =========================================

    private Follower follower;

    // =========================================
    // MECHANISMS
    // =========================================

    private DcMotor intake;
    private DcMotorEx launcher;
    private DcMotorEx windmill;

    // =========================================
    // IVY COMMANDS
    // =========================================

    private Command intakeCommand;
    private Command launcherCommand;
    private Command windmillCommand;

    // =========================================
    // LAUNCHER SETTINGS
    // =========================================

    private static final double LAUNCHER_RPM = 320.0;

    private static final double P = 40.0;
    private static final double I = 0.0;
    private static final double D = 0.0;
    private static final double F = 12.5;

    // =========================================
    // DRIVE SETTINGS
    // =========================================

    private boolean slowMode = false;
    private boolean previousA = false;

    private double driveSpeed = 1.0;

    // =========================================
    // INITIALIZATION
    // =========================================

    @Override
    public void init() {

        Scheduler.reset();

        // Initialize Pedro Pathing
        follower = Constants.create(hardwareMap);

        follower.setPose(new Pose(0, 0, 0));

        // Initialize intake
        intake = hardwareMap.get(
                DcMotorEx.class, "intake"
        );

        intake.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        intake.setPower(0);

        // Initialize launcher
        launcher = hardwareMap.get(
                DcMotorEx.class, "launcher"
        );

        launcher.setMode(
                DcMotor.RunMode.RUN_USING_ENCODER
        );

        launcher.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.FLOAT
        );

        launcher.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(P, I, D, F)
        );

        launcher.setVelocity(0);

        // Initialize windmill
        windmill = hardwareMap.get(
                DcMotorEx.class, "windmill_motor"
        );

        windmill.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        windmill.setPower(0);

        createCommands();

        telemetry.addLine("BIOBUZZ ROBOT CENTRIC READY");
        telemetry.addLine("Pedro Pathing 3 + Ivy");

        telemetry.update();
    }

    // =========================================
    // RPM CONVERSION
    // =========================================

    private double rpmToTicksPerSecond(double rpm) {

        double ticksPerRevolution =
                launcher.getMotorType().getTicksPerRev();

        return rpm * ticksPerRevolution / 60.0;
    }

    // =========================================
    // IVY COMMANDS
    // =========================================

    private void createCommands() {

        // INTAKE
        intakeCommand = Command.build()

                .setExecute(() -> {

                    if (gamepad2.right_trigger > 0.1) {
                        intake.setPower(1.0);
                    }
                    else if (gamepad2.left_trigger > 0.1) {
                        intake.setPower(-1.0);
                    }
                    else {
                        intake.setPower(0);
                    }
                })

                .setDone(() ->
                        gamepad2.right_trigger < 0.1 &&
                                gamepad2.left_trigger < 0.1
                )

                .setEnd(end -> intake.setPower(0))

                .requiring(intake);

        // LAUNCHER
        launcherCommand = Command.build()

                .setExecute(() ->
                        launcher.setVelocity(
                                rpmToTicksPerSecond(LAUNCHER_RPM)
                        )
                )

                .setDone(() -> !gamepad2.right_bumper)

                .setEnd(end -> launcher.setVelocity(0))

                .requiring(launcher);

        // WINDMILL
        // Left bumper = Forward
        // X = Reverse

        windmillCommand = Command.build()

                .setExecute(() -> {

                    if (gamepad2.left_bumper) {
                        windmill.setPower(0.6);
                    }
                    else if (gamepad2.x) {
                        windmill.setPower(-0.6);
                    }
                    else {
                        windmill.setPower(0);
                    }
                })

                .setDone(() ->
                        !gamepad2.left_bumper &&
                                !gamepad2.x
                )

                .setEnd(end -> windmill.setPower(0))

                .requiring(windmill);
    }

    // =========================================
    // START
    // =========================================

    @Override
    public void start() {
        follower.update();
    }

    // =========================================
    // MAIN LOOP
    // =========================================

    @Override
    public void loop() {

        // =====================================
        // GAMEPAD 1: ROBOT CENTRIC DRIVE
        // =====================================

        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        // Toggle slow mode
        if (gamepad1.a && !previousA) {
            slowMode = !slowMode;
        }

        previousA = gamepad1.a;

        driveSpeed = slowMode ? 0.35 : 1.0;

        forward *= driveSpeed;
        strafe *= driveSpeed;
        turn *= driveSpeed;

        // Robot-centric manual driving
        follower.manual(
                forward,
                strafe,
                turn
        );

        // =====================================
        // GAMEPAD 2: INTAKE
        // =====================================

        boolean intakeRequested =
                gamepad2.right_trigger > 0.1 ||
                        gamepad2.left_trigger > 0.1;

        // =====================================
        // GAMEPAD 2: LAUNCHER
        // =====================================

        boolean launcherRequested =
                gamepad2.right_bumper;

        // =====================================
        // GAMEPAD 2: WINDMILL
        // =====================================

        boolean windmillRequested =
                gamepad2.left_bumper ||
                        gamepad2.x;

        // =====================================
        // EMERGENCY STOP
        // =====================================

        if (gamepad2.y) {

            Scheduler.reset();

            intake.setPower(0);
            launcher.setVelocity(0);
            windmill.setPower(0);

        } else {

            if (intakeRequested &&
                    !intakeCommand.isScheduled()) {

                Scheduler.schedule(intakeCommand);
            }

            if (launcherRequested &&
                    !launcherCommand.isScheduled()) {

                Scheduler.schedule(launcherCommand);
            }

            if (windmillRequested &&
                    !windmillCommand.isScheduled()) {

                Scheduler.schedule(windmillCommand);
            }
        }

        // =====================================
        // UPDATE PEDRO
        // =====================================

        follower.update();

        // =====================================
        // UPDATE IVY
        // =====================================

        Scheduler.execute();

        // =====================================
        // TELEMETRY
        // =====================================

        Pose pose = follower.pose();

        telemetry.addLine("=== BIOBUZZ TELEOP ===");

        telemetry.addLine("--- DRIVE ---");

        telemetry.addData(
                "Drive Mode", "ROBOT CENTRIC"
        );

        telemetry.addData(
                "Speed", slowMode ? "SLOW" : "NORMAL"
        );

        telemetry.addData("X", pose.x());
        telemetry.addData("Y", pose.y());

        telemetry.addData(
                "Heading",
                Math.toDegrees(pose.heading())
        );

        telemetry.addLine("--- MECHANISMS ---");

        telemetry.addData(
                "Intake Power", intake.getPower()
        );

        telemetry.addData(
                "Launcher RPM",
                launcher.getVelocity() * 60.0 /
                        launcher.getMotorType().getTicksPerRev()
        );

        telemetry.addData(
                "Launcher Target RPM", LAUNCHER_RPM
        );

        telemetry.addData(
                "Windmill Power", windmill.getPower()
        );

        telemetry.update();
    }

    // =========================================
    // STOP
    // =========================================

    @Override
    public void stop() {

        Scheduler.reset();

        follower.stop();

        intake.setPower(0);
        launcher.setVelocity(0);
        windmill.setPower(0);
    }
}