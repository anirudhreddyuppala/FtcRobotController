package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.*;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT;

@TeleOp(name = "BB TeleOp Improved", group = "BIOBUZZ")
public class BB_Teleop_Improved extends OpMode {

    // Pedro Pathing
    private Follower follower;

    // Mechanisms
    private DcMotor intake;
    private DcMotor windmillMotor;
    private DcMotorEx launcher;
    private VoltageSensor batteryVoltageSensor;

    private CRServo leftIntakeServo;
    private CRServo rightIntakeServo;

    // Drive settings
    private boolean slowMode = false;
    private boolean previousA = false;
    private double driveSpeed = 1.0;

    // Ivy Auto-Launch state
    private boolean previousPad2A = false;
    private Command activeLaunchCommand = null;

    // =========================================================
    // LAUNCHER SETTINGS (goBILDA BIOBUZZ StarterBot 1:1 6000 RPM)
    // =========================================================
    // Standard goBILDA StarterBot target is 1200 - 1400 ticks/sec (~2570 - 3000 RPM)
    private static final double LAUNCHER_TARGET_RPM = 830.0; // = 1300 ticks/sec
    private static final double LAUNCHER_MIN_RPM = 801.0;    // = 1250 ticks/sec (ready threshold)
    private static final double LAUNCHER_IDLE_RPM = 800.0;   // Keeps flywheel warm for fast spin-up

    // 28-tick encoder on 1:1 6000 RPM motor shaft
    private static final double TICKS_PER_REV = 28.0;

    private static final double LAUNCHER_TARGET_VELOCITY =
            LAUNCHER_TARGET_RPM * TICKS_PER_REV / 60.0;

    private static final double LAUNCHER_MIN_VELOCITY =
            LAUNCHER_MIN_RPM * TICKS_PER_REV / 60.0;

    private static final double LAUNCHER_IDLE_VELOCITY =
            LAUNCHER_IDLE_RPM * TICKS_PER_REV / 60.0;

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
        windmillMotor = hardwareMap.get(DcMotor.class, "windmill_motor");
        leftIntakeServo = hardwareMap.get(CRServo.class, "left_intake_servo");
        rightIntakeServo = hardwareMap.get(CRServo.class, "right_intake_servo");
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        // Motor braking
        intake.setZeroPowerBehavior(BRAKE);
        windmillMotor.setZeroPowerBehavior(BRAKE);

        // Launcher configuration (RUN_USING_ENCODER + setVelocity)
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setZeroPowerBehavior(FLOAT); // FLOAT prevents flywheel from braking against itself

        // Tuned PIDF: D = 10 stops oscillation; F = 13.5 holds velocity steady at 12V
        launcher.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(30.0, 0.0, 10.0, 13.5)
        );

        // Servo directions
        rightIntakeServo.setDirection(DcMotorSimple.Direction.REVERSE);
        leftIntakeServo.setPower(0);
        rightIntakeServo.setPower(0);

        telemetry.addLine("BIOBUZZ Pedro TeleOp Initialized");
        telemetry.addLine("Gamepad 1: Driving (A = Slow Mode)");
        telemetry.addLine("Gamepad 2: Triggers = Intake | A = Auto Launch | RB = Spin Up | LB = Feed | Y = E-Stop");
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

        if (gamepad1.a && !previousA) {
            slowMode = !slowMode;
        }
        previousA = gamepad1.a;

        driveSpeed = slowMode ? 0.35 : 1.0;

        double forward = -gamepad1.left_stick_y * driveSpeed;
        double strafe = gamepad1.left_stick_x * driveSpeed * -1;
        double turn = gamepad1.right_stick_x * driveSpeed * -1;

        follower.manual(forward, strafe, turn);

        // =====================================
        // GAMEPAD 2 - MECHANISMS
        // =====================================

        boolean emergencyStop = gamepad2.y;
        boolean launcherAtSpeed = launcher.getVelocity() >= LAUNCHER_MIN_VELOCITY;

        if (emergencyStop) {

            stopMechanisms();

        } else {

            // INTAKE
            intakePower = gamepad2.right_trigger - gamepad2.left_trigger;
            intake.setPower(intakePower);
            leftIntakeServo.setPower(intakePower);
            rightIntakeServo.setPower(intakePower);

            // ONE-BUTTON AUTO LAUNCH SEQUENCE (Gamepad 2 A)
            if (gamepad2.a && !previousPad2A && activeLaunchCommand == null) {
                activeLaunchCommand = launchSequenceCommand();
                Scheduler.schedule(activeLaunchCommand);
            }
            previousPad2A = gamepad2.a;

            // Clear active command once finished
            if (activeLaunchCommand != null && activeLaunchCommand.isScheduled()) {
                activeLaunchCommand = null;
            }

            // MANUAL LAUNCHER & WINDMILL (Only active when Auto-Launch is NOT running)
            if (activeLaunchCommand == null) {

                // LAUNCHER
                // Right bumper = Full Target Velocity | Released = Idle Velocity
                if (gamepad2.right_bumper) {
                    launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
                    if (launcherAtSpeed) {
                        gamepad2.rumble(100);
                    }
                } else {
                    launcher.setVelocity(LAUNCHER_IDLE_VELOCITY);
                }

                // WINDMILL (Velocity-Gated)
                // Left bumper = Forward feed (only feeds when flywheel reaches target speed)
                // X = Reverse
                if (gamepad2.x) {
                    windmillMotor.setPower(-0.15);
                } else if (gamepad2.left_bumper) {
                    windmillMotor.setPower(0.2);
                } else {
                    windmillMotor.setPower(0);
                }
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

        telemetry.addData("Battery Voltage", "%.2f V", batteryVoltageSensor.getVoltage());
        telemetry.addData("Drive Mode", slowMode ? "SLOW" : "NORMAL");
        telemetry.addData("Auto Launch Active", activeLaunchCommand != null);
        telemetry.addData("Robot X", "%.2f", follower.pose().x());
        telemetry.addData("Robot Y", "%.2f", follower.pose().y());
        telemetry.addData("Heading", "%.1f", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Intake Power", "%.2f", intakePower);
        telemetry.addData("Launcher RPM", "%.1f / %.1f",
                launcher.getVelocity() * 60.0 / TICKS_PER_REV,
                (gamepad2.right_bumper || activeLaunchCommand != null)
                        ? LAUNCHER_TARGET_RPM : LAUNCHER_IDLE_RPM);
        telemetry.addData("Launcher Ready", launcherAtSpeed ? "YES - FIRE!" : "SPINNING UP...");
        telemetry.addData("Windmill Power", "%.2f", windmillMotor.getPower());
        telemetry.addData("Emergency Stop", emergencyStop);
        telemetry.update();
    }

    // =====================================
    // IVY COMMANDS
    // =====================================

    public Command launchSequenceCommand() {
        return sequential(
                instant(() -> launcher.setVelocity(LAUNCHER_TARGET_VELOCITY)),
                waitUntil(() -> launcher.getVelocity() >= LAUNCHER_MIN_VELOCITY),
                instant(() -> windmillMotor.setPower(0.25)),
                waitMs(1200),
                instant(() -> {
                    windmillMotor.setPower(0);
                    launcher.setVelocity(LAUNCHER_IDLE_VELOCITY);
                })
        );
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
        activeLaunchCommand = null;
        Scheduler.reset();
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
