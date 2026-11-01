package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import static com.pedropathing.api.Paths.*;

import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Scheduler;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static java.util.concurrent.locks.LockSupport.park;

@Autonomous
public class BB_Auto extends OpMode {

    // ============================================
    // HARDWARE
    // ============================================

    private Follower follower;

    private DcMotorEx launcher;
    private DcMotor windmill;
    private DcMotor intake;

    // ============================================
    // FIELD POSES
    // Headings are in degrees.
    // ============================================

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start =
            poseFactory.of(82.8452, 132.8733, 90);

    private final Pose path1End =
            poseFactory.of(83.1916, 114.9767, 90);

    private final Pose point2Start =
            poseFactory.of(83.1916, 114.9767, 90);

    private final Pose point2 =
            poseFactory.of(115.5998, 113.1255, 90);

    private final Pose point3Start =
            poseFactory.of(115.5998, 113.1255, 90);

    private final Pose point3 =
            poseFactory.of(124.284, 43.825, 90);

    // ============================================
    // LAUNCHER SETTINGS
    // ============================================

    private static final double LAUNCHER_RPM = 320.0;

    /*
     * IMPORTANT:
     * Replace 28.0 with the encoder ticks per ONE
     * revolution of the launcher's output shaft.
     *
     * Include gearing if the encoder is on the motor
     * shaft but RPM refers to the flywheel shaft.
     */
    private static final double LAUNCHER_TICKS_PER_REV = 28.0;

    // DcMotorEx.setVelocity() uses ticks per second.
    private static final double LAUNCHER_TICKS_PER_SECOND =
            LAUNCHER_RPM * LAUNCHER_TICKS_PER_REV / 60.0;

    private static final double LAUNCHER_P = 40.0;
    private static final double LAUNCHER_I = 0.0;
    private static final double LAUNCHER_D = 0.0;
    private static final double LAUNCHER_F = 12.5;

    // ============================================
    // WINDMILL SETTINGS
    // ============================================

    private static final double WINDMILL_POWER = 0.50;

    // Four timed activations
    private static final int WINDMILL_CYCLES = 4;

    // Wait for launcher to spin up
    private static final long SPINUP_TIME_MS = 1500;

    // Time between windmill activations
    private static final long WINDMILL_INTERVAL_MS = 1800;

    // Duration of each windmill activation
    private static final long WINDMILL_ON_MS = 400;

    private long mechanismStartTime;

    // ============================================
    // PATH DEFINITIONS
    // ============================================

    private Path path1() {
        return line(start, path1End).linear(start, path1End);
    }

    private Path path2() {
        return line(point2Start, point2)
                .linear(point2Start, point2);
    }

    private Path path3() {
        return line(point3Start, point3)
                .linear(point3Start, point3);
    }

    // ============================================
    // LAUNCHER + WINDMILL COMMAND
    // ============================================

    private Command shootAndRotateWindmill() {

        return Command.build()

                .setStart(() -> {

                    mechanismStartTime = System.nanoTime();

                    // Intake OFF
                    intake.setPower(0);

                    // Start launcher at 320 RPM target
                    launcher.setVelocity(
                            LAUNCHER_TICKS_PER_SECOND
                    );

                    // Windmill initially OFF
                    windmill.setPower(0);
                })

                .setExecute(() -> {

                    // Keep intake OFF
                    intake.setPower(0);

                    // Keep launcher running
                    launcher.setVelocity(
                            LAUNCHER_TICKS_PER_SECOND
                    );

                    long elapsedMs =
                            (System.nanoTime() - mechanismStartTime)
                                    / 1_000_000L;

                    // Launcher spin-up period
                    if (elapsedMs < SPINUP_TIME_MS) {
                        windmill.setPower(0);
                        return;
                    }

                    long windmillElapsed =
                            elapsedMs - SPINUP_TIME_MS;

                    long cycle =
                            windmillElapsed / WINDMILL_INTERVAL_MS;

                    long cycleTime =
                            windmillElapsed % WINDMILL_INTERVAL_MS;

                    // Four timed windmill activations
                    if (cycle < WINDMILL_CYCLES
                            && cycleTime < WINDMILL_ON_MS) {

                        windmill.setPower(WINDMILL_POWER);

                    } else {

                        windmill.setPower(0);
                    }
                })

                .setDone(() -> {

                    long elapsedMs =
                            (System.nanoTime() - mechanismStartTime)
                                    / 1_000_000L;

                    return elapsedMs >=
                            SPINUP_TIME_MS
                                    + WINDMILL_CYCLES
                                    * WINDMILL_INTERVAL_MS;
                })

                .setEnd(endCondition -> {

                    // Stop mechanisms at command completion
                    windmill.setPower(0);
                    launcher.setVelocity(0);
                    intake.setPower(0);
                })

                .requiring(launcher, windmill, intake);
    }

    // ============================================
    // AUTONOMOUS COMMAND SEQUENCE
    // ============================================

    private Command autoRoutine() {

        return follow(follower, path1())
                .then(shootAndRotateWindmill())
                .then(follow(follower, path2()))
                .then(follow(follower, path3()));
    }

    // ============================================
    // INITIALIZATION
    // ============================================

    @Override
    public void init() {

        Scheduler.reset();

        // Hardware mapping
        launcher = hardwareMap.get(
                DcMotorEx.class, "launcher");

        windmill = hardwareMap.get(
                DcMotor.class, "windmill");

        intake = hardwareMap.get(
                DcMotor.class, "intake");

        // Launcher setup
        launcher.setPower(0);

        launcher.setMode(
                DcMotor.RunMode.RUN_USING_ENCODER);

        launcher.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(
                        LAUNCHER_P,
                        LAUNCHER_I,
                        LAUNCHER_D,
                        LAUNCHER_F
                )
        );

        launcher.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.FLOAT);

        // Windmill setup
        windmill.setPower(0);

        windmill.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE);

        // Intake setup
        intake.setPower(0);

        intake.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE);

        // Pedro follower setup
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();

        telemetry.addLine("BIOBUZZ Blue 1 Initialized");
        telemetry.addData("Launcher target RPM", LAUNCHER_RPM);
        telemetry.addData(
                "Launcher ticks/sec",
                LAUNCHER_TICKS_PER_SECOND);
        telemetry.addData("Windmill activations", WINDMILL_CYCLES);
        telemetry.addLine("Intake: OFF");
        telemetry.update();
    }

    // ============================================
    // START
    // ============================================

    @Override
    public void start() {
        schedule(autoRoutine());
    }

    // ============================================
    // MAIN LOOP
    // ============================================

    @Override
    public void loop() {

        follower.update();
        Scheduler.execute();

        // Safety: intake remains OFF
        intake.setPower(0);

        telemetry.addData(
                "X", follower.pose().x());

        telemetry.addData(
                "Y", follower.pose().y());

        telemetry.addData(
                "Heading",
                Math.toDegrees(follower.pose().heading()));

        telemetry.addData(
                "Launcher target RPM",
                LAUNCHER_RPM);

        telemetry.addData(
                "Launcher measured ticks/sec",
                launcher.getVelocity());

        telemetry.addData(
                "Windmill power",
                windmill.getPower());

        telemetry.addData("Intake", "OFF");

        telemetry.update();
    }

    // ============================================
    // STOP / SAFETY
    // ============================================

    @Override
    public void stop() {

        Scheduler.reset();

        launcher.setVelocity(0);
        windmill.setPower(0);
        intake.setPower(0);

        if (follower != null) {
            follower.stop();
        }
    }
}