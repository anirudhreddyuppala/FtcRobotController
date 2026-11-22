
package org.firstinspires.ftc.teamcode.pedro;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

@Autonomous(name = "BB_Pedro_Auto_long", group = "BIOBUZZ")

public class BBAutoLong extends OpMode {

    // ============================================
    // HARDWARE
    // ============================================

    private Follower follower;

    private DcMotorEx launcher;
    private DcMotor windmill;
    private DcMotor intake;

    // ============================================
    // POSES (DEGREES)
    // ============================================

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start =
            poseFactory.of(82.8452, 132.8733, 90);

    private final Pose path1 =
            poseFactory.of(83.0311, 124.5714, 90);

    private final Pose point2Start =
            poseFactory.of(83.0311, 124.5714, 90);

    private final Pose point2 =
            poseFactory.of(115.7729, 105.5049, 90);

    private final Pose point3Start =
            poseFactory.of(115.7729, 105.5049, 90);

    private final Pose point3 =
            poseFactory.of(126.3784, 49.6419, 90);

    // ============================================
    // LAUNCHER CONFIGURATION
    // ============================================

    private static final double LAUNCHER_TARGET_RPM = 620.0;

    private static final double LAUNCHER_MIN_RPM = 560.0;

    /*
     * IMPORTANT:
     *
     * Set these values for your actual launcher motor.
     *
     * ENCODER_TICKS_PER_MOTOR_REV:
     * Actual encoder counts per motor shaft revolution.
     *
     * MOTOR_REVS_PER_LAUNCHER_REV:
     * Number of motor revolutions required for one
     * launcher wheel revolution.
     *
     * For a direct-drive motor with a 28-tick encoder:
     * 28 * 1 = 28 ticks per launcher revolution.
     *
     * Do not assume these values without checking
     * your actual motor and gearbox.
     */

    private static final double ENCODER_TICKS_PER_MOTOR_REV = 28.0;

    private static final double MOTOR_REVS_PER_LAUNCHER_REV = 1.0;

    private static final double TICKS_PER_LAUNCHER_REV =
            ENCODER_TICKS_PER_MOTOR_REV *
                    MOTOR_REVS_PER_LAUNCHER_REV;

    // Convert RPM to encoder ticks per second

    private static final double LAUNCHER_TARGET_VELOCITY =
            LAUNCHER_TARGET_RPM * TICKS_PER_LAUNCHER_REV / 60.0;

    private static final double LAUNCHER_MIN_VELOCITY =
            LAUNCHER_MIN_RPM * TICKS_PER_LAUNCHER_REV / 60.0;

    // PIDF values from your earlier TeleOp code.
    // These must be tuned for the actual motor.

    private static final double LAUNCHER_P = 40.0;
    private static final double LAUNCHER_I = 0.0;
    private static final double LAUNCHER_D = 0.0;
    private static final double LAUNCHER_F = 12.5;

    // ============================================
    // WINDMILL CONFIGURATION
    // ============================================

    private static final double WINDMILL_POWER = 0.2;

    private static final int WINDMILL_CYCLES = 6;

    private static final long SPINUP_TIME_MS = 3500;

    private static final long WINDMILL_INTERVAL_MS = 1800;

    private static final long WINDMILL_ON_MS = 600;

    // ============================================
    // MECHANISM STATE
    // ============================================

    private long mechanismStartTime;

    private boolean launcherReady = false;

    private int completedWindmillCycles = 0;

    private double actualLauncherRPM = 0;

    private double actualLauncherVelocity = 0;

    // ============================================
    // PATHS
    // ============================================

    private Path path1CommandPath() {

        return line(start, path1).linear(start, path1);
    }

    private Path path2CommandPath() {

        return line(point2Start, point2)
                .linear(point2Start, point2);
    }

    private Path path3CommandPath() {

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

                    launcherReady = false;
                    completedWindmillCycles = 0;

                    // Intake OFF
                    intake.setPower(0);

                    // Windmill OFF
                    windmill.setPower(0);

                    // Start launcher at target velocity
                    launcher.setVelocity(
                            LAUNCHER_TARGET_VELOCITY
                    );
                })

                .setExecute(() -> {

                    // Intake remains OFF
                    intake.setPower(0);

                    // Maintain launcher target velocity
                    launcher.setVelocity(
                            LAUNCHER_TARGET_VELOCITY
                    );

                    // Read actual encoder velocity
                    actualLauncherVelocity =
                            launcher.getVelocity();

                    // Convert actual velocity to RPM
                    actualLauncherRPM =
                            actualLauncherVelocity *
                                    60.0 / TICKS_PER_LAUNCHER_REV;

                    long elapsedMs =
                            (System.nanoTime() - mechanismStartTime)
                                    / 1_000_000L;

                    // =================================
                    // SPIN-UP PERIOD
                    // =================================

                    if (elapsedMs < SPINUP_TIME_MS) {

                        windmill.setPower(0);
                        launcherReady = false;
                        return;
                    }

                    // =================================
                    // CHECK LAUNCHER SPEED
                    // =================================

                    launcherReady =
                            actualLauncherVelocity >=
                                    LAUNCHER_MIN_VELOCITY;

                    /*if (!launcherReady) {

                        windmill.setPower(0);
                        return;
                    }*/

                    // =================================
                    // WINDMILL SEQUENCE
                    // =================================

                    long windmillElapsed =
                            elapsedMs - SPINUP_TIME_MS;

                    long cycle =
                            windmillElapsed / WINDMILL_INTERVAL_MS;

                    long cycleTime =
                            windmillElapsed % WINDMILL_INTERVAL_MS;

                    // Activate windmill once per interval
                    if (cycle < WINDMILL_CYCLES &&
                            cycleTime < WINDMILL_ON_MS) {

                        windmill.setPower(WINDMILL_POWER);

                        completedWindmillCycles =
                                (int) cycle + 1;

                    } else {

                        windmill.setPower(0);
                    }
                })

                .setDone(() -> {

                    long elapsedMs =
                            (System.nanoTime() - mechanismStartTime)
                                    / 1_000_000L;

                    return elapsedMs >=
                            SPINUP_TIME_MS +
                                    WINDMILL_CYCLES * WINDMILL_INTERVAL_MS;
                })

                .setEnd(endCondition -> {

                    windmill.setPower(0);

                    launcher.setVelocity(0);

                    intake.setPower(0);
                })

                .requiring(launcher, windmill, intake);
    }

    // ============================================
    // COMPLETE AUTONOMOUS ROUTINE
    // ============================================

    private Command autoRoutine() {

        return sequential(

                // Path 1
                follow(follower, path1CommandPath()),

                // Launcher ON and windmill sequence
                // Intake remains OFF
                shootAndRotateWindmill(),

                // Path 2
                follow(follower, path2CommandPath()),

                // Path 3
                follow(follower, path3CommandPath())
        );
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
                DcMotor.class, "windmill_motor");

        intake = hardwareMap.get(
                DcMotor.class, "intake");

        // =================================
        // LAUNCHER INITIALIZATION
        // =================================

        launcher.setPower(0);

        launcher.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.FLOAT
        );

        launcher.setMode(
                DcMotor.RunMode.RUN_USING_ENCODER
        );

        // Configure velocity PIDF
        launcher.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(
                        LAUNCHER_P,
                        LAUNCHER_I,
                        LAUNCHER_D,
                        LAUNCHER_F
                )
        );

        // =================================
        // WINDMILL INITIALIZATION
        // =================================

        windmill.setPower(0);

        windmill.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        // =================================
        // INTAKE INITIALIZATION
        // =================================

        intake.setPower(0);

        intake.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        // =================================
        // PEDRO INITIALIZATION
        // =================================

        follower = Constants.create(hardwareMap);

        follower.setPose(start);

        follower.update();

        // =================================
        // TELEMETRY
        // =================================

        telemetry.addLine("BIOBUZZ Blue 1 Initialized");

        telemetry.addData(
                "Target RPM",
                LAUNCHER_TARGET_RPM
        );

        telemetry.addData(
                "Minimum RPM",
                LAUNCHER_MIN_RPM
        );

        telemetry.addData(
                "Ticks per launcher revolution",
                TICKS_PER_LAUNCHER_REV
        );

        telemetry.addData(
                "Target encoder velocity",
                LAUNCHER_TARGET_VELOCITY
        );

        telemetry.addData(
                "PIDF",
                "P %.2f I %.2f D %.2f F %.2f",
                LAUNCHER_P,
                LAUNCHER_I,
                LAUNCHER_D,
                LAUNCHER_F
        );

        telemetry.addLine("Intake: OFF");

        telemetry.update();
    }

    // ============================================
    // START AUTONOMOUS
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

        // Safety: intake always OFF
        intake.setPower(0);

        // Read launcher values
        actualLauncherVelocity = launcher.getVelocity();

        actualLauncherRPM =
                actualLauncherVelocity *
                        60.0 / TICKS_PER_LAUNCHER_REV;

        // =================================
        // ROBOT TELEMETRY
        // =================================

        telemetry.addData(
                "Robot X",
                follower.pose().x()
        );

        telemetry.addData(
                "Robot Y",
                follower.pose().y()
        );

        telemetry.addData(
                "Heading",
                Math.toDegrees(follower.pose().heading())
        );

        // =================================
        // LAUNCHER TELEMETRY
        // =================================

        telemetry.addLine("----- LAUNCHER -----");

        telemetry.addData(
                "Target RPM",
                "%.1f",
                LAUNCHER_TARGET_RPM
        );

        telemetry.addData(
                "Actual RPM",
                "%.1f",
                actualLauncherRPM
        );

        telemetry.addData(
                "Target Velocity",
                "%.2f",
                LAUNCHER_TARGET_VELOCITY
        );

        telemetry.addData(
                "Actual Velocity",
                "%.2f",
                actualLauncherVelocity
        );

        telemetry.addData(
                "Motor Power",
                "%.3f",
                launcher.getPower()
        );

        telemetry.addData(
                "Encoder Position",
                launcher.getCurrentPosition()
        );

        telemetry.addData(
                "Launcher Ready",
                launcherReady
        );

        // =================================
        // WINDMILL TELEMETRY
        // =================================

        telemetry.addLine("----- WINDMILL -----");

        telemetry.addData(
                "Windmill Power",
                windmill.getPower()
        );

        telemetry.addData(
                "Completed Cycles",
                completedWindmillCycles
        );

        telemetry.addData(
                "Intake",
                "OFF"
        );

        telemetry.update();
    }

    // ============================================
    // STOP / SAFETY
    // ============================================

    @Override
    public void stop() {

        Scheduler.reset();

        launcher.setVelocity(0);

        launcher.setPower(0);

        windmill.setPower(0);

        intake.setPower(0);

        if (follower != null) {
            follower.stop();
        }
    }
}
