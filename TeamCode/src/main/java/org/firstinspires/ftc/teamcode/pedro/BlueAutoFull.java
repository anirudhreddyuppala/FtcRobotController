
package org.firstinspires.ftc.teamcode.pedro;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.api.Paths;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.commands.Commands.conditional;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;


@Autonomous()
public class BlueAutoFull extends OpMode {

    // ------------------------------------------------
    // HARDWARE
    // ------------------------------------------------

    private Follower follower;

    private DcMotorEx launcher;
    private DcMotor windmill;
    private DcMotor intake;

    // ------------------------------------------------
    // POSES (degrees)
    // ------------------------------------------------

    private final PoseFactory poseFactory = PoseFactory.degrees();
    private final Pose start = poseFactory.of(82.8452, 132.8733, 90);
    private final Pose path1 = poseFactory.of(82.8452, 117.7479, 90);
    private final Pose point2Start = poseFactory.of(82.8452, 117.7479, 90);
    private final Pose point2 = poseFactory.of(116.4657, 108.6224, 90);
    private final Pose point3Start = poseFactory.of(116.4657, 108.6224, 90);
    private final Pose point3 = poseFactory.of(126.1891, 47.2889, 90);




    // ------------------------------------------------
    // LAUNCHER AND WINDMILL SETTINGS
    // ------------------------------------------------

    // Example value. Tune this for your launcher.
    private static final double LAUNCHER_VELOCITY = 350;

    private static final double WINDMILL_POWER = 0.60;

    // Four windmill activations
    private static final int WINDMILL_CYCLES = 6;

    // Launcher spin-up time
    private static final long SPINUP_TIME_MS = 2000;

    // Time between windmill activations
    private static final long WINDMILL_INTERVAL_MS = 1800;

    // Duration of each windmill activation
    private static final long WINDMILL_ON_MS = 200;

    // Timer for the mechanism sequence
    private long mechanismStartTime;

    // ------------------------------------------------
    // PATHS
    // ------------------------------------------------

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

    // ------------------------------------------------
    // LAUNCHER + FOUR WINDMILL ACTIVATIONS
    // ------------------------------------------------

    private Command shootAndRotateWindmill() {

        return Command.build()

                .setStart(() -> {

                    mechanismStartTime = System.nanoTime();

                    // Intake must remain OFF
                    intake.setPower(0);

                    // Start launcher
                    launcher.setVelocity(LAUNCHER_VELOCITY);

                    // Windmill starts OFF
                    windmill.setPower(0);
                })

                .setExecute(() -> {

                    // Keep intake OFF throughout
                    intake.setPower(0);

                    // Keep launcher running
                    launcher.setVelocity(LAUNCHER_VELOCITY);

                    long elapsedMs =
                            (System.nanoTime() - mechanismStartTime)
                                    / 1_000_000L;

                    // Wait for launcher spin-up
                    if (elapsedMs < SPINUP_TIME_MS) {
                        windmill.setPower(0);
                        return;
                    }

                    long windmillElapsed =
                            elapsedMs - SPINUP_TIME_MS;

                    // Determine current windmill cycle
                    long cycle =
                            windmillElapsed / WINDMILL_INTERVAL_MS;

                    long cycleTime =
                            windmillElapsed % WINDMILL_INTERVAL_MS;

                    // Activate windmill once per interval
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
                                    + WINDMILL_CYCLES * WINDMILL_INTERVAL_MS;
                })

                .setEnd(endCondition -> {

                    // Stop all mechanisms after four activations
                    windmill.setPower(0);
                    launcher.setVelocity(0);
                    intake.setPower(0);
                })

                .requiring(launcher, windmill, intake);
    }

    // ------------------------------------------------
    // COMPLETE AUTONOMOUS ROUTINE
    // ------------------------------------------------

    private Command autoRoutine() {

        return sequential(

                // Path 1
                follow(follower, path1CommandPath()),

                // Launcher ON, windmill activates four times.
                // Intake remains OFF.
                shootAndRotateWindmill(),

                // Path 2
                follow(follower, path2CommandPath()),

                // Path 3
                follow(follower, path3CommandPath())
        );
    }

    // ------------------------------------------------
    // INITIALIZATION
    // ------------------------------------------------

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

        // Safe mechanism initialization
        launcher.setPower(0);
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        windmill.setPower(0);
        windmill.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake.setPower(0);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize Pedro follower
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();

        telemetry.addLine("BIOBUZZ Blue 1 initialized");
        telemetry.addData("Launcher velocity", LAUNCHER_VELOCITY);
        telemetry.addData("Windmill activations", WINDMILL_CYCLES);
        telemetry.addLine("Intake: OFF");
        telemetry.update();
    }

    // ------------------------------------------------
    // START AUTONOMOUS
    // ------------------------------------------------

    @Override
    public void start() {

        schedule(autoRoutine());
    }

    // ------------------------------------------------
    // MAIN LOOP
    // ------------------------------------------------

    @Override
    public void loop() {

        follower.update();
        Scheduler.execute();

        // Safety: intake stays OFF
        intake.setPower(0);

        telemetry.addData("Robot X", follower.pose().x());
        telemetry.addData("Robot Y", follower.pose().y());
        telemetry.addData(
                "Heading",
                Math.toDegrees(follower.pose().heading()));

        telemetry.addData(
                "Launcher velocity",
                launcher.getVelocity());

        telemetry.addData(
                "Windmill power",
                windmill.getPower());

        telemetry.addData("Intake", "OFF");

        telemetry.update();
    }

    // ------------------------------------------------
    // STOP / SAFETY
    // ------------------------------------------------

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