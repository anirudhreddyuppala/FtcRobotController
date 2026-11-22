package org.firstinspires.ftc.teamcode.pedro;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;

import java.util.Arrays;
import java.util.Collections;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.race;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static com.pedropathing.ivy.pedro.PedroCommands.hold;

@Autonomous(name = "BB_Pedro_Auto_2Tips", group = "BIOBUZZ")
public class BB_AutoTwoTips extends OpMode {

    // ============================================
    // ALLIANCE & APRILTAG CONFIGURATION (PER FIELD DIAGRAM)
    // ============================================
    public enum Alliance {
        BLUE, RED
    }

    // Default to BLUE (Switchable in init_loop via Gamepad 1: X = Blue, B = Red)
    private Alliance alliance = Alliance.BLUE;

    // Red Alliance Tags: 30-33 (Scoring), 34-37 (Audience)
    private static final Set<Integer> RED_SCORING_TAGS =
            Collections.unmodifiableSet(new HashSet<>(Arrays.asList(30, 31, 32, 33)));
    private static final Set<Integer> RED_AUDIENCE_TAGS =
            Collections.unmodifiableSet(new HashSet<>(Arrays.asList(34, 35, 36, 37)));

    // Blue Alliance Tags: 42-45 (Scoring), 38-41 (Audience)
    private static final Set<Integer> BLUE_SCORING_TAGS =
            Collections.unmodifiableSet(new HashSet<>(Arrays.asList(42, 43, 44, 45)));
    private static final Set<Integer> BLUE_AUDIENCE_TAGS =
            Collections.unmodifiableSet(new HashSet<>(Arrays.asList(38, 39, 40, 41)));

    // ============================================
    // HARDWARE
    // ============================================
    private Follower follower;
    private Limelight3A limelight;
    private DcMotorEx launcher;
    private DcMotor windmill;
    private DcMotor intake;

    // ============================================
    // LIMELIGHT SETTINGS
    // ============================================
    private static final int APRILTAG_PIPELINE_INDEX = 0;

    // Proportional turning gain & tolerance for Limelight AprilTag alignment
    private static final double AIM_KP = 0.65;
    private static final double AIM_TOLERANCE_DEG = 1.5;
    private static final long AIM_TIMEOUT_MS = 1200;

    private int detectedTagId = -1;
    private String detectedTagZone = "NONE";
    private double tagTx = 0.0;
    private double tagTy = 0.0;
    private double tagTa = 0.0;
    private boolean hasTagTarget = false;

    // Hive Tip Tracking
    private int tagIdBeforeShot = -1;
    private int confirmedHiveTips = 0;

    // ============================================
    // POSES (DEGREES) - BLUE DEFAULT (MIRROR X FOR RED IF NEEDED)
    // ============================================
    private final PoseFactory poseFactory = PoseFactory.degrees();

    private Pose start;
    private Pose scorePose;
    private Pose pickup1;
    private Pose pickup2;

    // ============================================
    // LAUNCHER CONFIGURATION
    // ============================================
    private static final double LAUNCHER_TARGET_RPM = 620.0;
    private static final double LAUNCHER_MIN_RPM = 560.0;

    private static final double ENCODER_TICKS_PER_MOTOR_REV = 28.0;
    private static final double MOTOR_REVS_PER_LAUNCHER_REV = 1.0;
    private static final double TICKS_PER_LAUNCHER_REV =
            ENCODER_TICKS_PER_MOTOR_REV * MOTOR_REVS_PER_LAUNCHER_REV;

    private static final double LAUNCHER_TARGET_VELOCITY =
            LAUNCHER_TARGET_RPM * TICKS_PER_LAUNCHER_REV / 60.0;
    private static final double LAUNCHER_MIN_VELOCITY =
            LAUNCHER_MIN_RPM * TICKS_PER_LAUNCHER_REV / 60.0;

    private static final double LAUNCHER_P = 40.0;
    private static final double LAUNCHER_I = 0.0;
    private static final double LAUNCHER_D = 0.0;
    private static final double LAUNCHER_F = 12.5;

    // ============================================
    // WINDMILL & INTAKE CONFIGURATION (30s PACED)
    // ============================================
    private static final double INTAKE_POWER = 0.85;
    private static final double WINDMILL_POWER = 0.25;

    private static final int WINDMILL_CYCLES = 4;
    private static final long SPINUP_TIME_MS = 1200;
    private static final long WINDMILL_INTERVAL_MS = 850;
    private static final long WINDMILL_ON_MS = 450;

    // ============================================
    // MECHANISM STATE
    // ============================================
    private long mechanismStartTime;
    private long aimStartTime;
    private double targetAimHeadingRad;
    private boolean launcherReady = false;
    private int completedWindmillCycles = 0;
    private double actualLauncherRPM = 0;
    private double actualLauncherVelocity = 0;

    // ============================================
    // ALLIANCE POSE SETUP
    // ============================================
    private void configureAlliancePoses() {
        if (alliance == Alliance.BLUE) {
            start     = poseFactory.of(82.8452, 132.8733, 90);
            scorePose = poseFactory.of(83.0311, 124.5714, 90);
            pickup1   = poseFactory.of(115.7729, 105.5049, 45);
            pickup2   = poseFactory.of(126.3784, 75.0000, 90);
        } else {
            // Mirrored X across a 144-inch field for Red Alliance
            start     = poseFactory.of(144.0 - 82.8452, 132.8733, 90);
            scorePose = poseFactory.of(144.0 - 83.0311, 124.5714, 90);
            pickup1   = poseFactory.of(144.0 - 115.7729, 105.5049, 135);
            pickup2   = poseFactory.of(144.0 - 126.3784, 75.0000, 90);
        }
        if (follower != null) {
            follower.setPose(start);
        }
    }

    // ============================================
    // PATHS
    // ============================================
    private Path toScoreFromStart() {
        return line(start, scorePose);
    }

    private Path toPickup1() {
        return line(scorePose, pickup1);
    }

    private Path toPickup2() {
        return line(pickup1, pickup2);
    }

    private Path returnToScore() {
        return line(pickup2, scorePose);
    }

    // ============================================
    // LIMELIGHT APRILTAG READING (ALLIANCE FILTERED)
    // ============================================
    private Set<Integer> getScoringTags() {
        return (alliance == Alliance.BLUE) ? BLUE_SCORING_TAGS : RED_SCORING_TAGS;
    }

    private Set<Integer> getAudienceTags() {
        return (alliance == Alliance.BLUE) ? BLUE_AUDIENCE_TAGS : RED_AUDIENCE_TAGS;
    }

    private void updateLimelight() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            if (fiducials != null && !fiducials.isEmpty()) {
                Set<Integer> scoringTags = getScoringTags();
                Set<Integer> audienceTags = getAudienceTags();

                LLResultTypes.FiducialResult bestTag = null;
                String zone = "NONE";

                // 1. First priority: Alliance Scoring Tags (30-33 Red / 42-45 Blue)
                for (LLResultTypes.FiducialResult tag : fiducials) {
                    if (scoringTags.contains(tag.getFiducialId())) {
                        bestTag = tag;
                        zone = alliance.name() + "_SCORING";
                        break;
                    }
                }

                // 2. Second priority: Alliance Audience Tags (34-37 Red / 38-41 Blue)
                if (bestTag == null) {
                    for (LLResultTypes.FiducialResult tag : fiducials) {
                        if (audienceTags.contains(tag.getFiducialId())) {
                            bestTag = tag;
                            zone = alliance.name() + "_AUDIENCE";
                            break;
                        }
                    }
                }

                // Ignore opponent tags completely so the robot never aims at the wrong Hive
                if (bestTag != null) {
                    hasTagTarget = true;
                    detectedTagId = bestTag.getFiducialId();
                    detectedTagZone = zone;
                    tagTx = bestTag.getTargetXDegrees();
                    tagTy = bestTag.getTargetYDegrees();
                    tagTa = bestTag.getTargetArea();
                    return;
                }
            }
        }

        hasTagTarget = false;
        detectedTagId = -1;
        detectedTagZone = "NONE";
        tagTx = 0.0;
        tagTy = 0.0;
        tagTa = 0.0;
    }

    // ============================================
    // IVY COMMANDS
    // ============================================
    private Command preSpinLauncher() {
        return Command.build()
                .setStart(() -> launcher.setVelocity(LAUNCHER_TARGET_VELOCITY))
                .setExecute(() -> launcher.setVelocity(LAUNCHER_TARGET_VELOCITY))
                .setDone(() -> true);
    }

    private Command alignToHiveTag() {
        return Command.build()
                .setStart(() -> {
                    aimStartTime = System.nanoTime();
                    tagIdBeforeShot = -1; // Reset before each shot cycle
                    targetAimHeadingRad = follower.pose().heading();
                    launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
                })
                .setExecute(() -> {
                    launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
                    if (hasTagTarget) {
                        tagIdBeforeShot = detectedTagId;
                        double errorRad = Math.toRadians(tagTx);
                        targetAimHeadingRad = follower.pose().heading() - (errorRad * AIM_KP);
                    }
                    Pose aimPose = new Pose(
                            scorePose.x(),
                            scorePose.y(),
                            targetAimHeadingRad
                    );
                    follower.hold(aimPose);
                })
                .setDone(() -> {
                    long elapsedMs = (System.nanoTime() - aimStartTime) / 1_000_000L;
                    boolean aligned = hasTagTarget && Math.abs(tagTx) <= AIM_TOLERANCE_DEG;
                    return aligned || elapsedMs >= AIM_TIMEOUT_MS;
                })
                .setEnd(endCondition -> {
// Hold final aligned pose
                });
    }
    private Command shootVolleyAndVerifyTip() {
        return Command.build()
                .setStart(() -> {
                    mechanismStartTime = System.nanoTime();
                    launcherReady = false;
                    completedWindmillCycles = 0;
                    intake.setPower(0);
                    windmill.setPower(0);
                    launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
                })
                .setExecute(() -> {
                    intake.setPower(0);
                    launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
                    actualLauncherVelocity = launcher.getVelocity();
                    actualLauncherRPM = actualLauncherVelocity * 60.0 / TICKS_PER_LAUNCHER_REV;
                    long elapsedMs = (System.nanoTime() - mechanismStartTime) / 1_000_000L;
                    if (elapsedMs < SPINUP_TIME_MS && actualLauncherVelocity < LAUNCHER_MIN_VELOCITY) {
                        windmill.setPower(0);
                        launcherReady = false;
                        return;
                    }
                    launcherReady = actualLauncherVelocity >= LAUNCHER_MIN_VELOCITY;
                    long windmillElapsed = elapsedMs - SPINUP_TIME_MS;
                    if (windmillElapsed < 0) {
                        windmillElapsed = 0;
                    }
                    long cycle = windmillElapsed / WINDMILL_INTERVAL_MS;
                    long cycleTime = windmillElapsed % WINDMILL_INTERVAL_MS;
                    if (cycle < WINDMILL_CYCLES && cycleTime < WINDMILL_ON_MS) {
                        windmill.setPower(WINDMILL_POWER);
                        completedWindmillCycles = (int) cycle + 1;
                    } else {
                        windmill.setPower(0);
                    }
                })
                .setDone(() -> {
                    long elapsedMs = (System.nanoTime() - mechanismStartTime) / 1_000_000L;
                    return elapsedMs >= SPINUP_TIME_MS + (WINDMILL_CYCLES * WINDMILL_INTERVAL_MS);
                })
                .setEnd(endCondition -> {
                    windmill.setPower(0);
                    launcher.setVelocity(0);
                    intake.setPower(0);
                    if (tagIdBeforeShot != -1 && hasTagTarget && detectedTagId != tagIdBeforeShot) {
                        confirmedHiveTips++;
                    }
                })
                .requiring(launcher, windmill, intake);
    }
    private Command runIntake() {
        return Command.build()
                .setStart(() -> intake.setPower(INTAKE_POWER))
                .setExecute(() -> intake.setPower(INTAKE_POWER))
                .setDone(() -> false)
                .setEnd(endCondition -> intake.setPower(0))
                .requiring(intake);
    }
    // ============================================
// COMPLETE 30-SECOND 2-HIVE-TIP ROUTINE
// ============================================
    private Command autoRoutine() {
        return sequential(
// TIP #1: PRELOADS (4 POLLEN)
                parallel(
                        follow(follower, toScoreFromStart()),
                        preSpinLauncher()
                ),
                alignToHiveTag(),
                shootVolleyAndVerifyTip(),
// CYCLE 2: COLLECT FIELD POLLEN (race stops intake when paths finish)
                race(
                        sequential(
                                follow(follower, toPickup1()),
                                follow(follower, toPickup2())
                        ),
                        runIntake()
                ),
// TIP #2: RETURN, AIM, AND SCORE
                parallel(
                        follow(follower, returnToScore()),
                        preSpinLauncher()
                ),
                alignToHiveTag(),
                shootVolleyAndVerifyTip(),
                hold(follower, scorePose)
        );
    }
    // ============================================
// INITIALIZATION
// ============================================
    @Override
    public void init() {
        Scheduler.reset();
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        windmill = hardwareMap.get(DcMotor.class, "windmill_motor");
        intake = hardwareMap.get(DcMotor.class, "intake");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(APRILTAG_PIPELINE_INDEX);
        limelight.start();
        launcher.setPower(0);
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(LAUNCHER_P, LAUNCHER_I, LAUNCHER_D, LAUNCHER_F)
        );
        windmill.setPower(0);
        windmill.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setPower(0);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        follower = Constants.create(hardwareMap);
        configureAlliancePoses();
        follower.update();
        telemetry.addLine("BIOBUZZ 2-Hive-Tip Auto Initialized");
        telemetry.addLine("Press X (Blue) or B (Red) on Gamepad 1");
        telemetry.update();
    }
    // ============================================
// INIT LOOP (ALLIANCE SELECTION & TAG CHECK)
// ============================================
    @Override
    public void init_loop() {
        if (gamepad1.x && alliance != Alliance.BLUE) {
            alliance = Alliance.BLUE;
            configureAlliancePoses();
        } else if (gamepad1.b && alliance != Alliance.RED) {
            alliance = Alliance.RED;
            configureAlliancePoses();
        }
        updateLimelight();
        telemetry.addLine("=== ALLIANCE SELECT (Gamepad 1: X=BLUE, B=RED) ===");
        telemetry.addData("Selected Alliance", alliance.name());
        telemetry.addData("Valid Tags", (alliance == Alliance.BLUE) ? "42-45 (Score), 38-41 (Audience)" : "30-33 (Score), 34-37 (Audience)");
        telemetry.addLine("----- LIMELIGHT CHECK -----");
        telemetry.addData("Tag Visible", hasTagTarget);
        telemetry.addData("Detected Tag ID", detectedTagId);
        telemetry.addData("Detected Zone", detectedTagZone);
        telemetry.addData("Tag TX (deg)", "%.2f", tagTx);
        telemetry.update();
    }
    @Override
    public void start() {
        schedule(autoRoutine());
    }
    @Override
    public void loop() {
        follower.update();
        updateLimelight();
        Scheduler.execute();
        actualLauncherVelocity = launcher.getVelocity();
        actualLauncherRPM = actualLauncherVelocity * 60.0 / TICKS_PER_LAUNCHER_REV;
        telemetry.addData("Alliance", alliance.name());
        telemetry.addData("Robot X", "%.2f", follower.pose().x());
        telemetry.addData("Robot Y", "%.2f", follower.pose().y());
        telemetry.addData("Heading (deg)", "%.1f", Math.toDegrees(follower.pose().heading()));
        telemetry.addLine("----- LIMELIGHT & HIVE -----");
        telemetry.addData("Tag Visible", hasTagTarget);
        telemetry.addData("Detected Tag ID", detectedTagId);
        telemetry.addData("Tag Zone", detectedTagZone);
        telemetry.addData("Tag Before Shot", tagIdBeforeShot);
        telemetry.addData("Confirmed Hive Tips", confirmedHiveTips);
        telemetry.addData("Tag TX (deg)", "%.2f", tagTx);
        telemetry.addLine("----- MECHANISMS -----");
        telemetry.addData("Launcher RPM", "%.1f / %.1f", actualLauncherRPM, LAUNCHER_TARGET_RPM);
        telemetry.addData("Launcher Ready", launcherReady);
        telemetry.addData("Windmill Cycles", completedWindmillCycles);
        telemetry.addData("Intake Power", "%.2f", intake.getPower());
        telemetry.update();
    }
    @Override
    public void stop() {
        Scheduler.reset();
        launcher.setVelocity(0);
        launcher.setPower(0);
        windmill.setPower(0);
        intake.setPower(0);
        if (limelight != null) {
            limelight.stop();
        }
        if (follower != null) {
            follower.stop();
        }
    }
}