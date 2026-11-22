package org.firstinspires.ftc.teamcode.pedro;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import java.util.List;

@TeleOp(name = "Limelight Hive Auto-Aim", group = "BIOBUZZ")
public class LimelightTesting extends LinearOpMode {

    // Drivetrain Motors
    private DcMotor frontLeft, frontRight, backLeft, backRight;

    // Limelight 3A Camera
    private Limelight3A limelight;

    // ============================================
    // TUNING & PIPELINE SETTINGS
    // ============================================
    private static final int APRILTAG_PIPELINE = 0;
    private static final double TURN_KP = 0.022;           // Proportional turn speed
    private static final double MIN_TURN_POWER = 0.08;     // Minimum power to overcome friction
    private static final double MAX_TURN_POWER = 0.55;     // Max rotation speed during auto-aim
    private static final double AIM_TOLERANCE_DEG = 1.5;   // Locked on within 1.5 degrees

    // ============================================
    // TRACKING STATE VARIABLES
    // ============================================
    private boolean hasTag = false;
    private int currentTagId = -1;
    private double tx = 0.0;
    private double ty = 0.0;

    @Override
    public void runOpMode() {
        // 1. Map Drivetrain Motors
        frontLeft  = hardwareMap.get(DcMotor.class, "left_front_drive");
        frontRight = hardwareMap.get(DcMotor.class, "right_front_drive");
        backLeft   = hardwareMap.get(DcMotor.class, "left_back_drive");
        backRight  = hardwareMap.get(DcMotor.class, "right_back_drive");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // 2. Initialize Limelight 3A
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(APRILTAG_PIPELINE);
        limelight.start();

        telemetry.addLine("Limelight Hive Aim Ready. Press Play.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            updateLimelight();

            // Hold 'A' (or Cross) on Gamepad 1 to Auto-Aim at the Hive
            if (gamepad1.a) {
                aimAtHive();
            } else {
                // Normal Mecanum TeleOp driving
                double drive  = -gamepad1.left_stick_y;
                double strafe =  gamepad1.left_stick_x;
                double turn   =  gamepad1.right_stick_x;
                driveMecanum(drive, strafe, turn);
            }

            displayHiveTelemetry();
        }

        limelight.stop();
    }

    /**
     * Reads AprilTags and prioritizes Scoring (Hive) tags (30-33, 42-45) over Audience tags.
     */
    private void updateLimelight() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            if (fiducials != null && !fiducials.isEmpty()) {
                LLResultTypes.FiducialResult bestTag = null;

                // First pass: look specifically for a Hive Scoring Tag (30-33 or 42-45)
                for (LLResultTypes.FiducialResult tag : fiducials) {
                    int id = tag.getFiducialId();
                    if (isHiveScoringTag(id)) {
                        bestTag = tag;
                        break;
                    }
                }

                // Fallback: if no Scoring Tag is visible, report whatever tag is seen
                if (bestTag == null) {
                    bestTag = fiducials.get(0);
                }

                hasTag = true;
                currentTagId = bestTag.getFiducialId();
                tx = bestTag.getTargetXDegrees();
                ty = bestTag.getTargetYDegrees();
                return;
            }
        }
        hasTag = false;
        currentTagId = -1;
        tx = 0.0;
        ty = 0.0;
    }

    /**
     * Checks if the AprilTag ID belongs to Red Scoring (30-33) or Blue Scoring (42-45).
     */
    private boolean isHiveScoringTag(int tagId) {
        return (tagId >= 30 && tagId <= 33) || (tagId >= 42 && tagId <= 45);
    }

    /**
     * Turns the robot to center horizontal error (tx) on the detected Hive Scoring Tag.
     */
    private void aimAtHive() {
        // Only auto-turn toward actual Hive Scoring Tags (ignores Audience tags)
        if (!hasTag || !isHiveScoringTag(currentTagId) || Math.abs(tx) <= AIM_TOLERANCE_DEG) {
            driveMecanum(0, 0, 0);
            return;
        }

        double turnPower = tx * TURN_KP;

        if (turnPower > 0) {
            turnPower = Math.max(turnPower, MIN_TURN_POWER);
        } else {
            turnPower = Math.min(turnPower, -MIN_TURN_POWER);
        }

        turnPower = Range.clip(turnPower, -MAX_TURN_POWER, MAX_TURN_POWER);

        double drive  = -gamepad1.left_stick_y;
        double strafe =  gamepad1.left_stick_x;

        driveMecanum(drive, strafe, turnPower);
    }

    /**
     * Standard Mecanum wheel power mixing with normalization.
     */
    private void driveMecanum(double drive, double strafe, double turn) {
        double fl = drive + strafe + turn;
        double fr = drive - strafe - turn;
        double bl = drive - strafe + turn;
        double br = drive + strafe - turn;

        double max = Math.max(1.0, Math.max(Math.abs(fl), Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br)))));

        frontLeft.setPower(fl / max);
        frontRight.setPower(fr / max);
        backLeft.setPower(bl / max);
        backRight.setPower(br / max);
    }

    /**
     * Translates all 16 field tag IDs from your diagram into clear Driver Station labels.
     */
    private String identifyTag(int tagId) {
        switch (tagId) {
            // Red Scoring Tags (Left to Right on diagram: 33, 32, 31, 30)
            case 33: return "Red Scoring Hive - Far Left (ID 33)";
            case 32: return "Red Scoring Hive - Mid Left (ID 32)";
            case 31: return "Red Scoring Hive - Mid Right (ID 31)";
            case 30: return "Red Scoring Hive - Inner/Right (ID 30)";

            // Blue Scoring Tags (Left to Right on diagram: 45, 44, 43, 42)
            case 45: return "Blue Scoring Hive - Inner/Left (ID 45)";
            case 44: return "Blue Scoring Hive - Mid Left (ID 44)";
            case 43: return "Blue Scoring Hive - Mid Right (ID 43)";
            case 42: return "Blue Scoring Hive - Far Right (ID 42)";

            // Red Audience Tags (Left to Right on diagram: 34, 35, 36, 37)
            case 34: return "Red Audience Wall - Far Left (ID 34)";
            case 35: return "Red Audience Wall - Mid Left (ID 35)";
            case 36: return "Red Audience Wall - Mid Right (ID 36)";
            case 37: return "Red Audience Wall - Inner/Right (ID 37)";

            // Blue Audience Tags (Left to Right on diagram: 38, 39, 40, 41)
            case 38: return "Blue Audience Wall - Inner/Left (ID 38)";
            case 39: return "Blue Audience Wall - Mid Left (ID 39)";
            case 40: return "Blue Audience Wall - Mid Right (ID 40)";
            case 41: return "Blue Audience Wall - Far Right (ID 41)";

            default: return "Unknown Tag (ID " + tagId + ")";
        }
    }

    /**
     * Updates Driver Station telemetry with the exact tag name and aim status.
     */
    private void displayHiveTelemetry() {
        telemetry.addLine("=== LIMELIGHT HIVE TRACKER ===");
        if (hasTag) {
            boolean isScoring = isHiveScoringTag(currentTagId);
            boolean isLocked = isScoring && Math.abs(tx) <= AIM_TOLERANCE_DEG;

            if (!isScoring) {
                telemetry.addData("Aim Status", "AUDIENCE TAG SEEN (Not a Hive)");
            } else {
                telemetry.addData("Aim Status", isLocked ? "LOCKED ON HIVE" : "TURNING TO HIVE...");
            }

            telemetry.addData("Looking At", identifyTag(currentTagId));
            telemetry.addData("Horizontal Error (tx)", "%.2f deg", tx);
            telemetry.addData("Vertical Angle (ty)", "%.2f deg", ty);
        } else {
            telemetry.addData("Aim Status", "SEARCHING (No Tag Visible)");
            telemetry.addData("Looking At", "None");
        }
        telemetry.update();
    }
}
