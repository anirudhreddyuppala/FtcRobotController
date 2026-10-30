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
    private Follower follower; // Add this
    private final PoseFactory poseFactory = PoseFactory.degrees(); // the factory we added before, we are still at the top of the class
    // our poses
    /*private final Pose start = poseFactory.of(82.8452, 132.8733, 90);
    private final Pose goToLaunch = poseFactory.of(83.1916, 114.9767, 90);
    private final Pose turnAfterLaunch = poseFactory.of(115.5998, 113.1255, -3.2694);
    private final Pose goToPark = poseFactory.of(124.284, 43.825, -82.8574);*/


    private final Pose start = poseFactory.of(82.8452, 132.8733, 90);
    private final Pose path1 = poseFactory.of(83.1916, 114.9767, 90);
    private final Pose point2Start = poseFactory.of(83.1916, 114.9767, 90);
    private final Pose point2 = poseFactory.of(115.5998, 113.1255, 90);
    private final Pose point3Start = poseFactory.of(115.5998, 113.1255, 90);
    private final Pose point3 = poseFactory.of(124.284, 43.825, 90);

    // variables for launcher and windmill

    private DcMotorEx launcher = null;
    private DcMotor windmillMotor = null;
    public final int LAUNCHER_TARGET_VELOCITY = 320; //2678 RPM
    public final int LAUNCHER_MIN_VELOCITY = 190; //2571 RPM
    boolean windmillMotorPower;

    public Path path1() {
        return line(start, path1).linear(start, path1);
    }

    public Path path2() {
        return line(point2Start, point2).linear(point2Start, point2);
    }

    public Path path3() {
        return line(point3Start, point3).linear(point3Start, point3);
    }
   /* private Path goToLaunch() {
        return line(start, goToLaunch).linear(start, goToLaunch);
    }

    private Path turnAfterLaunch() {
        return line(goToLaunch, turnAfterLaunch).tangent();
    }

    private Path goToPark() {
        return line(turnAfterLaunch, goToPark).tangent();
    }*/


    private Command autoRoutine() {
        return sequential(
                follow(follower, path1()),
                waitMs(150),
                instant(() -> launcher.setVelocity(LAUNCHER_TARGET_VELOCITY)),
                waitMs(150),
                follow(follower, path2()),
                // Add mechanism commands here.
                follow(follower, path3())
        );
    }

    @Override
    public void init() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        windmillMotor = hardwareMap.get(DcMotor.class, "windmill_motor");
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(40, 0, 0, 12.5));
    }



    @Override
    public void start() {
        schedule(autoRoutine());
    }


    @Override
    public void loop() {
        follower.update();
        Scheduler.execute();

        // add your other methods needed in the loop here
        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData("Heading", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Follower Mode", follower.mode());
        telemetry.update();
    }
}

