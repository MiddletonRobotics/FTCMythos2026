package org.firstinspires.ftc.teamcode.commands;


import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

public class FollowTrajectoryCommand extends CommandBase {

    private final Drivetrain drivetrain;
    private final PathChain path;
    private boolean holdEnd;
    private double maxPower = 1.0;
    private double completionThreshold = 0.975;

    public FollowTrajectoryCommand(Drivetrain drivetrain, PathChain path) {
        this(drivetrain, path, true);
    }

    public FollowTrajectoryCommand(Drivetrain drivetrain, PathChain path, boolean holdEnd) {
        this(drivetrain, path, holdEnd, 1.0);
    }

    public FollowTrajectoryCommand(Drivetrain drivetrain, PathChain path, double maxPower) {
        this(drivetrain, path, true, maxPower);
    }

    public FollowTrajectoryCommand(Drivetrain drivetrain, PathChain path, boolean holdEnd, double maxPower) {
        this.drivetrain = drivetrain;
        this.path = path;
        this.holdEnd = holdEnd;
        this.maxPower = maxPower;
    }

    public FollowTrajectoryCommand(Drivetrain drivetrain, PathChain path, boolean holdEnd, double maxPower, double completionThreshold) {
        this.drivetrain = drivetrain;
        this.path = path;
        this.holdEnd = holdEnd;
        this.maxPower = maxPower;
        this.completionThreshold = completionThreshold;
    }

    public FollowTrajectoryCommand(Drivetrain drivetrain, Path path) {
        this(drivetrain, path, true);
    }

    public FollowTrajectoryCommand(Drivetrain drivetrain, Path path, boolean holdEnd) {
        this(drivetrain, path, holdEnd, 1.0);
    }

    public FollowTrajectoryCommand(Drivetrain drivetrain, Path path, double maxPower) {
        this(drivetrain, path, true, maxPower);
    }

    public FollowTrajectoryCommand(Drivetrain drivetrain, Path path, boolean holdEnd, double maxPower) {
        this.drivetrain = drivetrain;
        this.path = new PathChain(path);
        this.holdEnd = holdEnd;
        this.maxPower = maxPower;
    }

    public FollowTrajectoryCommand(Drivetrain drivetrain, Path path, boolean holdEnd, double maxPower, double completionThreshold) {
        this.drivetrain = drivetrain;
        this.path = new PathChain(path);
        this.holdEnd = holdEnd;
        this.maxPower = maxPower;
        this.completionThreshold = completionThreshold;
    }


    /**
     * Decides whether or not to make the robot maintain its position once the path ends.
     *
     * @param holdEnd If the robot should maintain its ending position
     * @return This command for compatibility in command groups
     */
    public FollowTrajectoryCommand setHoldEnd(boolean holdEnd) {
        this.holdEnd = holdEnd;
        return this;
    }

    /**
     * Decides whether or not to make the robot maintain its position once the path ends.
     *
     * @param maxPower If the robot should maintain its ending position
     * @return This command for compatibility in command groups
     */
    public FollowTrajectoryCommand setMaxPower(double maxPower) {
        this.maxPower = maxPower;
        return this;
    }

    /**
     * Sets the threshold to stop the path once it surpasses that percent completion.
     *
     * @param completionThreshold Percentage to end the path
     * @return This command for compatibility in command groups
     */
    public FollowTrajectoryCommand setCompletionThreshold(double completionThreshold) {
        this.completionThreshold = completionThreshold;
        return this;
    }

    @Override
    public void initialize() {
        drivetrain.follower.setMaxPower(maxPower);
        drivetrain.follower.followPath(path, holdEnd);
    }

    @Override
    public boolean isFinished() {
        return !drivetrain.follower.isBusy();
    }
}