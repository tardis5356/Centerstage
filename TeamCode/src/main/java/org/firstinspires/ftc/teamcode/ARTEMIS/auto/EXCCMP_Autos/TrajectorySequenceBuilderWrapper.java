package org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;

import org.firstinspires.ftc.teamcode.ARTEMIS.trajectorysequence.TrajectorySequenceBuilder;

public class TrajectorySequenceBuilderWrapper extends TrajectorySequenceBuilder {

    private boolean flip;

    public TrajectorySequenceBuilderWrapper(Pose2d startPose, Double startTangent,
                                            TrajectoryVelocityConstraint baseVelConstraint,
                                            TrajectoryAccelerationConstraint baseAccelConstraint,
                                            double baseTurnConstraintMaxAngVel,
                                            double baseTurnConstraintMaxAngAccel, boolean flip) {
        super(startPose, startTangent,
                baseVelConstraint, baseAccelConstraint, baseTurnConstraintMaxAngVel, baseTurnConstraintMaxAngAccel);
        this.flip = flip;
    }

    public TrajectorySequenceBuilderWrapper(Pose2d startPose,
                                            TrajectoryVelocityConstraint baseVelConstraint,
                                            TrajectoryAccelerationConstraint baseAccelConstraint,
                                            double baseTurnConstraintMaxAngVel,
                                            double baseTurnConstraintMaxAngAccel,
                                            boolean flip) {
        super(startPose, baseVelConstraint, baseAccelConstraint, baseTurnConstraintMaxAngVel, baseTurnConstraintMaxAngAccel);
        this.flip = flip;
    }

    public TrajectorySequenceBuilderWrapper(Pose2d startPose, TrajectoryVelocityConstraint baseVelConstraint, TrajectoryAccelerationConstraint baseAccelConstraint, double baseTurnConstraintMaxAngVel, double baseTurnConstraintMaxAngAccel) {
        super(startPose, baseVelConstraint, baseAccelConstraint, baseTurnConstraintMaxAngVel, baseTurnConstraintMaxAngAccel);
        this.flip = false;
    }

    public TrajectorySequenceBuilderWrapper(Pose2d startPose, Double startTangent, TrajectoryVelocityConstraint baseVelConstraint, TrajectoryAccelerationConstraint baseAccelConstraint, double baseTurnConstraintMaxAngVel, double baseTurnConstraintMaxAngAccel) {
        super(startPose, startTangent, baseVelConstraint, baseAccelConstraint, baseTurnConstraintMaxAngVel, baseTurnConstraintMaxAngAccel);
        this.flip = false;
    }

    private static Pose2d flipPose(boolean flip, Pose2d originalPose) {
        if (!flip) {
            return originalPose;
        }
        return new Pose2d(originalPose.getX(), -originalPose.getY(), -originalPose.getHeading());
    }

    private Pose2d flipPose(Pose2d originalPose) {
        if (!this.flip) {
            return originalPose;
        }
        return new Pose2d(originalPose.getX(), -originalPose.getY(), -originalPose.getHeading());
    }

    private static double flipTangent(boolean flip, double tangent) {
        if (!flip) {
            return tangent;
        }
        return -tangent;
    }

    private double flipTangent(double tangent) {
        if (!this.flip) {
            return tangent;
        }
        return -tangent;
    }

    private Vector2d flipVector(Vector2d vector) {
        if (!this.flip) {
            return vector;
        }
        return new Vector2d(vector.getX(), -vector.getY());
    }

    @Override
    public TrajectorySequenceBuilderWrapper lineTo(Vector2d endPosition) {
        super.lineTo(flipVector(endPosition));
        return this;
    }

    @Override
    public TrajectorySequenceBuilderWrapper lineTo(
            Vector2d endPosition,
            TrajectoryVelocityConstraint velConstraint,
            TrajectoryAccelerationConstraint accelConstraint) {
        super.lineTo(flipVector(endPosition), velConstraint, accelConstraint);
        return this;
    }

    @Override
    public TrajectorySequenceBuilderWrapper lineToConstantHeading(Vector2d endPosition) {
        super.lineToConstantHeading(flipVector(endPosition));
        return this;
    }

    @Override
    public TrajectorySequenceBuilderWrapper lineToConstantHeading(
            Vector2d endPosition,
            TrajectoryVelocityConstraint velConstraint,
            TrajectoryAccelerationConstraint accelConstraint) {
        super.lineToConstantHeading(flipVector(endPosition), velConstraint, accelConstraint);
        return this;
    }

    @Override
    public TrajectorySequenceBuilderWrapper lineToLinearHeading(Pose2d endPose) {
        super.lineToLinearHeading(flipPose(endPose));
        return this;
    }

    @Override
    public TrajectorySequenceBuilderWrapper lineToLinearHeading(
            Pose2d endPose,
            TrajectoryVelocityConstraint velConstraint,
            TrajectoryAccelerationConstraint accelConstraint) {
        super.lineToLinearHeading(flipPose(endPose), velConstraint, accelConstraint);
        return this;
    }

    @Override
    public TrajectorySequenceBuilderWrapper lineToSplineHeading(Pose2d endPose) {
        super.lineToSplineHeading(flipPose(endPose));
        return this;
    }

    @Override
    public TrajectorySequenceBuilderWrapper lineToSplineHeading(
            Pose2d endPose,
            TrajectoryVelocityConstraint velConstraint,
            TrajectoryAccelerationConstraint accelConstraint) {
        super.lineToSplineHeading(flipPose(endPose), velConstraint, accelConstraint);
        return this;
    }

    @Override
    public TrajectorySequenceBuilderWrapper strafeTo(Vector2d endPosition) {
        super.strafeTo(flipVector(endPosition));
        return this;
    }

    @Override
    public TrajectorySequenceBuilderWrapper strafeTo(
            Vector2d endPosition,
            TrajectoryVelocityConstraint velConstraint,
            TrajectoryAccelerationConstraint accelConstraint) {
        super.strafeTo(flipVector(endPosition), velConstraint, accelConstraint);
        return this;
    }

    @Override
    public TrajectorySequenceBuilderWrapper strafeLeft(double distance) {
        super.strafeLeft(flipTangent(distance));
        return this;
    }

    @Override
    public TrajectorySequenceBuilderWrapper strafeLeft(
            double distance,
            TrajectoryVelocityConstraint velConstraint,
            TrajectoryAccelerationConstraint accelConstraint) {
        super.strafeLeft(flipTangent(distance), velConstraint, accelConstraint);
        return this;
    }

    @Override
    public TrajectorySequenceBuilderWrapper strafeRight(double distance) {
        super.strafeRight(flipTangent(distance));
        return this;
    }

    @Override
    public TrajectorySequenceBuilderWrapper strafeRight(
            double distance,
            TrajectoryVelocityConstraint velConstraint,
            TrajectoryAccelerationConstraint accelConstraint) {
        super.strafeRight(flipTangent(distance), velConstraint, accelConstraint);
        return this;
    }

    @Override
    public TrajectorySequenceBuilderWrapper splineTo(Vector2d endPosition, double endHeading) {
        super.splineTo(flipVector(endPosition), flipTangent(endHeading));
        return this;
    }

    @Override
    public TrajectorySequenceBuilderWrapper splineTo(
            Vector2d endPosition,
            double endHeading,
            TrajectoryVelocityConstraint velConstraint,
            TrajectoryAccelerationConstraint accelConstraint) {
        super.splineTo(flipVector(endPosition), flipTangent(endHeading), velConstraint, accelConstraint);
        return this;
    }

    @Override
    public TrajectorySequenceBuilderWrapper splineToConstantHeading(Vector2d endPosition, double endHeading) {
        super.splineToConstantHeading(flipVector(endPosition), flipTangent(endHeading));
        return this;
    }

    @Override
    public TrajectorySequenceBuilderWrapper splineToConstantHeading(
            Vector2d endPosition,
            double endHeading,
            TrajectoryVelocityConstraint velConstraint,
            TrajectoryAccelerationConstraint accelConstraint) {
        super.splineToConstantHeading(flipVector(endPosition), flipTangent(endHeading), velConstraint, accelConstraint);
        return this;
    }

    @Override
    public TrajectorySequenceBuilderWrapper splineToLinearHeading(Pose2d endPose, double endHeading) {
        super.splineToLinearHeading(flipPose(endPose), flipTangent(endHeading));
        return this;
    }

    @Override
    public TrajectorySequenceBuilderWrapper splineToLinearHeading(
            Pose2d endPose,
            double endHeading,
            TrajectoryVelocityConstraint velConstraint,
            TrajectoryAccelerationConstraint accelConstraint) {
        super.splineToLinearHeading(flipPose(endPose), flipTangent(endHeading), velConstraint, accelConstraint);
        return this;
    }

    @Override
    public TrajectorySequenceBuilderWrapper splineToSplineHeading(Pose2d endPose, double endHeading) {
        super.splineToSplineHeading(flipPose(endPose), flipTangent(endHeading));
        return this;
    }

    @Override
    public TrajectorySequenceBuilderWrapper splineToSplineHeading(
            Pose2d endPose,
            double endHeading,
            TrajectoryVelocityConstraint velConstraint,
            TrajectoryAccelerationConstraint accelConstraint) {
        super.splineToSplineHeading(flipPose(endPose), flipTangent(endHeading), velConstraint, accelConstraint);
        return this;
    }

    @Override
    public TrajectorySequenceBuilderWrapper turn(double angle) {
        super.turn(flipTangent(angle));
        return this;
    }

    @Override
    public TrajectorySequenceBuilderWrapper turn(double angle, double maxAngVel, double maxAngAccel) {
        super.turn(flipTangent(angle), maxAngVel, maxAngAccel);
        return this;
    }

    public TrajectorySequenceBuilderWrapper setFlip(boolean flip) {
        this.flip = flip;
        return this;
    }
}
