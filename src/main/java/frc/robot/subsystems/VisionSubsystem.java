/*
 * MIT License
 *
 * Copyright (c) PhotonVision
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

public class VisionSubsystem extends SubsystemBase {
    private final PhotonCamera aprilTagCameraFront;
    //private final PhotonCamera aprilTagCameraBack;
    private final PhotonCamera noteCamera;

    public final PhotonPoseEstimator photonEstimatorFront;
    //public final PhotonPoseEstimator photonEstimatorBack;

    private double lastEstTimestamp = 0;

    public VisionSubsystem() {
        aprilTagCameraFront = new PhotonCamera(Constants.Vision.kAprilTagCameraFront);
        //aprilTagCameraBack = new PhotonCamera(kAprilTagCameraBack);
        noteCamera = new PhotonCamera(Constants.Vision.kNoteCamera);

        photonEstimatorFront = new PhotonPoseEstimator(
                Constants.Vision.kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, aprilTagCameraFront, Constants.Vision.kRobotToCamFront);
        //photonEstimatorBack = new PhotonPoseEstimator(
        //        kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, aprilTagCameraBack, kRobotToCamBack);
        photonEstimatorFront.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        //photonEstimatorBack.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

    public PhotonPipelineResult getLatestResultATF() { // Get the latest result for the April Tag camera
        return aprilTagCameraFront.getLatestResult();
    }

    /*public PhotonPipelineResult getLatestResultATB() {
        return aprilTagCameraBack.getLatestResult();
    }*/

    public PhotonPipelineResult getLatestResultN() { // Get the latest result for the Note camera
        return noteCamera.getLatestResult();
    }

    /**
     * The latest estimated robot pose on the field from vision data. This may be empty. This should
     * only be called once per loop.
     *
     * @return An {@link EstimatedRobotPose} with an estimated pose, estimate
     *         timestamp, and targets
     *         used for estimation.
     */
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
        var visionEst = photonEstimatorFront.update();
        double latestTimestamp = aprilTagCameraFront.getLatestResult().getTimestampSeconds();
        boolean newResult = Math.abs(latestTimestamp - lastEstTimestamp) > 1e-5;
        if (newResult)
            lastEstTimestamp = latestTimestamp;
        return visionEst;
    }

    /**
     * The standard deviations of the estimated pose from
     * {@link #getEstimatedGlobalPose()}, for use
     * with {@link edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
     * SwerveDrivePoseEstimator}.
     * This should only be used when there are targets visible.
     *
     * @param estimatedPose The estimated pose to guess standard deviations for.
     */
    public Matrix<N3, N1> getEstimationStdDevs(Pose2d estimatedPose) {
        var estStdDevs = Constants.Vision.kSingleTagStdDevs;
        var targets = getLatestResultATF().getTargets();
        int numTags = 0;
        double avgDist = 0;
        for (var tgt : targets) {
            var tagPose = photonEstimatorFront.getFieldTags().getTagPose(tgt.getFiducialId());
            if (tagPose.isEmpty())
                continue;
            numTags++;
            avgDist += tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
        }
        if (numTags == 0)
            return estStdDevs;
        avgDist /= numTags;
        // Decrease std devs if multiple targets are visible
        if (numTags > 1)
            estStdDevs = Constants.Vision.kMultiTagStdDevs;
        // Increase std devs based on (average) distance
        if (numTags == 1 && avgDist > 4)
            estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        else
            estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));

        return estStdDevs;
    }
}