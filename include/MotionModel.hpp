/**
 * This file is part of S-PTAM.
 *
 * Copyright (C) 2015 Taihú Pire and Thomas Fischer
 * For more information see <https://github.com/lrse/sptam>
 *
 * S-PTAM is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * S-PTAM is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with S-PTAM. If not, see <http://www.gnu.org/licenses/>.
 *
 * Authors:  Taihú Pire <tpire at dc dot uba dot ar>
 *           Thomas Fischer <tfischer at dc dot uba dot ar>
 *
 * Laboratory of Robotics and Embedded Systems
 * Department of Computer Science
 * Faculty of Exact and Natural Sciences
 * University of Buenos Aires
 */
#pragma once

#include <eigen3/Eigen/Geometry>

/**
 * This motion model follows the one proposed in the master thesis of
 * Christof Hoppe: Large-Scale Robotic SLAM through Visual Mapping p.43
 * TODO (Maybe not anymore)
 */
class MotionModel
{
  public:

    MotionModel(const Eigen::Vector3d initialPosition, const Eigen::Quaterniond initialOrientation);

    // Get the current camera pose.
    void CurrentCameraPose(Eigen::Vector3d& currentPosition, Eigen::Quaterniond& currentOrientation) const;

    // Predict the next camera pose.
    void PredictNextCameraPose(Eigen::Vector3d& predictedPosition, Eigen::Quaterniond& predictedOrientation) const;
    // predict the incremental motion (tcp, rcp) of transform from previous to current frame, based on decay velocity model
    void PredictNextCameraMotion(Eigen::Vector3d& tcp, Eigen::Quaterniond& rcp) const;
    // Update the motion model given a new camera pose.
    void UpdateCameraPose(const Eigen::Vector3d& newPosition, const Eigen::Quaterniond& newOrientation);

  private:

    Eigen::Vector3d position_;// camera position in a world frame

    Eigen::Quaterniond orientation_;//rotation from camera frame to a world frame

    Eigen::Vector3d linearVelocity_;

    Eigen::Quaterniond angularVelocity_;
};
