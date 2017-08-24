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

#include "MotionModel.hpp"

MotionModel::MotionModel(const Eigen::Vector3d initialPosition, const Eigen::Quaterniond initialOrientation)
  : position_( initialPosition ),  orientation_(initialOrientation), linearVelocity_(0, 0, 0),
angularVelocity_( 1, 0, 0, 0)
{  
}

void MotionModel::CurrentCameraPose(Eigen::Vector3d& currentPosition, Eigen::Quaterniond& currentOrientation) const
{
  currentPosition = position_;
  currentOrientation =  orientation_;
}

void MotionModel::PredictNextCameraPose(Eigen::Vector3d& predictedPosition, Eigen::Quaterniond& predictedOrientation) const
{
  // Compute predicted position by integrating linear velocity

  predictedPosition = position_ + linearVelocity_;

  // Compute predicted orientation by integrating angular velocity

  Eigen::Quaterniond predictedOrientation_e = orientation_ * angularVelocity_;
  predictedOrientation_e.normalize();
  predictedOrientation =  predictedOrientation_e;
}
// subscript c means current, p means previous
void MotionModel::PredictNextCameraMotion(Eigen::Vector3d& tcp, Eigen::Quaterniond& rcp) const
{
  // Compute predicted orientation by integrating angular velocity
  Eigen::Quaterniond predictedOrientation_e = orientation_ * angularVelocity_;
  predictedOrientation_e.normalize();
  tcp = predictedOrientation_e.inverse()*(-linearVelocity_);
  rcp =  angularVelocity_.inverse();
}
// update the camera state given a new camera pose
void MotionModel::UpdateCameraPose(const Eigen::Vector3d& newPosition, const Eigen::Quaterniond& newOrientation)
{
  // Compute linear velocity
  Eigen::Vector3d newLinearVelocity( newPosition - position_ );
  // In order to be robust against fast camera movements linear velocity is smoothed over time
  newLinearVelocity = (newLinearVelocity + linearVelocity_) * 0.5;

  // compute rotation between q1 and q2: q2 * qInverse(q1);
  Eigen::Quaterniond newAngularVelocity =  orientation_.inverse()*newOrientation;

  // In order to be robust against fast camera movements angular velocity is smoothed over time
  newAngularVelocity = newAngularVelocity.slerp(0.5, angularVelocity_);

  newAngularVelocity.normalize();

  // Update the current state variables

  position_ = newPosition;
  orientation_ =  newOrientation;
  linearVelocity_ = newLinearVelocity;
  angularVelocity_ = newAngularVelocity;
}
