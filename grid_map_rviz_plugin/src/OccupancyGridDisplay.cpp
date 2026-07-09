/*
 * OccupancyGridDisplay.cpp
 *
 *  Display plugin for nav_msgs/OccupancyGrid.
 *  Mirrors GridMapDisplay.cpp; no height-mode or layer-selection properties.
 */

#include "grid_map_rviz_plugin/OccupancyGridDisplay.hpp"
#include "grid_map_rviz_plugin/OccupancyGridVisual.hpp"
#include "grid_map_rviz_plugin/GridMapColorMaps.hpp"

// OccupancyGrid uses the stock rviz include — no modified frame_manager needed.
#include <rviz/frame_manager.h>

#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>

#include <tf/transform_listener.h>

#include <rviz/visualization_manager.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/properties/editable_enum_property.h>

namespace grid_map_rviz_plugin {

OccupancyGridDisplay::OccupancyGridDisplay() {
  qRegisterMetaType<nav_msgs::OccupancyGrid::ConstPtr>(
      "nav_msgs::OccupancyGrid::ConstPtr");

  alphaProperty_ = new rviz::FloatProperty(
      "Alpha", 1.0,
      "0 is fully transparent, 1.0 is fully opaque.",
      this, SLOT(updateVisualization()));

  historyLengthProperty_ = new rviz::IntProperty(
      "History Length", 1,
      "Number of prior messages to keep visible.",
      this, SLOT(updateHistoryLength()));
  historyLengthProperty_->setMin(1);
  historyLengthProperty_->setMax(100);

  showGridLinesProperty_ = new rviz::BoolProperty(
      "Show Grid Lines", true,
      "Draw lines between grid cells.",
      this, SLOT(updateGridLines()));

  gridLinesThicknessProperty_ = new rviz::FloatProperty(
      "Grid Line Thickness", 0.1,
      "Thickness of the grid-cell lines.",
      this, SLOT(updateVisualization()));

  gridCellDecimationProperty_ = new rviz::IntProperty(
      "Grid Cell Decimation", 1,
      "Draw a grid line every N cells.",
      this, SLOT(updateVisualization()));

  colorMapProperty_ = new rviz::EditableEnumProperty(
      "Color Map", "viridis",
      "Colormap applied to occupancy values.",
      this, SLOT(updateVisualization()));

  invertColorMapProperty_ = new rviz::BoolProperty(
      "Invert Color Map", false,
      "Reverse the colormap direction.",
      this, SLOT(updateVisualization()));

  autocomputeIntensityProperty_ = new rviz::BoolProperty(
      "Auto Compute Intensity Bounds", true,
      "Automatically set intensity min/max from the data.",
      this, SLOT(updateAutocomputeIntensityBounds()));

  minIntensityProperty_ = new rviz::FloatProperty(
      "Min Intensity", 0.0,
      "Lower bound for colormap scaling.",
      this, SLOT(updateVisualization()));
  minIntensityProperty_->hide();

  maxIntensityProperty_ = new rviz::FloatProperty(
      "Max Intensity", 1.0,
      "Upper bound for colormap scaling.",
      this, SLOT(updateVisualization()));
  maxIntensityProperty_->hide();
}

OccupancyGridDisplay::~OccupancyGridDisplay() {}

void OccupancyGridDisplay::onInitialize() {
  MFDClass::onInitialize();
  updateHistoryLength();

  // Populate the colormap dropdown.
  for (const auto& name : getColorMapNames()) {
    colorMapProperty_->addOptionStd(name);
  }
}

void OccupancyGridDisplay::onEnable() {
  isReset_ = false;
  connect(this, &OccupancyGridDisplay::process,
          this, &OccupancyGridDisplay::onProcessMessage);
  MFDClass::onEnable();
}

void OccupancyGridDisplay::reset() {
  isReset_ = true;
  disconnect(this, &OccupancyGridDisplay::process,
             this, &OccupancyGridDisplay::onProcessMessage);
  MFDClass::reset();
  visuals_.clear();
}

// ---------------------------------------------------------------------------
// Slots
// ---------------------------------------------------------------------------

void OccupancyGridDisplay::updateHistoryLength() {
  visuals_.rset_capacity(historyLengthProperty_->getInt());
}

void OccupancyGridDisplay::updateColorMap() {
  updateVisualization();
}

void OccupancyGridDisplay::updateGridLines() {
  updateVisualization();
  const bool show = showGridLinesProperty_->getBool();
  gridLinesThicknessProperty_->setHidden(!show);
  gridCellDecimationProperty_->setHidden(!show);
}

void OccupancyGridDisplay::updateAutocomputeIntensityBounds() {
  updateVisualization();
  const bool autocompute = autocomputeIntensityProperty_->getBool();
  minIntensityProperty_->setHidden(autocompute);
  maxIntensityProperty_->setHidden(autocompute);
}

void OccupancyGridDisplay::updateVisualization() {
  const float alpha            = alphaProperty_->getFloat();
  const bool  showGridLines    = showGridLinesProperty_->getBool();
  const std::string colorMap   = colorMapProperty_->getStdString();
  const bool  invertColorMap   = invertColorMapProperty_->getBool();
  const bool  autocompute      = autocomputeIntensityProperty_->getBool();
  const float minIntensity     = minIntensityProperty_->getFloat();
  const float maxIntensity     = maxIntensityProperty_->getFloat();
  const float gridThickness    = gridLinesThicknessProperty_->getFloat();
  const int   gridDecimation   = gridCellDecimationProperty_->getInt();

  for (size_t i = 0; i < visuals_.size(); ++i) {
    visuals_[i]->computeVisualization(alpha, showGridLines, colorMap, invertColorMap,
                                      autocompute, minIntensity, maxIntensity,
                                      gridThickness, gridDecimation);
  }
}

// ---------------------------------------------------------------------------
// Message handling
// ---------------------------------------------------------------------------

void OccupancyGridDisplay::processMessage(
    const nav_msgs::OccupancyGrid::ConstPtr& msg) {
  process(msg);
}

void OccupancyGridDisplay::onProcessMessage(
    const nav_msgs::OccupancyGrid::ConstPtr& msg) {
  if (isReset_) {
    return;
  }

  Ogre::Quaternion orientation;
  Ogre::Vector3    position;
  if (!context_->getFrameManager()->getTransform(
          msg->header.frame_id, msg->header.stamp, position, orientation)) {
    ROS_DEBUG("OccupancyGridDisplay: cannot transform from frame '%s' to '%s'",
              msg->header.frame_id.c_str(), qPrintable(fixed_frame_));
    return;
  }

  boost::shared_ptr<OccupancyGridVisual> visual;
  if (visuals_.full()) {
    visual = visuals_.front();
  } else {
    visual.reset(new OccupancyGridVisual(context_->getSceneManager(), scene_node_));
  }

  visual->setMessage(msg);
  visual->setFramePosition(position);
  visual->setFrameOrientation(orientation);

  visual->computeVisualization(
      alphaProperty_->getFloat(),
      showGridLinesProperty_->getBool(),
      colorMapProperty_->getStdString(),
      invertColorMapProperty_->getBool(),
      autocomputeIntensityProperty_->getBool(),
      minIntensityProperty_->getFloat(),
      maxIntensityProperty_->getFloat(),
      gridLinesThicknessProperty_->getFloat(),
      gridCellDecimationProperty_->getInt());

  visuals_.push_back(visual);
}

}  // namespace grid_map_rviz_plugin

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(grid_map_rviz_plugin::OccupancyGridDisplay, rviz::Display)
