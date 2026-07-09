/*
 * OccupancyGridDisplay.hpp
 *
 *  RViz Display class for nav_msgs/OccupancyGrid with configurable colormap.
 *  Mirrors GridMapDisplay.hpp; height-mode and layer-selection are omitted
 *  because OccupancyGrid is flat and single-layer.
 */

#pragma once

#ifndef Q_MOC_RUN
#include <nav_msgs/OccupancyGrid.h>
#include <boost/circular_buffer.hpp>
// OccupancyGrid has a standard top-level header; use the stock rviz display,
// not the modified one (which assumes grid_map_msgs::GridMap's msg->info.header).
#include <rviz/message_filter_display.h>
#endif

namespace rviz {
class BoolProperty;
class FloatProperty;
class IntProperty;
class EditableEnumProperty;
}  // namespace rviz

namespace grid_map_rviz_plugin {

class OccupancyGridVisual;

class OccupancyGridDisplay : public rviz::MessageFilterDisplay<nav_msgs::OccupancyGrid>
{
Q_OBJECT
 public:
  OccupancyGridDisplay();
  virtual ~OccupancyGridDisplay();

 protected:
  virtual void onInitialize();
  virtual void onEnable();
  virtual void reset();

 Q_SIGNALS:
  // Marshals the incoming message to the UI thread for rendering.
  void process(const nav_msgs::OccupancyGrid::ConstPtr& msg);

 private Q_SLOTS:
  void updateHistoryLength();
  void updateColorMap();
  void updateVisualization();
  void updateGridLines();
  void updateAutocomputeIntensityBounds();
  void onProcessMessage(const nav_msgs::OccupancyGrid::ConstPtr& msg);

 private:
  // ROS message callback — immediately emits process().
  void processMessage(const nav_msgs::OccupancyGrid::ConstPtr& msg);

  std::atomic<bool> isReset_{false};
  boost::circular_buffer<boost::shared_ptr<OccupancyGridVisual>> visuals_;

  rviz::FloatProperty*        alphaProperty_;
  rviz::IntProperty*          historyLengthProperty_;
  rviz::BoolProperty*         showGridLinesProperty_;
  rviz::EditableEnumProperty* colorMapProperty_;
  rviz::BoolProperty*         invertColorMapProperty_;
  rviz::BoolProperty*         autocomputeIntensityProperty_;
  rviz::FloatProperty*        minIntensityProperty_;
  rviz::FloatProperty*        maxIntensityProperty_;
  rviz::FloatProperty*        gridLinesThicknessProperty_;
  rviz::IntProperty*          gridCellDecimationProperty_;
};

}  // namespace grid_map_rviz_plugin
