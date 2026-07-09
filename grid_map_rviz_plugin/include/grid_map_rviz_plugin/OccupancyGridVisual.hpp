/*
 * OccupancyGridVisual.hpp
 *
 *  Renders nav_msgs/OccupancyGrid as a flat coloured mesh in RViz.
 *  Deliberately has no dependency on grid_map_core or grid_map_ros.
 */

#pragma once

#include <OGRE/OgreMaterial.h>
#include <OGRE/OgreSharedPtr.h>
#include <Eigen/Core>
#include <nav_msgs/OccupancyGrid.h>
#include <boost/shared_ptr.hpp>

namespace Ogre {
class Vector3;
class Quaternion;
class ManualObject;
class ColourValue;
}  // namespace Ogre

namespace rviz {
class BillboardLine;
}

namespace grid_map_rviz_plugin {

// Visualises a single nav_msgs::OccupancyGrid message.
class OccupancyGridVisual {
 public:
  using ColorArray = Eigen::Array<Ogre::ColourValue, Eigen::Dynamic, Eigen::Dynamic>;

  OccupancyGridVisual(Ogre::SceneManager* sceneManager, Ogre::SceneNode* parentNode);
  virtual ~OccupancyGridVisual();

  // Convert int8 occupancy values to float [0,1]; -1 (unknown) → NaN.
  void setMessage(const nav_msgs::OccupancyGrid::ConstPtr& msg);

  // Rebuild the Ogre mesh from the stored data.
  void computeVisualization(float alpha, bool showGridLines, std::string colorMap,
                            bool invertColorMap, bool autocomputeIntensity,
                            float minIntensity, float maxIntensity,
                            float gridLineThickness, int gridCellDecimation);

  void setFramePosition(const Ogre::Vector3& position);
  void setFrameOrientation(const Ogre::Quaternion& orientation);

 private:
  Ogre::SceneNode*    frameNode_;
  Ogre::SceneManager* sceneManager_;
  Ogre::ManualObject* manualObject_;
  Ogre::MaterialPtr   material_;
  std::string         materialName_;

  boost::shared_ptr<rviz::BillboardLine> meshLines_;

  // Data from the last OccupancyGrid message.
  Eigen::MatrixXf data_;    // rows=height, cols=width; NaN = unknown cell
  double resolution_{0.0};
  double originX_{0.0};
  double originY_{0.0};
  unsigned int width_{0};
  unsigned int height_{0};
  bool haveData_{false};

  // Initialise (or clear) the ManualObject ready for new geometry.
  void initializeAndBeginManualObject(size_t nVertices);

  // Allocate BillboardLine buffer before adding points.
  void initializeMeshLines(size_t cols, size_t rows, double resolution,
                           double alpha, double lineWidth);

  // Map every cell's float value to an Ogre colour.
  ColorArray computeColorValues(const Eigen::MatrixXf& data, std::string colorMap,
                                bool invertColorMap, float minIntensity,
                                float maxIntensity, bool autocomputeIntensity);

  // Rainbow colouriser — intensity in [0,1].
  static Ogre::ColourValue getRainbowColor(float intensity);

  // Linear interpolation between two colours.
  Ogre::ColourValue getInterpolatedColor(float intensity,
                                         Ogre::ColourValue minColor,
                                         Ogre::ColourValue maxColor);

  // Returns up to 4 Ogre coordinates for the grid-line corners of cell (i,j).
  std::vector<Ogre::Vector3> computeMeshLineVertices(
      int i, int j, int gridCellDecimation, bool isNthRow, bool isNthCol,
      bool isLastRow, bool isLastCol) const;
};

}  // namespace grid_map_rviz_plugin
