/*
 * OccupancyGridVisual.cpp
 *
 *  Rendering back-end for OccupancyGridDisplay.
 *  Mirrors GridMapVisual.cpp; no grid_map_core / grid_map_ros dependency.
 */

#include "grid_map_rviz_plugin/OccupancyGridVisual.hpp"
#include "grid_map_rviz_plugin/GridMapColorMaps.hpp"

#include <ros/ros.h>

#include <OGRE/OgreManualObject.h>
#include <OGRE/OgreMaterialManager.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreTechnique.h>
#include <OGRE/OgreTextureManager.h>
#include <OGRE/OgreVector3.h>
#include <rviz/ogre_helpers/billboard_line.h>
#include <rviz/uniform_string_stream.h>

#include <cmath>
#include <limits>

namespace grid_map_rviz_plugin {

OccupancyGridVisual::OccupancyGridVisual(Ogre::SceneManager* sceneManager,
                                         Ogre::SceneNode* parentNode)
    : manualObject_(nullptr), haveData_(false) {
  sceneManager_ = sceneManager;
  frameNode_ = parentNode->createChildSceneNode();
  meshLines_.reset(new rviz::BillboardLine(sceneManager_, frameNode_));
}

OccupancyGridVisual::~OccupancyGridVisual() {
  if (manualObject_) {
    sceneManager_->destroyManualObject(manualObject_);
    material_->unload();
    Ogre::MaterialManager::getSingleton().remove(material_->getName());
  }
  sceneManager_->destroySceneNode(frameNode_);
}

void OccupancyGridVisual::setMessage(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
  width_      = msg->info.width;
  height_     = msg->info.height;
  resolution_ = msg->info.resolution;
  originX_    = msg->info.origin.position.x;
  originY_    = msg->info.origin.position.y;

  data_.resize(height_, width_);
  for (unsigned int r = 0; r < height_; ++r) {
    for (unsigned int c = 0; c < width_; ++c) {
      const int8_t v = msg->data[r * width_ + c];
      data_(r, c) = (v < 0) ? std::numeric_limits<float>::quiet_NaN()
                             : static_cast<float>(v) / 100.0f;
    }
  }
  haveData_ = true;
}

void OccupancyGridVisual::computeVisualization(float alpha, bool showGridLines,
                                               std::string colorMapName, bool invertColorMap,
                                               bool autocomputeIntensity, float minIntensity,
                                               float maxIntensity, float gridLineThickness,
                                               int gridCellDecimation) {
  if (!haveData_) {
    ROS_DEBUG("OccupancyGridVisual: no data; call setMessage() first.");
    return;
  }
  if (width_ < 2 || height_ < 2) {
    ROS_DEBUG("OccupancyGridVisual: grid too small to visualise.");
    return;
  }

  const size_t rows = height_;
  const size_t cols = width_;
  const size_t nVertices = rows * cols;

  initializeAndBeginManualObject(nVertices);

  meshLines_->clear();
  if (showGridLines) {
    initializeMeshLines(cols, rows, resolution_, alpha, gridLineThickness);
  }

  gridCellDecimation = std::max(gridCellDecimation, 1);

  const ColorArray colorValues = computeColorValues(
      data_, colorMapName, invertColorMap, minIntensity, maxIntensity, autocomputeIntensity);

  Eigen::ArrayXXi indexToOgreIndex;
  indexToOgreIndex.setConstant(rows, cols, -1);
  int ogreIndex = 0;

  for (size_t i = 0; i < rows; ++i) {
    for (size_t j = 0; j < cols; ++j) {
      const bool valid = !std::isnan(data_(i, j));

      if (valid) {
        // Cell centre in world coordinates.
        const double wx = originX_ + (j + 0.5) * resolution_;
        const double wy = originY_ + (i + 0.5) * resolution_;
        manualObject_->position(wx, wy, 0.0f);

        const Ogre::ColourValue& color = colorValues(i, j);
        // Grey sentinel (unknown) is rendered fully transparent.
        if (color.r == 0.5f && color.g == 0.5f && color.b == 0.5f) {
          manualObject_->colour(color.r, color.g, color.b, 0.0f);
        } else {
          manualObject_->colour(color.r, color.g, color.b, alpha);
        }

        indexToOgreIndex(i, j) = ogreIndex++;

        // Triangles can only be formed to the top-left of the current vertex.
        if (i == 0 || j == 0) {
          continue;
        }

        std::vector<int> vertexIndices;
        for (size_t k = 0; k < 2; ++k) {
          for (size_t l = 0; l < 2; ++l) {
            const int ri = static_cast<int>(i) - static_cast<int>(k);
            const int ci = static_cast<int>(j) - static_cast<int>(l);
            if (!std::isnan(data_(ri, ci))) {
              vertexIndices.emplace_back(indexToOgreIndex(ri, ci));
            }
          }
        }

        if (vertexIndices.size() > 2) {
          if (vertexIndices.size() == 3) {
            manualObject_->triangle(vertexIndices[0], vertexIndices[1], vertexIndices[2]);
          } else {
            manualObject_->quad(vertexIndices[0], vertexIndices[2],
                                vertexIndices[3], vertexIndices[1]);
          }
        }
      }

      // Grid-line geometry.
      const bool isNthRow  = (i % gridCellDecimation == 0);
      const bool isNthCol  = (j % gridCellDecimation == 0);
      const bool isLastRow = (i == rows - 1);
      const bool isLastCol = (j == cols - 1);
      const bool drawLines = (isNthRow && isNthCol) || (isLastRow && isNthCol) ||
                              (isLastCol && isNthRow) || (isLastRow && isLastCol);

      if (!showGridLines || !drawLines) {
        continue;
      }

      std::vector<Ogre::Vector3> pts = computeMeshLineVertices(
          i, j, gridCellDecimation, isNthRow, isNthCol, isLastRow, isLastCol);

      if (pts.size() > 2) {
        meshLines_->addPoint(pts[0]); meshLines_->addPoint(pts[1]); meshLines_->newLine();

        if (pts.size() == 3) {
          meshLines_->addPoint(pts[1]); meshLines_->addPoint(pts[2]); meshLines_->newLine();
        } else {
          meshLines_->addPoint(pts[1]); meshLines_->addPoint(pts[3]); meshLines_->newLine();
          meshLines_->addPoint(pts[3]); meshLines_->addPoint(pts[2]); meshLines_->newLine();
        }

        meshLines_->addPoint(pts[2]); meshLines_->addPoint(pts[0]); meshLines_->newLine();
      }
    }  // cols
  }  // rows

  manualObject_->end();
  material_->getTechnique(0)->setLightingEnabled(false);

  if (alpha < 0.9998f) {
    material_->getTechnique(0)->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
    material_->getTechnique(0)->setDepthWriteEnabled(false);
  } else {
    material_->getTechnique(0)->setSceneBlending(Ogre::SBT_REPLACE);
    material_->getTechnique(0)->setDepthWriteEnabled(true);
  }
}

// ---------------------------------------------------------------------------
// Private helpers
// ---------------------------------------------------------------------------

std::vector<Ogre::Vector3> OccupancyGridVisual::computeMeshLineVertices(
    int i, int j, int gridCellDecimation, bool isNthRow, bool isNthCol,
    bool isLastRow, bool isLastCol) const {
  std::vector<Ogre::Vector3> pts;
  pts.reserve(4);

  for (int k = 0; k < 2; ++k) {
    for (int l = 0; l < 2; ++l) {
      const int strideX = isLastRow ? (i % gridCellDecimation + int(isNthRow) * gridCellDecimation)
                                    : gridCellDecimation;
      const int strideY = isLastCol ? (j % gridCellDecimation + int(isNthCol) * gridCellDecimation)
                                    : gridCellDecimation;
      const int ri = std::max(i - k * strideX, 0);
      const int ci = std::max(j - l * strideY, 0);

      if (std::isnan(data_(ri, ci))) {
        continue;
      }
      const double wx = originX_ + (ci + 0.5) * resolution_;
      const double wy = originY_ + (ri + 0.5) * resolution_;
      pts.emplace_back(static_cast<float>(wx), static_cast<float>(wy), 0.0f);
    }
  }
  return pts;
}

void OccupancyGridVisual::initializeAndBeginManualObject(size_t nVertices) {
  if (!manualObject_) {
    static uint32_t count = 0;
    rviz::UniformStringStream ss;
    ss << "OccupancyGridMesh" << count++;
    manualObject_ = sceneManager_->createManualObject(ss.str());
    manualObject_->setDynamic(true);
    frameNode_->attachObject(manualObject_);

    ss << "Material";
    materialName_ = ss.str();
    material_ = Ogre::MaterialManager::getSingleton().create(
        materialName_, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
    material_->setReceiveShadows(false);
    material_->getTechnique(0)->setLightingEnabled(true);
    material_->setCullingMode(Ogre::CULL_NONE);
  }

  manualObject_->clear();
  manualObject_->estimateVertexCount(nVertices);
  manualObject_->begin(materialName_, Ogre::RenderOperation::OT_TRIANGLE_LIST);
}

void OccupancyGridVisual::initializeMeshLines(size_t cols, size_t rows, double resolution,
                                              double alpha, double lineWidth) {
  meshLines_->setColor(0.0, 0.0, 0.0, alpha);
  meshLines_->setLineWidth(resolution * lineWidth);
  meshLines_->setMaxPointsPerLine(2);
  const size_t nLines = 2 * (rows * (cols - 1) + cols * (rows - 1));
  meshLines_->setNumLines(nLines);
}

OccupancyGridVisual::ColorArray OccupancyGridVisual::computeColorValues(
    const Eigen::MatrixXf& data, std::string colorMapName,
    bool invertColorMap, float minIntensity, float maxIntensity,
    bool autocomputeIntensity) {

  if (autocomputeIntensity) {
    float minVal = std::numeric_limits<float>::max();
    float maxVal = std::numeric_limits<float>::lowest();
    for (int r = 0; r < data.rows(); ++r) {
      for (int c = 0; c < data.cols(); ++c) {
        const float v = data(r, c);
        if (std::isfinite(v)) {
          if (v < minVal) minVal = v;
          if (v > maxVal) maxVal = v;
        }
      }
    }
    if (minVal <= maxVal) {
      minIntensity = minVal;
      maxIntensity = minVal + std::max(maxVal - minVal, 0.2f);
    }
  }

  const float minI = minIntensity;
  const float maxI = std::max(maxIntensity, minIntensity + 1e-6f);

  // Lambda: normalize a raw value to [0,1] and optionally invert.
  auto normalize = [&](float v) -> float {
    float norm = (v - minI) / (maxI - minI);
    norm = std::min(std::max(norm, 0.0f), 1.0f);
    return invertColorMap ? 1.0f - norm : norm;
  };

  const bool useTable = (colorMap.count(colorMapName) > 0);

  if (useTable) {
    const std::vector<std::vector<float>>& ctable = colorMap.at(colorMapName);
    return data.unaryExpr([&](float v) -> Ogre::ColourValue {
      if (std::isnan(v)) return Ogre::ColourValue(0.5f, 0.5f, 0.5f, 0.0f);
      return getColorMap(normalize(v), ctable);
    });
  } else {
    // Fallback: built-in rainbow gradient.
    return data.unaryExpr([&](float v) -> Ogre::ColourValue {
      if (std::isnan(v)) return Ogre::ColourValue(0.5f, 0.5f, 0.5f, 0.0f);
      return getRainbowColor(normalize(v));
    });
  }
}

Ogre::ColourValue OccupancyGridVisual::getRainbowColor(float intensity) {
  intensity = std::min(std::max(intensity, 0.0f), 1.0f);

  float h = intensity * 5.0f + 1.0f;
  int   i = static_cast<int>(std::floor(h));
  float f = h - i;
  if (!(i & 1)) f = 1.0f - f;  // even i: invert fraction
  float n = 1.0f - f;

  Ogre::ColourValue color;
  if      (i <= 1) { color[0] = n;    color[1] = 0.0f; color[2] = 1.0f; }
  else if (i == 2) { color[0] = 0.0f; color[1] = n;    color[2] = 1.0f; }
  else if (i == 3) { color[0] = 0.0f; color[1] = 1.0f; color[2] = n;    }
  else if (i == 4) { color[0] = n;    color[1] = 1.0f; color[2] = 0.0f; }
  else             { color[0] = 1.0f; color[1] = n;    color[2] = 0.0f; }

  return color;
}

Ogre::ColourValue OccupancyGridVisual::getInterpolatedColor(float intensity,
                                                             Ogre::ColourValue minColor,
                                                             Ogre::ColourValue maxColor) {
  intensity = std::min(std::max(intensity, 0.0f), 1.0f);
  Ogre::ColourValue color;
  color.r = intensity * (maxColor.r - minColor.r) + minColor.r;
  color.g = intensity * (maxColor.g - minColor.g) + minColor.g;
  color.b = intensity * (maxColor.b - minColor.b) + minColor.b;
  return color;
}

void OccupancyGridVisual::setFramePosition(const Ogre::Vector3& position) {
  frameNode_->setPosition(position);
}

void OccupancyGridVisual::setFrameOrientation(const Ogre::Quaternion& orientation) {
  frameNode_->setOrientation(orientation);
}

}  // namespace grid_map_rviz_plugin
