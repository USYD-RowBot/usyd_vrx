#pragma once 

#include <math.h>

namespace usyd_vrx {

namespace GuidanceAlgorithms {

  //! Struct representing (x,y) point, 2D vector
  struct Vec2D {
    float x;
    float y;
  };

  /*!
  * Calculates square of distance from vec1 to vec2.
  * @param vec1 first vector.
  * @param vec2 second vector.
  * @return squared distance from vec1 to vec2.
  */
  float Distance_2(Vec2D& vec1, Vec2D& vec2);

  /*!
  * Computes virtual waypoint target using nonlinear guidance law.
  * @param wp_prev the previous waypoint.
  * @param wp_next the next waypoint.
  * @param vessel the vessel position.
  * @param nlgl_radius desired radius for the NLGL.
  * @return the virtual waypoint target.
  */
  Vec2D NonlinearGuidanceLaw(
    Vec2D& wp_prev, Vec2D& wp_next, Vec2D& vessel, float& nlgl_radius);

  /*!
  * Computes direct angle from vessel to target waypoint.
  * @param target the target waypoint.
  * @param vessel the vessel position.
  * @return the angle in radians from vessel to target waypoint.
  */
  float PurePursuit(Vec2D& target, Vec2D& vessel);

}
}