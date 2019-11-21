#include "GuidanceAlgorithms.hpp"

namespace usyd_vrx {

namespace GuidanceAlgorithms {

float Distance_2(Vec2D& vec1, Vec2D& vec2)
{
  return pow(vec2.x - vec1.x, 2) + pow(vec2.y - vec1.y, 2);
}

float Distance(Vec2D& vec1, Vec2D& vec2)
{
  return sqrt(pow(vec2.x - vec1.x, 2) + pow(vec2.y - vec1.y, 2));
}

Vec2D DistanceComponents(Vec2D& vec1, Vec2D& vec2)
{
  Vec2D distance_components;

  distance_components.x = vec2.x - vec1.x;
  distance_components.y = vec2.y - vec1.y;

  return distance_components;
}

Vec2D NonlinearGuidanceLaw(
  Vec2D& wp_prev, Vec2D& wp_next, Vec2D& vessel, float& nlgl_radius)
{
  Vec2D wp_output; // Virtual waypoint to return

  // Square of distance from vessel to next waypoint
  float wp_dist_2 = Distance_2(vessel, wp_next);
  float rad_2  = pow(nlgl_radius, 2); // Square of nlgl circle radius

  // If target waypoint is already inside NLGL radius, move directly to it
  if (wp_dist_2 < rad_2)
    wp_output = wp_next; // Set virtual waypoint to actual waypoint

  else
  {
    // Vector from previous waypoint to next waypoint
    Vec2D wp_vec = {(wp_next.x - wp_prev.x), (wp_next.y - wp_prev.y)};

    // Squared values of wp_vec, for convenience
    Vec2D wp_vec_2 = {pow(wp_vec.x, 2), pow(wp_vec.y, 2)};

    // Closest point from vessel to direct waypoint line
    Vec2D closest_point;

    closest_point.y = (1/(wp_vec_2.x + wp_vec_2.y))
      *(vessel.y*wp_vec_2.y + wp_prev.y*wp_vec_2.x + (vessel.x - wp_prev.x)*wp_vec.x*wp_vec.y);

    closest_point.x = 
      (1/wp_vec.y)*(closest_point.y*wp_vec.x + wp_prev.x*wp_vec.y - wp_prev.y*wp_vec.x);

    // Square of distance from vessel to closest point on waypoint line, avoiding sqrt
    float dist_2 = Distance_2(vessel, closest_point); 
    float rad_2  = pow(nlgl_radius, 2); // Square of nlgl circle radius

    // If minimum distance to waypoint line is less than our circle radius
    if (dist_2 > rad_2)
      wp_output = wp_prev; // Return to previous waypoint

    else // Set virtual waypoint
    {
      // Distance along waypoint line from closest point in order to intersect circle
      float travel_dist = sqrt(rad_2 - dist_2);

      // Normalise wp_vec into unit vector and scale by travel_dist
      float scaler = travel_dist/sqrt(wp_vec_2.x + wp_vec_2.y);

      // Create the two possible virtual waypoints
      Vec2D virtual1 = {closest_point.x + scaler*wp_vec.x, 
                        closest_point.y + scaler*wp_vec.y};
      Vec2D virtual2 = {closest_point.x - scaler*wp_vec.x, 
                        closest_point.y - scaler*wp_vec.y};

      // Calculate distances from virtual waypoints to target waypoint
      float dist_2_virtual1 = Distance_2(virtual1, wp_next);
      float dist_2_virtual2 = Distance_2(virtual2, wp_next);
    
      // Select virtual waypoint closest to target waypoint
      if (dist_2_virtual1 < dist_2_virtual2)
        wp_output = virtual1;
      else
        wp_output = virtual2;
    }
  }

  return wp_output;
}

float PurePursuit(Vec2D& target, Vec2D& vessel)
{
  float theta; // Angle from vessel to target, to be returned

  if (target.x == vessel.x) // Check for vertical line
  {
    if (target.y > vessel.y)
      theta = M_PI_2;
    else
      theta = -M_PI_2;
  }
  else // Calculate angle from vessel to target with atan2
    theta = atan2(target.y - vessel.y, target.x - vessel.x);

  return theta;  
}

}
}