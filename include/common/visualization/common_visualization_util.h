/**
 * @file common_visualization_util.h
 * @author HKUST Aerial Robotics Group
 * @brief
 * @version 0.1
 * @date 2019-03-18
 *
 * @copyright Copyright (c) 2019
 */
#ifndef _COMMON_INC_COMMON_VISUALIZATION_VISUALIZATION_UTIL_H__
#define _COMMON_INC_COMMON_VISUALIZATION_VISUALIZATION_UTIL_H__

#include "common/basics/basics.h"
#include "common/basics/colormap.h"
#include "common/basics/semantics.h"
#include "common/basics/shapes.h"
#include "common/basics/tool_func.h"
#include "common/circle_arc/circle_arc.h"
#include "common/circle_arc/circle_arc_branch.h"
#include "common/lane/lane.h"
#include "common/math/calculations.h"
#include "common/spline/polynomial.h"
#include "common/spline/spline.h"
#include "common/state/state.h"
#include "common/state/waypoint.h"
#include "common/trajectory/trajectory.h"

namespace common {

class VisualizationUtil {
 public:
  /**
   * @brief Get the marker from polynomial parameterization
   * @notice this function do not take care of the header (incl. stamp, frame,
   * and marker id)
   *
   * @tparam N_DEG
   * @tparam N_DIM
   * @param poly input polynomial
   * @param s0 evaluation start
   * @param s1 evaluation end
   * @param step evaluation step
   * @param scale scale for the three dimension
   * @param color color in the order of r, g, b, a
   * @param marker output
   * @return ErrorType
   */
  template <int N_DEG, int N_DIM>
  static ErrorType GetMarkerByPolynomial(const PolynomialND<N_DEG, N_DIM>& poly,
                                         const decimal_t s0, const decimal_t s1,
                                         const decimal_t step,
                                         const Vec3f scale,
                                         const ColorARGB color,
                                         std::vector<Point>& marker) {
    for (decimal_t s = s0; s < s1; s += step) {
      auto v = poly.evaluate(s);
      marker.push_back(Point(v[0], v[1], v[2]));
    }
    return kSuccess;
  }

  /**
   * @brief Get the Marker By Spline object
   *
   * @tparam N_DEG
   * @tparam N_DIM
   * @param spline
   * @param step
   * @param scale
   * @param color
   * @param offset_z
   * @param marker
   * @return ErrorType
   */
  template <int N_DEG, int N_DIM>
  static ErrorType GetMarkerBySpline(const Spline<N_DEG, N_DIM>& spline,
                                     const decimal_t step, const Vec3f& scale,
                                     const ColorARGB& color,
                                     const decimal_t offset_z,
                                     std::vector<Point>& marker) {
    for (decimal_t s = spline.begin(); s < spline.end(); s += step) {
      Vecf<N_DIM> ret;
      if (spline.evaluate(s, 0, &ret) == kSuccess) {
        marker.push_back(Point(ret[0], ret[1], offset_z));
      }
    }

    return kSuccess;
  }

  /**
   * @brief Get the Marker By Lane object
   *
   * @param lane
   * @param step
   * @param scale
   * @param color
   * @param offset_z
   * @param marker
   * @return ErrorType
   */
  static ErrorType GetMarkerByLane(const Lane& lane, const decimal_t step,
                                   const Vec3f& scale, const ColorARGB& color,
                                   const decimal_t offset_z,
                                   std::vector<Point>& marker) {
    // unwrap the parameterization

    if (!lane.IsValid()) return kIllegalInput;
    GetMarkerBySpline<LaneDegree, LaneDim>(lane.position_spline(), step, scale,
                                           color, offset_z, marker);
    return kSuccess;
  }

  /**
   * @brief Get the Marker Array By Trajectory object
   *
   * @param traj
   * @param step
   * @param scale
   * @param color
   * @param offset_z
   * @param marker_arr
   * @return ErrorType
   */
  static ErrorType GetMarkerArrayByTrajectory(const Trajectory& traj,
                                              const decimal_t step,
                                              const Vec3f& scale,
                                              const ColorARGB& color,
                                              const decimal_t offset_z,
                                              std::vector<Point>& marker) {
    if (!traj.IsValid()) return kIllegalInput;
    for (decimal_t s = traj.begin(); s < traj.end(); s += step) {
      common::State state;
      if (traj.GetState(s, &state) == kSuccess) {
        marker.push_back(
            Point(state.vec_position[0], state.vec_position[1], offset_z));
      }
    }
    return kSuccess;
  }

  /**
   * @brief Get the marker array from state vec
   *
   * @param state_vec a vector of states (probably from one trajectory)
   * @param color color
   * @param marker_arr returned marker array
   * @return ErrorType
   */
  static ErrorType GetMarkerArrayByStateVector(const vec_E<State>& state_vec,
                                               const ColorARGB& color,
                                               std::vector<Point>& marker) {
    for (auto& state : state_vec) {
      marker.push_back(Point(state.vec_position[0], state.vec_position[1]));
    }
    return kSuccess;
  }
};

}  // namespace common

#endif  // _COMMON_INC_COMMON_VISUALIZATION_VISUALIZATION_UTIL_H__
