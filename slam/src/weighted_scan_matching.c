#include <math.h>
#include <slam/logging.h>
#include <slam/utils.h>
#include <slam/weighted_scan_matching.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#define TRANSLATION_EPSILON 1e-6f
#define ROTATION_EPSILON 1e-6f

static inline void vec2_reset(float *x, float *y) {
  *x = 0.0f;
  *y = 0.0f;
}

static inline void mat2x2_reset(float m[4]) {
  m[0] = m[1] = m[2] = m[3] = 0.0f;
}

static inline bool mat2x2_inv(float *mat, float inv[4]) {
  float det = mat[0] * mat[3] - mat[1] * mat[2];
  if (fabs(det) < 1e-15f) {
    ERROR("Matrix is singular, cannot invert");
    return false;
  }
  inv[0] = mat[3] / det;  // inv[0][0]
  inv[1] = -mat[1] / det; // inv[0][1]
  inv[2] = -mat[2] / det; // inv[1][0]
  inv[3] = mat[0] / det;  // inv[1][1]
  return true;
}

static inline void mat2x2_add_inplace(float *a, const float *b) {
  a[0] += b[0];
  a[1] += b[1];
  a[2] += b[2];
  a[3] += b[3];
}

static inline void scan_to_point(const scan_t *scan, size_t i, float *x,
                                 float *y) {
  float r = scan->range[i];
  float a = DEG2RAD(i);
  *x = r * cosf(a);
  *y = r * sinf(a);
}

/**
 * @brief Calculate the covariance matrix for the measurement process
 * noise.
 *
 * @param cov the covariance matrix to add noise to
 * @param angle the angle of the point in radians
 * @param range the range of the point
 * @param sigma_theta the standard deviation of the angle noise
 * @param sigma_l the standard deviation of the range noise
 */
static inline void calc_noise_covariance(float cov[4], float angle, float range,
                                         float sigma_theta, float sigma_l) {
  const float a = 0.5f * range * range * sigma_theta * sigma_theta;
  const float b = 0.5f * sigma_l * sigma_l;
  const float sin_theta = sinf(angle);
  const float cos_theta = cosf(angle);
  const float sin2 = sin_theta * sin_theta;
  const float cos2 = cos_theta * cos_theta;
  const float sin2theta = sinf(2 * angle);
  cov[0] = a * 2.0f * sin2 + b * 2.0f * cos2;
  cov[1] = cov[2] = a * -sin2theta + b * sin2theta;
  cov[3] = a * 2.0f * cos2 + b * 2.0f * sin2;
}

/**
 * @brief Calculate the covariance matrix for the correspondence error
 *
 * @param cov the covariance matrix to add the correspondence error to
 * @param scan the scan containing the points
 * @param index the index of the point in the scan
 * @param map the occupancy quadtree map
 * @param dist the distance to the nearest point in the map
 */
static void calc_correspondence_covariance(float cov[4], const scan_t *scan,
                                           uint16_t index,
                                           const occupancy_quadtree_t *map) {
  const float leaf_size = map->size >> map->max_depth;
  const float delta = leaf_size;
  const float variance = (delta * delta) / 3.0f;

  size_t prev = (index + 359) % 360;
  size_t next = (index + 1) % 360;
  float rp = scan->range[prev], rn = scan->range[next];

  if (rp <= 1e-6f || rn <= 1e-6f) {
    cov[0] = cov[3] = variance;
    cov[1] = cov[2] = 0.0f;
    return;
  }

  float ap = DEG2RAD(prev), an = DEG2RAD(next);
  float px = rp * cosf(ap), py = rp * sinf(ap);
  float nx = rn * cosf(an), ny = rn * sinf(an);
  float dx = nx - px, dy = ny - py;
  float len = hypotf(dx, dy);
  if (len < 1e-6f || len > 2.0f * leaf_size) {
    cov[0] = cov[3] = variance;
    cov[1] = cov[2] = 0.0f;
    return;
  }
  dx /= len;
  dy /= len;
  cov[0] = variance * dx * dx;
  cov[1] = cov[2] = variance * dx * dy;
  cov[3] = variance * dy * dy;
}

bool scan_matching_match(const scan_t *current_scan,
                         const lidar_sensor_t *sensor,
                         occupancy_quadtree_t *map, const pose_t *initial_guess,
                         robot_pose_t *pose_estimate, uint16_t max_iterations) {
  // From the papers: Weighted Range Sensor Matching Algorithms for Mobile
  // Robot Displacement Estimation (2002) and Robust Weighted Scan Matching
  // with Quadtrees (2009)

  // TODO: use error from the initial guess to limit the search space
  const uint16_t max_match_dist = 50;

  float t_x = initial_guess->x;
  float t_y = initial_guess->y;
  float phi = initial_guess->r;

  float noise_cov[4], corresp_cov[4], cov[4], cov_inv[4];
  float cov_inv_sum[4], cov_inv_sum_inv[4];
  float cov_inv_r_sum_x, cov_inv_r_sum_y;
  float delta_theta_num, delta_theta_den;

  for (int iter = 0; iter < max_iterations; iter++) {
    mat2x2_reset(cov_inv_sum);
    vec2_reset(&cov_inv_r_sum_x, &cov_inv_r_sum_y);
    delta_theta_num = delta_theta_den = 0.0f;

    for (size_t k = 0; k < 360; k++) {
      float range = current_scan->range[k];
      if (range <= 1e-6f)
        continue;

      float bx, by, rx, ry, tx, ty;
      scan_to_point(current_scan, k, &bx, &by);

      float c = cosf(phi), s = sinf(phi);
      rx = c * bx - s * by;
      ry = s * bx + c * by;
      tx = rx + t_x;
      ty = ry + t_y;

      uint16_t dist;
      occupancy_quadtree_t *node =
          occupancy_quadtree_nearest(map, tx, ty, &dist);
      if (!node || dist > max_match_dist)
        continue;

      float ax = node->x;
      float ay = node->y;

      calc_correspondence_covariance(corresp_cov, current_scan, k, map);
      calc_noise_covariance(noise_cov, DEG2RAD(k), range, sensor->bearing_error,
                            sensor->range_error);

      // rotate noise_cov into world frame and add to correspondence
      float tmp00 = c * noise_cov[0] + -s * noise_cov[2];
      float tmp01 = c * noise_cov[1] + -s * noise_cov[3];
      float tmp10 = s * noise_cov[0] + c * noise_cov[2];
      float tmp11 = s * noise_cov[1] + c * noise_cov[3];

      cov[0] = corresp_cov[0] + tmp00 * c + tmp01 * s;
      cov[1] = corresp_cov[1] + tmp00 * -s + tmp01 * c;
      cov[2] = corresp_cov[2] + tmp10 * c + tmp11 * s;
      cov[3] = corresp_cov[3] + tmp10 * -s + tmp11 * c;

      if (!mat2x2_inv(cov, cov_inv))
        continue;
      mat2x2_add_inplace(cov_inv_sum, cov_inv);

      float dx = ax - rx;
      float dy = ay - ry;
      cov_inv_r_sum_x += cov_inv[0] * dx + cov_inv[1] * dy;
      cov_inv_r_sum_y += cov_inv[2] * dx + cov_inv[3] * dy;

      float px = ax - tx;
      float py = ay - ty;
      float Jqx = -ry;
      float Jqy = rx;

      float p_dot_Jq = px * (cov_inv[0] * Jqx + cov_inv[1] * Jqy) +
                       py * (cov_inv[2] * Jqx + cov_inv[3] * Jqy);
      float Jq_P_Jq = Jqx * (cov_inv[0] * Jqx + cov_inv[1] * Jqy) +
                      Jqy * (cov_inv[2] * Jqx + cov_inv[3] * Jqy);
      delta_theta_num += p_dot_Jq;
      delta_theta_den += Jq_P_Jq;
    }

    if (!mat2x2_inv(cov_inv_sum, cov_inv_sum_inv))
      return false;
    if (fabsf(delta_theta_den) < 1e-6f)
      return false;

    float ntx = cov_inv_sum_inv[0] * cov_inv_r_sum_x +
                cov_inv_sum_inv[1] * cov_inv_r_sum_y;
    float nty = cov_inv_sum_inv[2] * cov_inv_r_sum_x +
                cov_inv_sum_inv[3] * cov_inv_r_sum_y;
    float dtx = ntx - t_x, dty = nty - t_y;
    t_x = ntx;
    t_y = nty;

    float dr = delta_theta_num / delta_theta_den;
    phi = clamp_rotation(phi + dr);

    DEBUG("Iter %d: t = (%.4f, %.4f), phi = %.4f, dtx = %.4f, dty = %.4f, dr = "
          "%.4f",
          iter, t_x, t_y, phi, dtx, dty, dr);

    if (fabsf(dr) < ROTATION_EPSILON && fabsf(dtx) < TRANSLATION_EPSILON &&
        fabsf(dty) < TRANSLATION_EPSILON) {
      pose_estimate->pose.x = t_x;
      pose_estimate->pose.y = t_y;
      pose_estimate->pose.r = phi;

      float rotational_variance = 1.0f / delta_theta_den;

      INFO("Translational covariance: [%f, %f; %f, %f]", cov_inv_sum_inv[0],
           cov_inv_sum_inv[1], cov_inv_sum_inv[2], cov_inv_sum_inv[3]);
      INFO("Rotational covariance: %f", rotational_variance);

      // For now we'll just use the diagonal elements of the covariance
      // as the error
      pose_estimate->error.x = ceilf(cov_inv_sum_inv[0]);
      pose_estimate->error.y = ceilf(cov_inv_sum_inv[3]);
      pose_estimate->error.r = rotational_variance;

      INFO("Convergence reached after %d iterations", iter);
      return true;
    }
  }

  WARN("Scan matching failed to converge");
  return false;
}
