#include <math.h>
#include <slam/logging.h>
#include <slam/utils.h>
#include <slam/weighted_scan_matching.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#define TRANSLATION_EPSILON 1e-5f
#define ROTATION_EPSILON 1e-5f

typedef struct {
  float x, y;
} vec2_t;

// === Utility Functions ===

static inline vec2_t vec2_add(vec2_t a, vec2_t b) {
  return (vec2_t){a.x + b.x, a.y + b.y};
}

static inline vec2_t vec2_sub(vec2_t a, vec2_t b) {
  return (vec2_t){a.x - b.x, a.y - b.y};
}

static inline vec2_t vec2_rotate(vec2_t p, float angle) {
  float c = cosf(angle), s = sinf(angle);
  return (vec2_t){c * p.x - s * p.y, s * p.x + c * p.y};
}

static inline float vec2_norm(vec2_t v) { return hypotf(v.x, v.y); }

static inline bool mat2x2_inv(float* mat, float inv[4]) {
  float det = mat[0] * mat[3] - mat[1] * mat[2];
  if (fabs(det) < 1e-15f) {
    ERROR("Matrix is singular, cannot invert");
    return false;
  }
  inv[0] = mat[3] / det;   // inv[0][0]
  inv[1] = -mat[1] / det;  // inv[0][1]
  inv[2] = -mat[2] / det;  // inv[1][0]
  inv[3] = mat[0] / det;   // inv[1][1]
  return true;
}

static inline void mat2x2_add_inplace(float* a, const float* b) {
  a[0] += b[0];
  a[1] += b[1];
  a[2] += b[2];
  a[3] += b[3];
}

vec2_t scan_to_point(const scan_t* scan, size_t index) {
  if (index >= 360) {
    fprintf(stderr, "Index out of bounds in scan_to_point\n");
    return (vec2_t){0, 0};
  }
  float angle = index * PI / 180.0f;  // convert degrees to radians
  float range = scan->range[index];
  return (vec2_t){range * cosf(angle), range * sinf(angle)};
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
static void calc_noise_covariance(float* cov, float angle, float range,
                                  float sigma_theta, float sigma_l) {
  DEBUG(
      "Calculating noise covariance for angle %.3f, range %.3f, "
      "sigma_theta %.3f, sigma_l %.3f",
      angle, range, sigma_theta, sigma_l);

  const float a = 0.5f * range * range * sigma_theta * sigma_theta;
  const float b = 0.5f * sigma_l * sigma_l;
  const float sin_theta = sinf(angle);
  const float cos_theta = cosf(angle);
  const float sin_sqr_theta = sin_theta * sin_theta;
  const float cos_sqr_theta = cos_theta * cos_theta;
  const float sin_2_theta = sinf(2 * angle);
  cov[0] = a * 2.0f * sin_sqr_theta + b * 2.0f * cos_sqr_theta;  // xx
  cov[1] = a * -sin_2_theta + b * sin_2_theta;                   // xy
  cov[2] = a * -sin_2_theta + b * sin_2_theta;                   // yx
  cov[3] = a * 2.0f * cos_sqr_theta + b * 2.0f * sin_sqr_theta;  // yy
}

/**
 * @brief Calculate the covariance matrix for the correspondence error
 *
 * @param cov the covariance matrix to add the correspondence error to
 * @param scan the scan containing the points
 * @param index the index of the point in the scan
 * @param map the occupancy quadtree map
 */
static void calc_correspondence_covariance(float* cov, const scan_t* scan,
                                           uint16_t index,
                                           const occupancy_quadtree_t* map) {
  // calculate the variance of the correspondence error based on
  // the quadtree resolution
  const float leaf_size = map->size / (1 << map->max_depth);
  const float variance = (leaf_size * leaf_size) / 3.0f;
  // calculate the tangent
  const size_t prev_index = (index - 1 + 360) % 360;
  const size_t next_index = (index + 1) % 360;
  if (scan->range[prev_index] <= 1e-6f || scan->range[next_index] <= 1e-6f) {
    cov[0] = cov[3] = variance;
    cov[1] = cov[2] = 0;
    return;
  }

  const vec2_t prev = scan_to_point(scan, prev_index);
  const vec2_t next = scan_to_point(scan, next_index);
  const vec2_t td = vec2_sub(next, prev);

  const float length = vec2_norm(td);
  // fallback if the length is too small or the points
  // are too far apart
  if (length < 1e-6f || length > leaf_size * 2.0f) {
    cov[0] = cov[3] = variance;
    cov[1] = cov[2] = 0;
    return;
  }
  // normalize the tangent vector
  const vec2_t tangent = (vec2_t){td.x / length, td.y / length};
  cov[0] = variance * tangent.x * tangent.x;  // xx
  cov[1] = variance * tangent.x * tangent.y;  // xy
  cov[2] = variance * tangent.y * tangent.x;  // yx
  cov[3] = variance * tangent.y * tangent.y;  // yy
}

bool scan_matching_match(const scan_t* current_scan,
                         const lidar_sensor_t* sensor,
                         occupancy_quadtree_t* map, const pose_t* initial_guess,
                         pose_t* pose_estimate) {
  // From the papers: Weighted Range Sensor Matching Algorithms for Mobile
  // Robot Displacement Estimation (2002) and Robust Weighted Scan Matching
  // with Quadtrees (2009)

  const uint16_t leaf_size = map->size >> map->max_depth;

  // nearest neighbor outlier distance
  const uint16_t nn_outlier_distance = leaf_size * 2;  // N x the leaf size

  double phi = initial_guess->r;
  vec2_t t = {initial_guess->x, initial_guess->y};

  float corresp_cov[4];
  float noise_cov[4];
  float cov[4];
  float cov_inv[4];
  float cov_inv_sum[4];      // sum(inverse covariance)
  vec2_t cov_inv_r_sum;      // sum(inverse covariance x residual)
  float cov_inv_sum_inv[4];  // inverse of the sum of the inverse covariances
                             // (P_pp)
  float delta_theta_num;     // numerator for the rotation update
  float delta_theta_den;     // denominator for the rotation update
  float dr, dtx, dty;        // delta rotation update and translation updates
  float ntx, nty;            // new translation

  vec2_t point_b, rotated_point_b, transformed_point_b;

  for (int iter = 0; iter < 100; iter++) {
    // INFO(
    //     "Iter %d: t ="
    //     " (%.4f, %.4f), phi = %.4f",
    //     iter, t.x, t.y, phi);

    // reset the covariance matrix
    cov_inv_sum[0] = cov_inv_sum[1] = cov_inv_sum[2] = cov_inv_sum[3] = 0;
    cov_inv_r_sum.x = cov_inv_r_sum.y = 0;

    // reset the rotation update variables
    delta_theta_num = 0;
    delta_theta_den = 0;

    // iterate over all points in the current scan
    for (size_t k = 0; k < 360; k++) {
      if (current_scan->range[k] <= 1e-6f) {
        // skip points with no range data
        continue;
      }

      point_b = scan_to_point(current_scan, k);
      rotated_point_b = vec2_rotate(point_b, phi);  // q
      transformed_point_b = vec2_add(rotated_point_b, t);

      DEBUG(
          "Scan point %zu: (%.3f, %.3f) → rotated (%.3f, %.3f) "
          "→ transformed (%.3f, %.3f)",
          k, point_b.x, point_b.y, rotated_point_b.x, rotated_point_b.y,
          transformed_point_b.x, transformed_point_b.y);

      // find the closest point in the map
      uint16_t distance;
      occupancy_quadtree_t* closest_node = occupancy_quadtree_nearest(
          map, transformed_point_b.x, transformed_point_b.y, &distance);
      DEBUG("Scan point %zu: closest node at (%.3f, %.3f) with distance %.3f",
            k, closest_node ? closest_node->x : 0.0f,
            closest_node ? closest_node->y : 0.0f, distance);
      if (!closest_node || distance > nn_outlier_distance) {
        continue;
      }
      vec2_t point_a = {closest_node->x, closest_node->y};

      // DEBUG("Correspondence at scan %zu → map point (%.3f, %.3f)", k,
      // point_a.x,
      //       point_a.y);
      // calculate the covariance for this point
      calc_correspondence_covariance(corresp_cov, current_scan, k, map);
      calc_noise_covariance(noise_cov, k * PI / 180.0f, current_scan->range[k],
                            sensor->bearing_error, sensor->range_error);

      DEBUG(
          "Scan %zu: noise_cov = [[%.9f, %.9f], [%.9f, %.9f]], "
          "corresp_cov = [[%.9f, %.9f], [%.9f, %.9f]]",
          k, noise_cov[0], noise_cov[1], noise_cov[2], noise_cov[3],
          corresp_cov[0], corresp_cov[1], corresp_cov[2], corresp_cov[3]);

      float c = cosf(phi);
      float s = sinf(phi);

      // R * N * R^T
      float N[2][2], R[2][2];
      N[0][0] = noise_cov[0];
      N[0][1] = noise_cov[1];
      N[1][0] = noise_cov[2];
      N[1][1] = noise_cov[3];
      R[0][0] = c;
      R[0][1] = -s;
      R[1][0] = s;
      R[1][1] = c;

      // tmp = R * N
      float tmp[2][2] = {{R[0][0] * N[0][0] + R[0][1] * N[1][0],
                          R[0][0] * N[0][1] + R[0][1] * N[1][1]},
                         {R[1][0] * N[0][0] + R[1][1] * N[1][0],
                          R[1][0] * N[0][1] + R[1][1] * N[1][1]}};

      // cov = tmp * R^T
      cov[0] = corresp_cov[0] + tmp[0][0] * R[0][0] + tmp[0][1] * R[0][1];
      cov[1] = corresp_cov[1] + tmp[0][0] * R[1][0] + tmp[0][1] * R[1][1];
      cov[2] = corresp_cov[2] + tmp[1][0] * R[0][0] + tmp[1][1] * R[0][1];
      cov[3] = corresp_cov[3] + tmp[1][0] * R[1][0] + tmp[1][1] * R[1][1];

      DEBUG("Scan %zu: covariance matrix = [[%.9f, %.9f], [%.9f, %.9f]]", k,
            cov[0], cov[1], cov[2], cov[3]);

      // accumulate the inverse covariance matrix
      if (!mat2x2_inv(cov, cov_inv)) {
        ERROR("Failed to invert covariance matrix for scan point %zu", k);
        continue;
      }
      mat2x2_add_inplace(cov_inv_sum, cov_inv);
      // calculate the difference between the point in the map and the
      // rotated point
      vec2_t rk = vec2_sub(point_a, rotated_point_b);  // for translation update
      // accumulate the inverse covariance times the residual
      // (residual = point_a - rotated_point_b)
      cov_inv_r_sum.x += cov_inv[0] * rk.x + cov_inv[1] * rk.y;
      cov_inv_r_sum.y += cov_inv[2] * rk.x + cov_inv[3] * rk.y;

      DEBUG(
          "Scan %zu: residual = (%.3f, %.3f), "
          "cov_inv_r_sum = (%.3f, %.3f)",
          k, rk.x, rk.y, cov_inv_r_sum.x, cov_inv_r_sum.y);

      // rotation update
      vec2_t pk =
          vec2_sub(point_a, transformed_point_b);  // for rotation update
      float Jq_x = -rotated_point_b.y;
      float Jq_y = rotated_point_b.x;
      float p_dot_Jq = pk.x * cov_inv[0] * Jq_x + pk.x * cov_inv[1] * Jq_y +
                       pk.y * cov_inv[2] * Jq_x + pk.y * cov_inv[3] * Jq_y;
      float Jq_P_Jq = Jq_x * (cov_inv[0] * Jq_x + cov_inv[1] * Jq_y) +
                      Jq_y * (cov_inv[2] * Jq_x + cov_inv[3] * Jq_y);
      delta_theta_num += p_dot_Jq;
      delta_theta_den += Jq_P_Jq;
    }

    // update the translation
    if (!mat2x2_inv(cov_inv_sum, cov_inv_sum_inv)) {
      ERROR("Failed to invert covariance sum matrix");
      return false;
    }

    if (fabs(delta_theta_den) < 1e-6f) {
      ERROR("Delta theta denominator is too small, skipping rotation update");
      return false;
    }

    ntx = cov_inv_sum_inv[0] * cov_inv_r_sum.x +
          cov_inv_sum_inv[1] * cov_inv_r_sum.y;
    nty = cov_inv_sum_inv[2] * cov_inv_r_sum.x +
          cov_inv_sum_inv[3] * cov_inv_r_sum.y;

    dtx = ntx - t.x;  // translation update in x
    dty = nty - t.y;  // translation update in y

    // update the translation
    t.x = ntx;
    t.y = nty;

    // update the rotation
    dr = delta_theta_num / delta_theta_den;

    phi = clamp_rotation(phi + dr);

    INFO(
        "Iter %d: t = (%.4f, %.4f), phi = %.4f, "
        "dtx = %.4f, dty = %.4f, dr = %.4f",
        iter, t.x, t.y, phi, dtx, dty, dr);

    if (fabsf(dr) < ROTATION_EPSILON && fabsf(dtx) < TRANSLATION_EPSILON &&
        fabsf(dty) < TRANSLATION_EPSILON) {
      pose_estimate->x = t.x;
      pose_estimate->y = t.y;
      pose_estimate->r = phi;
      INFO("Convergence reached after %d iterations", iter);
      return true;
    }
  }

  WARN("Scan matching failed to converge");
  return false;
}
