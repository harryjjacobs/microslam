#include <math.h>
#include <microslam/map.h>
#include <microslam/utils.h>
#include <stdio.h>
#include <stdlib.h>

void map_add_scan(occupancy_quadtree_t *occupancy, scan_t *scan, pose_t *pose) {
  float r, dx, dy;
  for (unsigned short i = 0; i < 360; i++) {
    if (scan->range[i] < 1e-6) {
      continue;
    }

    r = pose->r + DEG2RAD(i);

    // update along the ray from the robot to the endpoint of the scan as free
    float step = occupancy->size / powf(2, occupancy->max_depth);
    for (float d = 0; d < scan->range[i]; d += step) {
      dx = d * cos(r);
      dy = d * sin(r);
      occupancy_quadtree_update(occupancy, pose->x + dx, pose->y + dy,
                                LOG_ODDS_FREE);
    }
  }

  // update the endpoint of the scan as occupied.
  // we do this as a separate step to avoid overwriting the occupied cells
  // with free cells from nearby rays that hit further away
  for (unsigned short i = 0; i < 360; i++) {
    if (scan->range[i] < 1e-6) {
      continue;
    }

    r = pose->r + DEG2RAD(i);

    dx = scan->range[i] * cos(r);
    dy = scan->range[i] * sin(r);

    // update the endpoint of the scan as occupied
    occupancy_quadtree_update(occupancy, pose->x + dx, pose->y + dy,
                              LOG_ODDS_OCCUPIED * 0.1f);
  }
}

void compute_scan_score(occupancy_quadtree_t *occupancy, scan_t *scan,
                        pose_t *pose, float *score) {
  const int decimate = 10;  // decimate the scan to speed up the computation
  float half_leaf_size = occupancy->size / powf(2, occupancy->max_depth) * 0.5f;

  // transform the scan to the pose and compute the score
  float r, x, y;
  unsigned short hits = 0;

  *score = 0;
  for (unsigned short i = 0; i < 360; i += decimate) {
    if (scan->range[i] < 1e-6) {
      continue;
    }

    // transform the scan to world coordinates
    r = pose->r + DEG2RAD(i);
    x = pose->x + scan->range[i] * cos(r);
    y = pose->y + scan->range[i] * sin(r);

    // check if the ray intersects with an occupied leaf in the quadtree
    // occupancy_quadtree_t *leaf = occupancy_quadtree_find(occupancy, x, y);
    float distance;
    occupancy_quadtree_t *leaf = occupancy_quadtree_raycast(
        occupancy, pose->x, pose->y, x, y, scan->range[i], &distance);
    if (leaf != NULL && fabsf(distance - scan->range[i]) <= half_leaf_size) {
      // if the leaf is occupied, add the log odds to the score
      *score += leaf->log_odds;
      hits++;
    }
  }

  // normalize the score
  if (hits > 0) {
    *score /= 360.0f / (float)decimate;
  } else {
    *score = -INFINITY;
  }
}

// Compute the gradient of the distance function using finite differences.
void scan_match_distance_gradient(occupancy_quadtree_t *occupancy, float x,
                                  float y, float *grad_x, float *grad_y) {
  float delta = occupancy->size / powf(2, occupancy->max_depth) * 0.5f;
  // printf("delta: %f\n", delta);

  *grad_x = 0;
  *grad_y = 0;

  float d_x_plus, d_x_minus, d_y_plus, d_y_minus;

  if (occupancy_quadtree_nearest(occupancy, x + delta, y, &d_x_plus) == NULL ||
      occupancy_quadtree_nearest(occupancy, x - delta, y, &d_x_minus) == NULL ||
      occupancy_quadtree_nearest(occupancy, x, y + delta, &d_y_plus) == NULL ||
      occupancy_quadtree_nearest(occupancy, x, y - delta, &d_y_minus) == NULL) {
    fprintf(stderr, "gradient calculation failed at (%f, %f)\n", x, y);
    return;
  }

  *grad_x = (d_x_plus - d_x_minus) / (2.0f * delta);
  *grad_y = (d_y_plus - d_y_minus) / (2.0f * delta);

  // printf("d_x_plus: %f, d_x_minus: %f\n", d_x_plus, d_x_minus);
  // printf("d_y_plus: %f, d_y_minus: %f\n", d_y_plus, d_y_minus);
  // printf("grad_x: %f, grad_y: %f\n", *grad_x, *grad_y);
}

void map_scan_match_gradient(occupancy_quadtree_t *occupancy, scan_t *scan,
                             pose_t *pose, pose_t *gradient, float *score) {
  // the cost function is the sum of the squared distances to the nearest leaf:
  // E(p)=∑_i(d(T_p(s_i)))2
  // for pose p and scan s, where T_p(s_i) is the transformed scan point s_i
  // and d is the distance to the nearest leaf in the quadtree

  const int decimate = 2;  // decimate the scan to speed up the computation
  // nearest points further than 5 leaves away are considered outliers
  const float inlier_distance =
      5.0f * occupancy->size / powf(2, occupancy->max_depth);

  // float tolerance = occupancy->size / powf(2, occupancy->max_depth) * 10.0f;
  // fprintf(stderr, "tolerance: %f\n", tolerance);

  // transformed scan coordinates (in world frame)
  float r, x, y;
  float cos_r, sin_r;

  // partial derivatives of the transformed (world frame) x and y, w.r.t. theta
  float dx_dtheta, dy_dtheta;
  // partial derivatives of the distance function w.r.t. x and y
  float dd_dx, dd_dy;
  // partial derivative of the distance function w.r.t. theta
  float dd_dtheta;

  int hits = 0;
  // float normalisation_factor = 0.0f;

  gradient->x = 0;
  gradient->y = 0;
  gradient->r = 0;

  *score = 0;
  for (unsigned short i = 0; i < 360; i += decimate) {
    if (scan->range[i] < 1e-5) {
      continue;
    }

    // the angle of the scan in the world frame
    r = pose->r + DEG2RAD(i);
    cos_r = cosf(r);
    sin_r = sinf(r);
    // the transformed x and y of the scan in the world frame
    x = pose->x + scan->range[i] * cos_r;
    y = pose->y + scan->range[i] * sin_r;

    // find the closest leaf in the quadtree
    float distance;
    occupancy_quadtree_t *leaf =
        occupancy_quadtree_nearest(occupancy, x, y, &distance);
    // fprintf(stderr, "distance: %f\n", distance);
    if (leaf != NULL &&
        distance < inlier_distance) {  // && distance > inlier_distance
      // calculate the partial derivatives of the transformed x and y, w.r.t.
      // theta
      // x​=x+r.​cos(θ+ϕi​) => dx/dθ = -r.​sin(θ+ϕi​)
      // y​=y+r.​sin(θ+ϕi​) => dy/dθ = r.​cos(θ+ϕi​)
      dx_dtheta = -scan->range[i] * sin_r;
      dy_dtheta = scan->range[i] * cos_r;
      // calculate the gradient of the distance function w.r.t. x and y
      scan_match_distance_gradient(occupancy, x, y, &dd_dx, &dd_dy);
      // calculate the partial derivative of the distance function w.r.t. theta
      // dd/dθ = (dd/dx * dx/dθ + dd/dy * dy/dθ)
      dd_dtheta = (dd_dx * dx_dtheta + dd_dy * dy_dtheta);
      gradient->x += distance * dd_dx;
      gradient->y += distance * dd_dy;
      gradient->r += distance * dd_dtheta;
      // normalisation_factor += leaf->log_odds;
      *score += distance * distance;
      hits++;
    }
  }

  if (hits == 0) {
    *score = -INFINITY;
    return;
  }

  // printf("hits: %d\n", hits);
  // printf("score: %f\n", *score);
  // printf("gradient: %f %f %f\n", gradient->x, gradient->y, gradient->r);

  gradient->x /= (float)hits;
  gradient->y /= (float)hits;
  gradient->r /= (float)hits;

  // printf("gradient: %f %f %f\n", gradient->x, gradient->y, gradient->r);
}

unsigned char map_scan_match(occupancy_quadtree_t *occupancy, scan_t *scan,
                             state_t *prior, pose_t *estimate, float *score,
                             unsigned short iterations) {
  const float epsilon = occupancy->size / powf(2, occupancy->max_depth) *
                        0.2f;  // convergence threshold

  *score = -INFINITY;

  // gradient descent
  float step_size = 0.00001f;  // note - this depends on the log odds scale
                               // (because gradients are weighted by log odds)
  float current_score;
  pose_t current_estimate = prior->pose;
  pose_t last_update = prior->pose;
  pose_t gradient = {0, 0, 0};

  unsigned short i = 0;
  while (i < iterations) {
    map_scan_match_gradient(occupancy, scan, &current_estimate, &gradient,
                            &current_score);

    // printf("gradient: %f %f %f\n", gradient.x, gradient.y, gradient.r);

    // if the score is better than the best score, update the best score
    if (current_score > *score) {
      *score = current_score;
      *estimate = current_estimate;
      last_update = current_estimate;
    }

    // update the estimate using the gradient
    current_estimate.x -= step_size * gradient.x;
    current_estimate.y -= step_size * gradient.y;
    current_estimate.r -= 0.1f * step_size * gradient.r;

    // check for convergence
    // if (fabsf(current_estimate.x - last_update.x) < epsilon &&
    //     fabsf(current_estimate.y - last_update.y) < epsilon &&
    //     fabsf(current_estimate.r - last_update.r) < epsilon) {
    //   break;
    // }

    float grad_norm = sqrtf(gradient.x * gradient.x + gradient.y * gradient.y +
                            gradient.r * gradient.r);
    if (grad_norm < epsilon) {
      break;
    }

    i++;
  }

  printf("final estimate: %.9g %.9g %.9g\n", current_estimate.x,
         current_estimate.y, current_estimate.r);

  printf("diff: %.9g %.9g %.9g\n", fabsf(current_estimate.x - last_update.x),
         fabsf(current_estimate.y - last_update.y),
         fabsf(current_estimate.r - last_update.r));

  printf("gradient: %.9g %.9g %.9g\n", gradient.x, gradient.y, gradient.r);

  printf("gradient norm: %.9g\n",
         sqrtf(gradient.x * gradient.x + gradient.y * gradient.y +
               gradient.r * gradient.r));

  // TODO: check if convergence was reached
  if (i == iterations || *score < 0) {
    printf("did not converge\n");
    return 0;
  } else {
    printf("converged\n");
    return 1;
  }
}

void entropy_count(occupancy_quadtree_t *leaf, void *count) {
  (void)leaf;
  (*(size_t *)count)++;
}

float map_entropy(occupancy_quadtree_t *occupancy) {
  size_t occupied = 0;
  size_t free = pow(2, 2 * occupancy->max_depth);
  occupancy_quadtree_iterate_leafs_depth_first(occupancy, &occupied,
                                               entropy_count);
  float epsilon = 1e-10;  // small value to prevent log(0)
  float P_occ = (float)occupied / (occupied + free) + epsilon;
  float P_free = (float)free / (occupied + free) + epsilon;
  return -(P_occ * log(P_occ) + P_free * log(P_free));
}