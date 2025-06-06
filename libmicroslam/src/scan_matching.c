#include <math.h>
#include <microslam/scan_matching.h>
#include <microslam/utils.h>
#include <stdio.h>
#include <stdlib.h>

// compute the gradient of the distance function using finite differences.
void scan_matching_distance_gradient(occupancy_quadtree_t *occupancy, float x,
                                     float y, float *grad_x, float *grad_y) {
  float delta = occupancy->size / (1 << occupancy->max_depth) * 0.5f;
  float d_x_plus_dist, d_x_minus_dist, d_y_plus_dist, d_y_minus_dist;

  occupancy_quadtree_t *d_x_plus =
      occupancy_quadtree_nearest(occupancy, x + delta, y, &d_x_plus_dist);
  if (d_x_plus == NULL) goto fail;

  occupancy_quadtree_t *d_x_minus =
      occupancy_quadtree_nearest(occupancy, x - delta, y, &d_x_minus_dist);
  if (d_x_minus == NULL) goto fail;

  occupancy_quadtree_t *d_y_plus =
      occupancy_quadtree_nearest(occupancy, x, y + delta, &d_y_plus_dist);
  if (d_y_plus == NULL) goto fail;

  occupancy_quadtree_t *d_y_minus =
      occupancy_quadtree_nearest(occupancy, x, y - delta, &d_y_minus_dist);
  if (d_y_minus == NULL) goto fail;

  *grad_x = (d_x_plus_dist - d_x_minus_dist) / (2.0f * delta);
  *grad_y = (d_y_plus_dist - d_y_minus_dist) / (2.0f * delta);
  return;

fail:
  fprintf(stderr, "gradient calculation failed at (%f, %f)\n", x, y);
  *grad_x = 0;
  *grad_y = 0;
}

void scan_matching_gradient(occupancy_quadtree_t *occupancy, scan_t *scan,
                            pose_t *pose, pose_t *gradient, float *sum) {
  // the cost function is the sum of the squared distances to the nearest leaf:
  // E(p)=∑_i(d(T_p(s_i)))2
  // for pose p and scan s, where T_p(s_i) is the transformed scan point s_i
  // and d is the distance to the nearest leaf in the quadtree

  const int decimate = 1;  // decimate the scan to speed up the computation
  const float inlier_distance = MICROSLAM_SCAN_MATCHING_INLIER_DISTANCE_COUNT *
                                occupancy->size / (1 << occupancy->max_depth);

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

  gradient->x = 0;
  gradient->y = 0;
  gradient->r = 0;

  *sum = 0;
  for (unsigned short i = 0; i < 360; i += decimate) {
    if (scan->range[i] < 1e-6f) {
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
    if (leaf != NULL) {  //  && distance < inlier_distance
      // calculate the partial derivatives of the transformed x and y, w.r.t.
      // theta
      // x​=x+r.​cos(θ+ϕi​) => dx/dθ = -r.​sin(θ+ϕi​)
      // y​=y+r.​sin(θ+ϕi​) => dy/dθ = r.​cos(θ+ϕi​)
      dx_dtheta = -scan->range[i] * sin_r;
      dy_dtheta = scan->range[i] * cos_r;
      // calculate the gradient of the distance function w.r.t. x and y
      scan_matching_distance_gradient(occupancy, x, y, &dd_dx, &dd_dy);
      // calculate the partial derivative of the distance function w.r.t. theta
      // dd/dθ = (dd/dx * dx/dθ + dd/dy * dy/dθ)
      dd_dtheta = (dd_dx * dx_dtheta + dd_dy * dy_dtheta);
      gradient->x += distance * dd_dx;
      gradient->y += distance * dd_dy;
      gradient->r += distance * dd_dtheta;
      *sum += distance * distance;
      hits++;
    }
  }

  if (hits == 0) {
    *sum = -INFINITY;
    return;
  }

  gradient->x /= (float)hits;
  gradient->y /= (float)hits;
  gradient->r /= (float)hits;
}

unsigned short scan_match_lm(occupancy_quadtree_t *occupancy, scan_t *scan,
                             state_t *prior, pose_t *estimate, float *cost,
                             unsigned short iterations) {
  const int decimate = 1;
  const float convergence_epsilon = 1e-10;
  const float inlier_distance = MICROSLAM_SCAN_MATCHING_INLIER_DISTANCE_COUNT *
                                occupancy->size / (1 << occupancy->max_depth);
  const float initial_lambda = 1e-3f;
  const float lambda_up = 1.5f;
  const float lambda_down = 0.1f;

  float lambda = initial_lambda;
  *estimate = prior->pose;

  for (int iter = 0; iter < iterations; iter++) {
    *cost = 0.0f;
    float H[3][3] = {0};  // approximated Hessian
    float b[3] = {0};     // gradient vector

    for (int i = 0; i < 360; i += decimate) {
      float r_i = scan->range[i];
      if (r_i < 1e-5f) continue;

      float angle = estimate->r + DEG2RAD(i);
      float cos_r = cosf(angle);
      float sin_r = sinf(angle);

      float x = estimate->x + r_i * cos_r;
      float y = estimate->y + r_i * sin_r;

      float distance;
      if (occupancy_quadtree_nearest(occupancy, x, y, &distance) == NULL) {
        //  || distance > inlier_distance
        continue;
      }

      float ddx, ddy;
      scan_matching_distance_gradient(occupancy, x, y, &ddx, &ddy);

      float dx_dtheta = -r_i * sin_r;
      float dy_dtheta = r_i * cos_r;

      float J[3] = {ddx, ddy, ddx * dx_dtheta + ddy * dy_dtheta};

      // huber loss
      float huber_k = inlier_distance;
      float w = 1.0f;
      // if (distance > huber_k) {
      //   w = huber_k / distance;  // reduces influence of large residuals
      // }

      for (int m = 0; m < 3; ++m) {
        b[m] += w * distance * J[m];
        for (int n = 0; n < 3; ++n) {
          H[m][n] += w * J[m] * J[n];
        }
      }

      *cost += w * distance * distance;
    }

    if (*cost < 1e-6f) {
      *cost = -INFINITY;
      return 0;
    }

    // apply damping to Hessian
    for (int i = 0; i < 3; ++i) {
      H[i][i] += lambda;
    }

    // solve (H + λI) * delta = -b
    float delta[3];
    solve_linear_system_3x3(H, b, delta);

    // try the update
    pose_t new_pose = {
        .x = estimate->x - delta[0],
        .y = estimate->y - delta[1],
        .r = estimate->r - delta[2],
    };

    // evaluate new score
    float new_cost = 0.0f;
    for (int i = 0; i < 360; i += decimate) {
      float r_i = scan->range[i];
      if (r_i < 1e-5f) continue;

      float angle = new_pose.r + DEG2RAD(i);
      float cos_r = cosf(angle);
      float sin_r = sinf(angle);

      float x = new_pose.x + r_i * cos_r;
      float y = new_pose.y + r_i * sin_r;

      float distance;
      if (occupancy_quadtree_nearest(occupancy, x, y, &distance) == NULL ||
          distance > inlier_distance) {
        continue;
      }

      new_cost += distance * distance;
    }

    printf("cost: %.9g\n", *cost);
    printf("new_cost: %.9g\n", new_cost);
    printf("Δcost: %.9g\n", *cost - new_cost);
    printf("delta: %.9g %.9g %.9g\n", delta[0], delta[1], delta[2]);

    if (new_cost < *cost) {
      // accept step, reduce damping
      *estimate = new_pose;
      lambda *= lambda_down;
      *cost = new_cost;
    } else {
      // reject step, increase damping
      lambda *= lambda_up;
    }

    // Convergence check
    if (fabsf(delta[0]) + fabsf(delta[1]) + fabsf(delta[2]) <
        convergence_epsilon) {
      printf(
          "Converged at %.9g %.9g %.9g, with delta %.9g %.9g %.9g after %d "
          "iterations\n",
          estimate->x, estimate->y, estimate->r, delta[0], delta[1], delta[2],
          iter);
      return 1;
    }
  }

  return 0;
}

unsigned char scan_match(occupancy_quadtree_t *occupancy, scan_t *scan,
                         state_t *prior, pose_t *estimate, float *score,
                         unsigned short iterations) {
  // convergence threshold
  const float epsilon =
      powf(MICROSLAM_SCAN_MATCHING_CONVERGENCE_EPSILON_FACTOR *
               occupancy->size / (1 << occupancy->max_depth),
           2.0f);

  printf("convergence epsilon: %.9g\n", epsilon);

  *score = -INFINITY;

  // gradient descent
  float step_size = 0.001f;
  float current_score;
  pose_t current_estimate = prior->pose;
  pose_t last_update = prior->pose;
  pose_t gradient = {0, 0, 0};

  unsigned short i = 0;
  for (; i < iterations; i++) {
    scan_matching_gradient(occupancy, scan, &current_estimate, &gradient,
                           &current_score);

    printf("gradient: %f %f %f\n", gradient.x, gradient.y, gradient.r);

    // if the score is better than the best score,
    // update the best score
    if (current_score > *score) {
      *score = current_score;
      *estimate = current_estimate;
      last_update = current_estimate;
    }

    // update the estimate using the gradient
    current_estimate.x -= step_size * gradient.x;
    current_estimate.y -= step_size * gradient.y;
    current_estimate.r -= 0.1f * step_size * gradient.r;

    float grad_norm = gradient.x * gradient.x + gradient.y * gradient.y +
                      gradient.r * gradient.r;
    if (grad_norm < epsilon) {
      break;
    }
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

  if (i == iterations || *score < 0) {
    printf("did not converge\n");
    return 0;
  } else {
    printf("converged in %d iterations\n", i);
    return 1;
  }
}
