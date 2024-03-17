# Simple particle filter based SLAM in C

### My notes

- Particle filter is a Monte Carlo method for estimating the state of a system.

Key idea:

- You can use multiple samples to represent an arbitrary probability distribution - it works for any distribution, not just Gaussian, unlike the Kalman filter. (non-parametric)
- You build a particle set of weighted particles, where each particle represents a possible state of the system.
- Each particle contains a _hypothesis_ of the current state and an _importance weight_ that represents the probability of the hypothesis being true.
- The particles approximate the posterior distribution of the state of the system.
- Here is an example of approximating a Gaussian distribution using dirac delta functions showing the similarity of the distribution and the particle set when integrated:
  ![Particle filter](https://i.stack.imgur.com/9TouJ.png)

Particle filter algorithm for localisation:

1. Initialise the particles with a random state.
2. Sample the particles from the proposal distribution (e.g. the motion model).
3. Update the importance weights of the particles using the likelihood of the measurement (e.g. the sensor model). The likelihood is the probability of the measurement given the state of the system: p(z | x).
4. Update the state estimate using the weighted particles.
5. Resample the particles based on their weights to avoid degeneracy (i.e. the weights of most particles are close to zero so they don't contribute to the estimate). This is done by drawing a new set of particles from the current set with replacement, where the probability of drawing a particle is proportional to its weight.
6. Repeat.

Outline of the sampling process:
![Particle filter method](https://www.lancaster.ac.uk/stor-i-student-sites/martin-dimitrov/wp-content/uploads/sites/31/2021/05/sampling.png)

- Obviously much easier with a known map. But we could use a SLAM algorithm to build the map as we go.

### References

- https://docs.ufpr.br/~danielsantos/ProbabilisticRobotics.pdf
- [Particle Filters: The Good, The Bad, The Ugly](https://www.cs.cmu.edu/~16831-f14/notes/F11/16831_lecture04_tianyul.pdf)
- [Particle filter](https://en.wikipedia.org/wiki/Particle_filter)
- [Dirac delta function](https://en.wikipedia.org/wiki/Dirac_delta_function) (infinite peak at 0, area under the curve is 1)
- [Stochastic universal sampling](https://en.wikipedia.org/wiki/Stochastic_universal_sampling)
