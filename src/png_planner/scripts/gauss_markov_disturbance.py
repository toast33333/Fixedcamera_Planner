#!/usr/bin/env python3
"""Small, deterministic first-order Gauss-Markov disturbance generator."""

import math

import numpy as np


class FirstOrderGaussMarkov:
    """Generate bounded colored noise with a requested stationary standard deviation.

    The exact discrete update is

        x[k+1] = exp(-dt/tau) x[k]
                 + sigma sqrt(1-exp(-2 dt/tau)) w[k]

    where w is unit Gaussian white noise.  Clipping is only a simulation safety
    guard; with the default 3-sigma limit it rarely changes mild disturbances.
    """

    def __init__(self, standard_deviation, time_constant_s, seed,
                 clip_sigma=3.0):
        self.standard_deviation = np.maximum(
            0.0, np.asarray(standard_deviation, dtype=float))
        if self.standard_deviation.ndim != 1:
            raise ValueError("standard_deviation must be a one-dimensional vector")
        self.time_constant_s = max(float(time_constant_s), 1.0e-3)
        self.clip_sigma = max(float(clip_sigma), 0.0)
        # RandomState accepts only uint32 seeds. Normalizing here also keeps the
        # derived seed streams (seed + 1, seed + 2) safe at the boundary.
        self.seed = int(seed) % (2 ** 32)
        self._rng = np.random.RandomState(self.seed)
        self._state = np.zeros_like(self.standard_deviation)

    @property
    def enabled(self):
        return bool(np.any(self.standard_deviation > 0.0))

    def reset(self):
        self._state.fill(0.0)

    def step(self, dt_s):
        dt_s = max(float(dt_s), 0.0)
        if dt_s <= 0.0 or not self.enabled:
            return self._state.copy()
        decay = math.exp(-dt_s / self.time_constant_s)
        innovation_scale = math.sqrt(max(0.0, 1.0 - decay * decay))
        innovation = (
            self.standard_deviation * innovation_scale *
            self._rng.normal(0.0, 1.0, size=self._state.shape))
        self._state = decay * self._state + innovation
        if self.clip_sigma > 0.0:
            limit = self.clip_sigma * self.standard_deviation
            self._state = np.maximum(-limit, np.minimum(self._state, limit))
        return self._state.copy()


class BoundedRandomWalk:
    """Generate a reproducible cumulative bias with an absolute safety bound.

    ``walk_density`` has units of output-units / sqrt(second).  The exact
    discrete increment is ``density * sqrt(dt) * N(0, 1)``.  Unlike the
    Gauss-Markov process above, this process does not decay toward zero, so it
    represents a slowly accumulating sensor bias.  ``absolute_limit`` is a
    simulation guard, not a stochastic restoring force.
    """

    def __init__(self, walk_density, seed, absolute_limit):
        self.walk_density = np.maximum(
            0.0, np.asarray(walk_density, dtype=float))
        self.absolute_limit = np.maximum(
            0.0, np.asarray(absolute_limit, dtype=float))
        if self.walk_density.ndim != 1:
            raise ValueError("walk_density must be a one-dimensional vector")
        if self.absolute_limit.shape != self.walk_density.shape:
            raise ValueError(
                "absolute_limit must have the same shape as walk_density")
        self.seed = int(seed) % (2 ** 32)
        self._rng = np.random.RandomState(self.seed)
        self._state = np.zeros_like(self.walk_density)

    @property
    def enabled(self):
        return bool(np.any(self.walk_density > 0.0))

    def reset(self):
        self._state.fill(0.0)

    def step(self, dt_s):
        dt_s = max(float(dt_s), 0.0)
        if dt_s <= 0.0 or not self.enabled:
            return self._state.copy()
        self._state += (
            self.walk_density * math.sqrt(dt_s) *
            self._rng.normal(0.0, 1.0, size=self._state.shape))
        self._state = np.maximum(
            -self.absolute_limit,
            np.minimum(self._state, self.absolute_limit))
        return self._state.copy()
