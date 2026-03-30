//! Shared simulation traits for batch/vectorized physics backends.
//!
//! Both `horus-sim3d` and `horus-mujoco` implement [`BatchPhysicsStep`] so that
//! RL training code can swap backends without changing the stepping loop.

/// Trait for vectorized physics environments (RL batch stepping).
///
/// Implementations hold N independent physics worlds that can be stepped in
/// parallel. Observations and actions are flat `f64` slices for zero-copy
/// interop with numpy / PyTorch.
///
/// # Example
///
/// ```ignore
/// let mut env: Box<dyn BatchPhysicsStep> = make_backend(model, 64);
/// let actions = vec![0.0; env.n_envs() * env.action_dim()];
/// let (obs, rewards, dones) = env.batch_step(&actions);
/// ```
pub trait BatchPhysicsStep {
    /// Step all environments with the given actions.
    ///
    /// - `actions`: flat `[n_envs * action_dim]`
    ///
    /// Returns `(observations, rewards, dones)`:
    /// - `observations`: flat `[n_envs * obs_dim]`
    /// - `rewards`: `[n_envs]`
    /// - `dones`: `[n_envs]`
    fn batch_step(&mut self, actions: &[f64]) -> (&[f64], &[f64], &[bool]);

    /// Reset specific environments by index.
    ///
    /// Returns observations for the reset environments (flat,
    /// `reset_indices.len() * obs_dim`).
    fn batch_reset(&mut self, reset_indices: &[usize]) -> Vec<f64>;

    /// Number of parallel environments.
    fn n_envs(&self) -> usize;

    /// Observation dimension per environment.
    fn obs_dim(&self) -> usize;

    /// Action dimension per environment.
    fn action_dim(&self) -> usize;
}
