#!/usr/bin/env python3
"""
 * @file export_history_policy_onnx.py
 * @brief Rebuild the deployment ONNX (HistoryEncoder + actor) for the
 *        history_adaptation policy.
 *
 * The standard rsl_rl ONNX export only traces the actor MLP: its single input
 * is [policy_obs (86), history_latent (32)] = 118 dims, and the history encoder
 * is NOT part of the graph. Deployment therefore needs the encoder too.
 *
 * This script loads an rsl_rl checkpoint (logs/rsl_rl/history_adaptation/...),
 * rebuilds HistoryEncoder (TCN) + actor MLP from the source modules, and
 * exports ONE ONNX with:
 *
 *   inputs:  obs          [B, 86]   (policy obs, single step)
 *            obs_history  [B, 770]  (10 steps x 77 single-step obs)
 *   outputs: actions      [B, 23]   (12 leg pos + 4 wheel vel + 7 ee_ik)
 *
 * Usage (inside the training/deploy env with torch + rsl_rl available):
 *   PYTHONPATH=<path-to-rsl_rl>/rsl_rl python3 scripts/export_history_policy_onnx.py \
 *       --checkpoint logs/.../model_19999.pt --output policy/history_adaptation_full.onnx
"""

import argparse

import torch

from rsl_rl.networks.mlp import MLP
from rsl_rl.networks.history_encoder import HistoryEncoder


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--checkpoint", required=True, help="rsl_rl model_*.pt checkpoint")
    parser.add_argument("--output", required=True, help="output ONNX path")
    parser.add_argument("--num_actions", type=int, default=23)
    parser.add_argument("--policy_obs_dim", type=int, default=86,
                        help="single-step policy obs dim (3+3+3+22+22+23+7+3)")
    parser.add_argument("--history_length", type=int, default=10)
    parser.add_argument("--history_obs_dim", type=int, default=77,
                        help="single-step history obs dim (3+3+24+24+23)")
    parser.add_argument("--latent_dim", type=int, default=32)
    parser.add_argument("--actor_hidden", nargs="+", type=int, default=[512, 256, 128])
    args = parser.parse_args()

    ck = torch.load(args.checkpoint, map_location="cpu", weights_only=False)
    sd = ck["model_state_dict"]

    encoder = HistoryEncoder(
        num_single_step_obs=args.history_obs_dim,
        history_length=args.history_length,
        latent_dim=args.latent_dim,
        hidden_channels=(32, 32, 32),
        kernel_sizes=(4, 3, 2),
        strides=(2, 1, 1),
        activation="elu",
    )
    actor = MLP(args.policy_obs_dim + args.latent_dim, args.num_actions,
                tuple(args.actor_hidden), "elu")

    enc_sd = {k[len("history_encoder."):]: v for k, v in sd.items() if k.startswith("history_encoder.")}
    act_sd = {k[len("actor."):]: v for k, v in sd.items() if k.startswith("actor.")}
    assert enc_sd, "no history_encoder.* weights found in the checkpoint"
    assert act_sd, "no actor.* weights found in the checkpoint"

    missing, unexpected = encoder.load_state_dict(enc_sd)
    assert not missing and not unexpected, f"encoder load: {missing} {unexpected}"
    missing, unexpected = actor.load_state_dict(act_sd)
    assert not missing and not unexpected, f"actor load: {missing} {unexpected}"

    class DeployNet(torch.nn.Module):
        def __init__(self, encoder_: HistoryEncoder, actor_: MLP):
            super().__init__()
            self.encoder = encoder_
            self.actor = actor_

        def forward(self, obs: torch.Tensor, obs_history: torch.Tensor) -> torch.Tensor:
            latent = self.encoder(obs_history)
            return self.actor(torch.cat([obs, latent], dim=-1))

    net = DeployNet(encoder, actor).eval()
    dummy_obs = torch.randn(1, args.policy_obs_dim)
    dummy_hist = torch.randn(1, args.history_length * args.history_obs_dim)

    torch.onnx.export(
        net,
        (dummy_obs, dummy_hist),
        args.output,
        input_names=["obs", "obs_history"],
        output_names=["actions"],
        opset_version=17,
        dynamic_axes={
            "obs": {0: "batch"},
            "obs_history": {0: "batch"},
            "actions": {0: "batch"},
        },
    )
    print(f"[export_history_policy_onnx] exported {args.output}")
    print(f"  policy_obs={args.policy_obs_dim}, history={args.history_length}x{args.history_obs_dim}, "
          f"latent={args.latent_dim}, actions={args.num_actions}")


if __name__ == "__main__":
    main()
