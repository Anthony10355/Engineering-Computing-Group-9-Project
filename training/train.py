"""Behaviour cloning: fit PlannerNet to the recorded expert trajectories."""

import sys
from pathlib import Path

import numpy as np
import torch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from planners.learned_example.features import FEATURE_VERSION, NUM_FEATURES   # noqa: E402
from planners.learned_example.model import PlannerNet                         # noqa: E402

torch.manual_seed(0)
data = np.load(ROOT / "training" / "data.npz")
X, Y = torch.from_numpy(data["X"]), torch.from_numpy(data["Y"])
assert X.shape[1] == NUM_FEATURES, "features.py changed since the data was recorded"
x_mean, x_std = X.mean(0), X.std(0)
x_std = torch.where(x_std < 1e-3, torch.ones_like(x_std), x_std)   # features that never varied in the data
y_mean, y_std = Y.mean(0), Y.std(0) + 1e-6                          # (e.g. no traffic lights) must not explode
Xn, Yn = (X - x_mean) / x_std, (Y - y_mean) / y_std

model = PlannerNet(NUM_FEATURES)
opt = torch.optim.Adam(model.parameters(), lr=1e-3)
for epoch in range(30):
    perm = torch.randperm(len(Xn))
    for i in range(0, len(Xn), 256):
        idx = perm[i:i + 256]
        loss = torch.nn.functional.mse_loss(model(Xn[idx]), Yn[idx])
        opt.zero_grad()
        loss.backward()
        opt.step()
    print(f"epoch {epoch}: loss {loss.item():.4f}")

torch.save({"model": model.state_dict(), "feature_version": FEATURE_VERSION,
            "x_mean": x_mean, "x_std": x_std, "y_mean": y_mean, "y_std": y_std},
           ROOT / "planners" / "learned_example" / "weights" / "model.pt")
