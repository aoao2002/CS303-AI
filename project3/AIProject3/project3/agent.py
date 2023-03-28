import torch
import time
from typing import Tuple
from .src import RADIUS, N_CTPS, evaluate, compute_traj
from torch import nn
import os.path as osp
from functorch import vmap


class NeuralNetwork(nn.Module):
    def __init__(self):
        super(NeuralNetwork, self).__init__()
        self.flatten = nn.Flatten()
        self.linear_relu_stack = nn.Sequential(
            nn.Linear(256, 256),
            nn.ReLU(),
            nn.Linear(256, 256),
            nn.ReLU(),
            nn.Linear(256, 10)
        )

    def forward(self, x):
        x = self.flatten(x)
        logits = self.linear_relu_stack(x)
        return logits


class Agent:

    def __init__(self):
        """Initialize the agent, e.g., load the classifier model. """

        # TODO: prepare your agent here
        super()
        self.model = NeuralNetwork()
        model_path = osp.join(osp.dirname(__file__), "model.pth")
        self.model.load_state_dict(torch.load(model_path))

    def get_action(self,
                   target_pos: torch.Tensor,
                   target_features: torch.Tensor,
                   class_scores: torch.Tensor,
                   ) -> Tuple[torch.Tensor, torch.Tensor]:
        """Compute the parameters required to fire a projectile. 
        
        Args:
            target_pos: x-y positions of shape `(N, 2)` where `N` is the number of targets. 
            target_features: features of shape `(N, d)`.
            class_scores: scores associated with each class of targets. `(K,)` where `K` is the number of classes.
        Return: Tensor of shape `(N_CTPS-2, 2)`
            the second to the second last control points
        """
        assert len(target_pos) == len(target_features)

        # TODO: compute the firing speed and angle that would give the best score.
        t = time.time()
        # classify targets
        # target_features = self.pca.transform(target_features)
        # target_cls = self.model.predict(target_features)
        target_features = target_features
        target_cls = self.model(target_features)
        target_cls = torch.argmax(target_cls, dim=1)
        # initialize the control points using a random configuration
        best_ctps_inter = torch.randn((N_CTPS - 2, 2)) * torch.tensor([N_CTPS - 2, 2.]) + torch.tensor([1., -1.])
        best_traj = compute_traj(best_ctps_inter)
        max_score = evaluate(best_traj, target_pos, class_scores[target_cls], RADIUS)
        while (time.time() - t) < 0.26:
            # # random to get one point
            # ctps_inter = torch.rand((N_CTPS - 2, 2)) * torch.tensor([N_CTPS - 2, 2.]) + torch.tensor([1., -1.])
            # optimize the control points
            ctps_inter1 = [torch.randn((N_CTPS - 2, 2)) * torch.tensor([N_CTPS - 2, 2.]) + torch.tensor([1., -1.])
                           for _ in range(20)]
            ctps_inter2 = [torch.rand((N_CTPS - 2, 2)) * torch.tensor([N_CTPS - 2, 2.]) + torch.tensor([1., -1.])
                           for _ in range(20)]
            ctps_inter = torch.stack(ctps_inter1 + ctps_inter2)
            traj = vmap(compute_traj, in_dims=0)(ctps_inter)
            score = vmap(evaluate, in_dims=(0, None, None, None))(traj, target_pos, class_scores[target_cls], RADIUS)
            best_idx = torch.argmax(score)
            ctps_inter = ctps_inter[best_idx]
            ctps_inter.requires_grad = True
            lr = 1
            last_score = 0
            k = 0
            for i in range(20):
                # compute trajectory
                traj = compute_traj(ctps_inter)
                gra_score = evaluate_modify(traj, target_pos, class_scores[target_cls], RADIUS)
                act_score = evaluate(traj, target_pos, class_scores[target_cls], RADIUS)
                if act_score == last_score:
                    k += 1
                else:
                    k = 0
                if k > 2:
                    break  # stop if the score is not improving
                last_score = act_score
                if act_score > max_score:
                    max_score = act_score
                    best_ctps_inter = ctps_inter.data
                gra_score.backward()
                ctps_inter.data = ctps_inter.data + lr * ctps_inter.grad / torch.norm(
                    ctps_inter.grad)
                if i % 10 == 0:
                    lr /= 2
        # self.forward(target_features)
        return best_ctps_inter


def evaluate_modify(
        traj: torch.Tensor,
        target_pos: torch.Tensor,
        target_scores: torch.Tensor,
        radius: float,
) -> torch.Tensor:
    """Evaluate the trajectory and return the score it gets.

    Parameters
    ----------
    traj: Tensor of shape `(*, T, 2)`
        The discretized trajectory, where `*` is some batch dimension and `T` is the discretized time dimension.
    target_pos: Tensor of shape `(N, 2)`
        x-y positions of shape where `N` is the number of targets.
    target_scores: Tensor of shape `(N,)`
        Scores you get when the corresponding targets get hit.
    """
    cdist = torch.cdist(target_pos, traj)  # see https://pytorch.org/docs/stable/generated/torch.cdist.html
    d = cdist.min(-1).values
    hit = (d <= radius)
    d[hit] = 1
    d[~hit] = radius / d[~hit]
    value = torch.sum(d * target_scores, dim=-1)
    return value
