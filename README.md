# Heli3Dof

A simple 3‑degree‑of‑freedom helicopter model. The original implementation was
in MATLAB; this repository now provides a basic Python version with
visualisation utilities and a Jupyter notebook ready for Google Colab.

## Requirements

```
pip install -r requirements.txt
```

## Running the tests

```
pytest
```

## Using the notebook

Open `Heli3Dof.ipynb` in Jupyter or upload it to [Google Colab](https://colab.research.google.com/). It demonstrates how to compute the kinematics,
simulate a simple PD controller and visualise the result.


## New utilities for class notebooks

The package now includes two helper modules used in control/estimation classes:

- `heli3dof.control`: compact fuzzy-PD utilities and a pitch/roll controller that maps angle errors into front/back motor forces.
- `heli3dof.estimation`: generic EKF predict/update helpers with numerical Jacobians for nonlinear models.

These helpers are intentionally lightweight so they can be reused directly in notebooks for experiments and homework.
