"""Lightweight MuJoCo visual-state recording and offline rendering.

Keep package import side-effect free: the renderer must select ``MUJOCO_GL``
before importing MuJoCo in a headless process.
"""
