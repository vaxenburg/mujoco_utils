"""Utilities to work with compiled mjcf.physics."""

import numpy as np


def joint_to_dof_id(physics: 'mjcf.Physics',
                    joint_name: str = None,
                    joint_id: int = None) -> int | list[int]:
    """Get degree-of-freedom index dof_id from joint_name or joint_id.
    
    dof_id can be used to index into qvel, qfrc_applied, qfrc_actuator, etc.

    Args:
        physics: An mjcf.physics instance.
        joint_name, joint_id: Either name or id of joint to get dof_id for.
            If both are provided, joint_id is ignored.

    Returns:
        dof_id for requested joint. For freejoint or slider, returns list of ids.
    """
    if joint_name is None and joint_id is None:
        raise ValueError('Either joint_name or joint_id should be provided.')
    if joint_name is not None:
        joint_id = physics.model.name2id(joint_name, 'joint')
    jnt_type = physics.model.jnt_type[joint_id]
    dof_id = physics.model.jnt_dofadr[joint_id]
    # Check if free or ball joint. Otherwise, slide and hidge have only one DoF.
    if jnt_type == 0:
        # Free joint.
        dof_id = [*range(dof_id, dof_id+6)]
    elif jnt_type == 1:
        # Ball joint.
        dof_id = [*range(dof_id, dof_id+3)]
    return dof_id


def is_position_actuator(physics: 'mjcf.physics',
                         actuator_id: int) -> bool:
    """Check if given actuator a position actuator parametrized as:
    biastype: "affine"
    gainprm: (kp, 0, 0)
    biasprm: (0, -kp, 0)    
    TODO: later could also consider biasprm: (0, -kp, -kv)
    """
    biastype = physics.model.actuator_biastype[actuator_id]
    gainprm = physics.model.actuator_gainprm[actuator_id]
    biasprm = physics.model.actuator_biasprm[actuator_id]
    print(gainprm[0], biasprm[1])
    return (biastype == 1 and
            np.isclose(gainprm[0], - biasprm[1]) and
            all(gainprm[1:] == 0) and
            biasprm[0] == 0 and
            all(biasprm[2:] == 0))


def get_enabled_observables(walker) -> dict:
    """Get dict of enabled observables from walker."""
    enabled_obs = {}
    for k, v in walker.observables._observables.items():
        if v.enabled:
            enabled_obs[k] = v
    return enabled_obs

