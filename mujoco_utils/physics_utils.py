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


def get_critical_damping(physics, joint_name, qpos0=None,
                         dof_name=None, actuator_name=None,
                         joint_spring=True, actuator_spring=True,
                         return_current_damping=False,
                         test_forces=None):
    """Calculate critical damping for a joint, possibly taking into account both
    joint stiffness spring and position actuator gainprm.
    
    Args:
        physics: physics instance.
        joint_name: Joint name to calculate critical damping for.
        qpos0: Pose to calculate effective inertia for. If not provided, the
            current physics.data.qpos is taken as qpos0.
        dof_name: Damping's DoF name, only used if return_current == True.
            None: use joint_name for dof_name.
        actuator_name: Actuator's name. None: use joint_name for actuator_name.
        joint_spring: Whether to use joint (stiffness) spring constant.
        actuator_spring: Whether to use actuator gainprm as spring constant.
        return_current_daping: Whether to return the current damping as well
            (for comparison, diagnostics, etc.)
        test_forces: Test forces to calculate effective inertia for. If not
            provided, a default range is used.
        
    Returns:
        Depending on return_current_damping, return critical_damping or 
            (critical_damping, current_damping).
    """
    # Prevent in-place changes to physics.
    physics = physics.copy()
    
    if qpos0 is None:
        qpos0 = physics.data.qpos.copy()
    if dof_name is None:
        dof_name = joint_name
    if actuator_name is None:
        actuator_name = joint_name
    if test_forces is None:
        # Three orders of magnitude default range.
        test_forces = np.logspace(-2, 1, 20)
    
    # Get effective inertia.
    inertia = []
    for force in test_forces:
        with physics.reset_context():
            physics.data.qpos = qpos0.copy()
        physics.named.data.qfrc_applied[joint_name] = force
        physics.step()
        acc = physics.named.data.qacc[joint_name].copy()
        assert len(acc) == 1, f'{joint_name} is not a hinge joint.'
        inertia.append(force/acc[0])
        
    mean_inertia = np.mean(inertia)
    # Assume effective inertia is not very different for all test forces.
    assert np.std(inertia) / mean_inertia < 0.2
    
    spring_const = 0.
    if joint_spring:
        spring_const = spring_const + physics.named.model.jnt_stiffness[joint_name]
    if actuator_spring:
        spring_const = (spring_const + 
                        physics.named.model.actuator_gainprm[actuator_name][0])
    critical_damping = 2 * np.sqrt(spring_const * mean_inertia)

    if return_current_damping:
        current_damping = physics.named.model.dof_damping[dof_name][0]
        return critical_damping, current_damping
    return critical_damping
