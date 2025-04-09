from .processor_merge_parts import ProcessorMergeParts
from .processor_scad import ProcessorScad
from .processor_simplify_stls import ProcessorSimplifySTLs
from .processor_fixed_links import ProcessorFixedLinks
from .processor_dummy_base_link import ProcessorDummyBaseLink
from .processor_convex_decomposition import ProcessorConvexDecomposition
from .processor_collision_as_visual import ProcessorCollisionAsVisual
from .processor_no_collision_meshes import ProcessorNoCollisionMeshes
from .processor_ball_to_euler import ProcessorBallToEuler

default_processors = [
    ProcessorBallToEuler,
    ProcessorScad,
    ProcessorMergeParts,
    ProcessorSimplifySTLs,
    ProcessorFixedLinks,
    ProcessorDummyBaseLink,
    ProcessorConvexDecomposition,
    ProcessorNoCollisionMeshes,
    ProcessorCollisionAsVisual,
]
