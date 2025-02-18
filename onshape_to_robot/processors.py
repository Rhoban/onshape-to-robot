from .processor_merge_parts import ProcessorMergeParts
from .processor_scad import ProcessorScad
from .processor_simplify_stls import ProcessorSimplifySTLs
from .processor_fixed_links import ProcessorFixedLinks
from .processor_dummy_base_link import ProcessorDummyBaseLink

processors = [
    ProcessorScad,
    ProcessorMergeParts,
    ProcessorSimplifySTLs,
    ProcessorFixedLinks,
    ProcessorDummyBaseLink
]
