from .processor_merge_parts import ProcessorMergeParts
from .processor_scad import ProcessorScad
from .processor_simplify_stls import ProcessorSimplifySTLs
from .processor_fixed_links import ProcessorFixedLinks

processors = [
    ProcessorScad,
    ProcessorMergeParts,
    ProcessorSimplifySTLs,
    ProcessorFixedLinks,
]
