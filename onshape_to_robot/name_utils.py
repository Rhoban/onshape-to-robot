import re

def clean_configuration_suffix(name):
    """
    Remove configuration suffixes from names
    Examples:
    - part_name__configuration_default -> part_name
    - part_name__configuration_screwless_2 -> part_name
    - part_name_configuration_default -> part_name
    """
    # Match either double underscore or single underscore followed by 'configuration_'
    # and capture everything until the end of the string or next underscore
    return re.sub(r'(__|_)configuration_[^_]*(_\d+)?', '', name)


def clean_parent_prefix(name, parent_name):
    """
    Remove parent name prefix from part/joint names if they start with parent name
    Examples:
    - parent_name_actual_name -> actual_name
    - parent__name__actual_name -> actual_name
    """
    if not parent_name:
        return name
    
    # Normalize parent name to handle different separator styles
    parent_slug = re.sub(r'_+', '_', parent_name.lower())
    
    # Try different separation patterns
    patterns = [
        f"^{parent_slug}_+",  # parent_name_actual_name
        f"^{parent_slug.replace('_', '__')}__"  # parent__name__actual_name
    ]
    
    result = name
    for pattern in patterns:
        if re.search(pattern, name.lower()):
            result = re.sub(pattern, '', name, flags=re.IGNORECASE)
            break
    
    return result


def clean_name(name, parent_name=None, clean_config=False, remove_parent=False):
    """
    Clean a name according to configuration options
    
    Args:
        name (str): The name to clean
        parent_name (str, optional): Parent link/assembly name for prefix removal
        clean_config (bool): Whether to remove configuration suffixes
        remove_parent (bool): Whether to remove parent prefixes
    
    Returns:
        str: The cleaned name
    """
    result = name
    
    # First apply parent prefix removal if enabled
    if remove_parent and parent_name:
        result = clean_parent_prefix(result, parent_name)
    
    # Then apply configuration suffix removal if enabled
    if clean_config:
        result = clean_configuration_suffix(result)
    
    return result
